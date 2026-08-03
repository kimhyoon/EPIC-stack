/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-29 16:54:46
 * @LastEditTime: 2024-03-11 13:22:44
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>
#include <epic_planner/fast_exploration_manager.h>
#include <plan_manage/planner_manager.h>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <iomanip>
#include <traj_utils/planning_visualization.h>
using Eigen::Vector3d;
using Eigen::Vector4d;
bool debug_planner;
typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

// A NO_FRONTIER result right after the trigger usually means the map/pointcloud
// hasn't been published yet (so no frontiers exist *yet*), not that exploration
// is actually done. Suppress the FINISH transition until either the planner has
// succeeded at least once (frontiers confirmed to exist) or a warmup timeout
// elapses (so a genuinely empty/enclosed map still terminates instead of hanging).
bool FastExplorationFSM::explorationReallyFinished() {
  if (frontiers_ever_seen_)
    return true;
  if (explore_start_time_.toSec() < 1e-6)
    explore_start_time_ = ros::Time::now();  // start the warmup clock on first attempt
  return (ros::Time::now() - explore_start_time_).toSec() > explore_warmup_timeout_;
}

// 동점 판정 여유. 그래프 거리[m]와 제곱거리[m^2] 양쪽에 쓰는데, 1e-9 는 어느
// 쪽으로 봐도 기하학적으로 의미 없는 차이(<0.03mm)라 "진짜 동점" 만 잡는다.
static constexpr double kDistTieEps = 1e-9;

// 좌표 사전순(x -> y -> z). 동점일 때 결정적으로 하나를 고르기 위한 기준이다.
// 노드 포인터(주소) 비교는 실행마다 결과가 달라지므로 절대 쓰면 안 된다.
static bool lexLess(const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
  if (a.x() != b.x())
    return a.x() < b.x();
  if (a.y() != b.y())
    return a.y() < b.y();
  return a.z() < b.z();
}

bool FastExplorationFSM::insideExplorationBox(const Eigen::Vector3f &p) const {
  const auto &lp = planner_manager_->lidar_map_interface_->lp_;
  if (lp->box_num_ <= 0)
    return true;
  for (int i = 0; i < lp->box_num_; i++) {
    const Eigen::Vector3f &lo = lp->global_box_min_boundary_vec_[i];
    const Eigen::Vector3f &hi = lp->global_box_max_boundary_vec_[i];
    if ((p.array() >= lo.array()).all() && (p.array() <= hi.array()).all())
      return true;
  }
  return false;
}

// odom_node_ 를 시작점으로 topology graph 에 Dijkstra 를 한 번 흘려, 도달 가능한
// 노드 전부와 그 그래프 거리[m]를 돌려준다.
//
// 노드마다 getPathCost() 를 부르지 않는 이유:
//   1) 도달 불가 시 2e3 + 직선거리를 돌려주므로(fast_exploration_manager.cpp), 비용
//      최댓값을 그냥 뽑으면 반드시 "도달 불가 노드" 가 1등이 된다.
//   2) 내부 topoSearch 는 10ms 타임아웃이라 먼 노드일수록 거짓 NO_PATH 가 난다.
//      우리는 하필 가장 먼 노드를 찾는 중이라 이 함정에 정면으로 걸린다.
// Dijkstra 는 둘 다 없다. 못 닿는 성분은 아예 방문되지 않고, 타임아웃 개념이 없다.
// 엣지 가중치(weight_)는 calculatePathCost = 순수 유클리드 경로 길이[m] 이므로
// 거리값은 미터 단위로 그대로 해석된다.
//
// next_goal_node_(가상 goal adapter)는 그래프에서 떼어내지 않는다. 예전 구현은
// Dijkstra 전에 그 이웃 연결을 전부 끊었는데, planGlobalPath 가 NO_FRONTIER 로
// 조기 리턴하면 updateGoalNode() 까지 못 가서 goal 이 끊긴 채로 남는다. 여기서는
// "후보에서만 제외" 하면 충분하다 — 그 위치의 실제 topo node 는 별도 포인터라
// 여전히 후보로 남고, 가상 goal 을 경유하는 경로는 updateGoalNode 가 A* 로 검증한
// 실제 통과 가능 경로라 도달성 판정을 왜곡하지 않는다.
bool FastExplorationFSM::computeReachableNodes(
    std::unordered_map<TopoNode::Ptr, double> &dist) {
  dist.clear();
  auto graph = planner_manager_->topo_graph_;
  if (!graph)
    return false;
  TopoNode::Ptr odom_node = graph->odom_node_;
  if (!odom_node)
    return false;

  std::priority_queue<std::pair<double, TopoNode::Ptr>,
                      std::vector<std::pair<double, TopoNode::Ptr>>,
                      std::greater<std::pair<double, TopoNode::Ptr>>>
      pq;
  dist[odom_node] = 0.0;
  pq.push({0.0, odom_node});

  while (!pq.empty()) {
    double d = pq.top().first;
    TopoNode::Ptr n = pq.top().second;
    pq.pop();
    if (d > dist[n] + 1e-6)
      continue;  // stale entry
    for (auto &nbr : n->neighbors_) {
      if (!nbr)
        continue;
      auto w_it = n->weight_.find(nbr);
      if (w_it == n->weight_.end())
        continue;
      double nd = d + w_it->second;
      auto it = dist.find(nbr);
      if (it == dist.end() || nd < it->second) {
        dist[nbr] = nd;
        pq.push({nd, nbr});
      }
    }
  }
  return dist.size() > 1;
}

bool FastExplorationFSM::isNearFlownPath(const Eigen::Vector3f &p) const {
  if (early_finish_visited_radius_ <= 0.0)
    return false;
  const float r2 = static_cast<float>(early_finish_visited_radius_ *
                                      early_finish_visited_radius_);
  for (const auto &h : planner_manager_->topo_graph_->history_odom_nodes_) {
    if (h && (h->center_ - p).squaredNorm() < r2)
      return true;
  }
  return false;
}

// EFP 최초 선정 (하이브리드 2단 판정의 2단).
// 후보 = 도달 가능 && exploration box 안 && 미방문(비행경로 반경 밖). 이미
// 지나가며 관측한 곳으로 probe 를 보내면 관측 이득이 없다.
// 랭킹 = 탐사 시작점(expl_origin_) 기준 유클리드 거리 최대. "지금 위치에서
// 그래프 거리 최대" 는 제자리 셔플로도 커질 수 있는데, 구제의 목적이 "원점에서
// 진출해 관측을 늘리는 것" 이므로 트리거(max_displacement_)와 같은 기준점을 쓴다.
// dist_out 은 planGlobalPath/로그가 쓰는 그래프 거리라 선정 노드의 것을 그대로
// 반환한다 (랭킹 기준과 다름에 주의).
bool FastExplorationFSM::selectFarthestReachableNode(TopoNode::Ptr &node_out,
                                                     double &dist_out,
                                                     bool require_gain) {
  node_out = nullptr;
  dist_out = 0.0;
  std::unordered_map<TopoNode::Ptr, double> dist;
  if (!computeReachableNodes(dist))
    return false;

  TopoNode::Ptr odom_node = planner_manager_->topo_graph_->odom_node_;
  TopoNode::Ptr virtual_goal = expl_manager_->ed_->next_goal_node_;
  TopoNode::Ptr best = nullptr;
  double best_origin_dist = -1.0;
  double best_graph_dist = 0.0;
  // unordered_map 순회 순서는 포인터 주소에 좌우돼 실행마다 다르다. 동점(부동소수
  // 오차 범위 포함)은 좌표 사전순으로 깨야 같은 입력에 항상 같은 EFP 가 나온다.
  auto pick = [&](bool skip_visited) {
    best = nullptr;
    best_origin_dist = -1.0;
    best_graph_dist = 0.0;
    for (const auto &kv : dist) {
      const TopoNode::Ptr &n = kv.first;
      if (!n || n == odom_node || n == virtual_goal)
        continue;
      if (!insideExplorationBox(n->center_))
        continue;
      if (skip_visited && isNearFlownPath(n->center_))
        continue;
      const double origin_dist = (n->center_ - expl_origin_).norm();
      bool better;
      if (!best)
        better = true;
      else if (origin_dist > best_origin_dist + kDistTieEps)
        better = true;
      else if (origin_dist < best_origin_dist - kDistTieEps)
        better = false;
      else
        better = lexLess(n->center_, best->center_);
      if (better) {
        best = n;
        best_origin_dist = origin_dist;
        best_graph_dist = kv.second;
      }
    }
  };

  pick(true);
  // 강제 요청은 항상 시도한다: 전부 방문한 그래프라면 방문 필터를 풀고 재선정.
  if (!best && !require_gain)
    pick(false);
  ROS_INFO("[EARLY_FINISH] dijkstra reached %zu nodes, "
           "origin_dist=%.2fm graph_dist=%.2fm (max_disp=%.2fm)",
           dist.size(), best_origin_dist, best_graph_dist, max_displacement_);
  if (!best)
    return false;
  if (require_gain &&
      best_origin_dist < max_displacement_ + early_finish_probe_min_gain_) {
    ROS_INFO("[EARLY_FINISH] best candidate gains nothing "
             "(origin_dist=%.2fm < max_disp=%.2fm + gain=%.2fm) -> no probe",
             best_origin_dist, max_displacement_, early_finish_probe_min_gain_);
    return false;
  }
  node_out = best;
  dist_out = best_graph_dist;
  return true;
}

void FastExplorationFSM::publishEarlyFinishStatus(
    const string &status, const string &detail) {
  early_finish_status_ = status;
  std_msgs::String msg;
  msg.data = detail.empty() ? status : status + " | " + detail;
  early_finish_state_pub_.publish(msg);
}

// 현재 EFP 를 planGlobalPath(ed_) 와 rviz 마커로 밀어넣는다. EFP 가 없으면 끈다.
void FastExplorationFSM::pushEarlyFinishProbe() {
  auto ed = expl_manager_->ed_;
  auto frt = expl_manager_->frontier_manager_ptr_;
  if (early_finish_active_ && early_finish_node_) {
    ed->efp_active_ = true;
    ed->efp_node_ = early_finish_node_;
    ed->efp_pos_ = early_finish_target_pos_;
    ed->efp_yaw_ = early_finish_yaw_;
    ed->efp_graph_dist_ = early_finish_graph_distance_;
    if (frt)
      frt->setEarlyFinishMarker(true, early_finish_target_pos_,
                                early_finish_yaw_);
  } else {
    ed->efp_active_ = false;
    ed->efp_node_ = nullptr;
    ed->efp_graph_dist_ = 0.0;
    ed->efp_tour_index_ = -1;
    if (frt)
      frt->setEarlyFinishMarker(false, early_finish_target_pos_,
                                early_finish_yaw_);
  }
}

// EFP 설치(최초 1회) — 여기서 그래프를 수술하지 않는다. EFP 는 그냥 "TSP 후보
// 목록의 원소 하나" 이고, goal 연결은 평소처럼 updateGoalNode() 가 처리한다.
bool FastExplorationFSM::installEarlyFinishProbe(const TopoNode::Ptr &node,
                                                 double graph_distance) {
  if (!node || node == planner_manager_->topo_graph_->odom_node_ ||
      node == expl_manager_->ed_->next_goal_node_)
    return false;

  early_finish_node_ = node;
  early_finish_target_pos_ = node->center_;
  early_finish_graph_distance_ = graph_distance;
  early_finish_rebind_count_ = 0;
  early_finish_active_ = true;

  // yaw 는 "EFP 로 가는 진행 방향". topo skeleton node 의 yaw_ 는 아무도 채우지
  // 않아 항상 0(=+x) 이라 그대로 쓰면 엉뚱한 방향을 본다.
  Eigen::Vector3f dir = early_finish_target_pos_ - fd_->odom_pos_;
  early_finish_yaw_ = (dir.head<2>().norm() > 1e-3)
                          ? std::atan2(dir.y(), dir.x())
                          : fd_->odom_yaw_;
  pushEarlyFinishProbe();

  // 다음 planGlobalPath(<=0.2s) 가 EFP 를 포함한 제대로 된 tour 를 만들 때까지의
  // 공백을 메운다. 이 사이에 로컬 계획이 먼저 돌면 global_tour_ 가 비어
  // NO_FRONTIER -> FINISH 로 새기 때문이다. 특별한 수술이 아니라, 뷰포인트가
  // 하나뿐일 때 planGlobalPath 가 하는 것과 똑같은 2-노드 tour + updateGoalNode.
  auto ed = expl_manager_->ed_;
  ed->global_tour_.clear();
  ed->global_tour_.push_back(fd_->odom_pos_);
  ed->global_tour_.push_back(early_finish_target_pos_);
  ed->efp_tour_index_ = 1;
  planner_manager_->local_data_.end_yaw_ = early_finish_yaw_;
  expl_manager_->updateGoalNode();
  if (planner_manager_->graph_visualizer_) {
    planner_manager_->graph_visualizer_->vizTour(ed->global_tour_,
                                                 VizColor::BLUE, "global");
    planner_manager_->graph_visualizer_->vizTour(ed->global_tour_,
                                                 VizColor::MAGNA, "global_efp");
  }

  char detail[192];
  snprintf(detail, sizeof(detail), "efp=(%.2f,%.2f,%.2f) graph_dist=%.2fm",
           early_finish_target_pos_.x(), early_finish_target_pos_.y(),
           early_finish_target_pos_.z(), graph_distance);
  publishEarlyFinishStatus("PROBE_INSTALLED", detail);
  return true;
}

// 사건 1 — 도달. FSM 틱마다 상시 감시한다(상태 분기가 아니라 감시자).
// 도달하면 EFP 가 후보에서 빠지고, 그 뒤는 원래 흐름 그대로다:
// 남은 vp 가 있으면 계속 탐사, 없으면 곧바로 NO_FRONTIER -> FINISH -> RTH.
void FastExplorationFSM::checkEarlyFinishReached() {
  if (!early_finish_active_ || !fd_->have_odom_)
    return;
  const double d = (early_finish_target_pos_ - fd_->odom_pos_).norm();
  if (!std::isfinite(d) || d >= early_finish_reach_tol_)
    return;

  char detail[192];
  snprintf(detail, sizeof(detail),
           "efp=(%.2f, %.2f, %.2f) dist=%.2fm tol=%.2fm rebinds=%d",
           early_finish_target_pos_.x(), early_finish_target_pos_.y(),
           early_finish_target_pos_.z(), d, early_finish_reach_tol_,
           early_finish_rebind_count_);
  clearEarlyFinishPath("TARGET_REACHED");
  elog_.log("EARLY_FINISH",
            "probe reached -> normal exploration / RTH unblocked", detail, 0.0,
            EventLogger::L_WARN, true);
  ROS_WARN("\033[33m[EARLY_FINISH] probe reached (%.2fm < %.2fm)\033[0m", d,
           early_finish_reach_tol_);
}

// 매 전역계획 직전에 한 번. 도달 판정 -> 연결성 확인/재바인딩 -> planner 로 push.
// 사건 3(topo graph 끊김)은 별도 이벤트 감지 없이 여기서 매 주기 처리된다.
void FastExplorationFSM::updateEarlyFinishProbe() {
  if (!early_finish_active_) {
    pushEarlyFinishProbe();  // ed_ 를 확실히 꺼둔다
    return;
  }
  checkEarlyFinishReached();
  if (!early_finish_active_)
    return;

  std::unordered_map<TopoNode::Ptr, double> dist;
  if (!computeReachableNodes(dist)) {
    // 그래프가 통째로 비었다(odom_node 이웃 없음). 재바인딩할 대상이 없으므로
    // 좌표만 들고 기다린다 — 다음 주기에 그래프가 살아나면 자동 복구된다.
    publishEarlyFinishStatus("ACTIVE", "no reachable topology node this cycle");
    pushEarlyFinishProbe();
    return;
  }

  auto it = early_finish_node_ ? dist.find(early_finish_node_) : dist.end();
  if (it != dist.end()) {
    early_finish_graph_distance_ = it->second;
    early_finish_target_pos_ = early_finish_node_->center_;
    pushEarlyFinishProbe();
    return;
  }

  // --- 사건 3: EFP 가 odom_node 와 끊겼다 -> 그 자리에서 재바인딩 ---
  // 기준은 "odom 에서 가장 먼" 이 아니라 **"도달 가능한 것들 중 옛 EFP 좌표에
  // 최근접"** 이다. 옛 좌표는 노드가 죽어도 남아 있는 유일한 기준점이다.
  const Eigen::Vector3f old_pos = early_finish_target_pos_;
  TopoNode::Ptr odom_node = planner_manager_->topo_graph_->odom_node_;
  TopoNode::Ptr virtual_goal = expl_manager_->ed_->next_goal_node_;
  TopoNode::Ptr best = nullptr;
  double best_d2 = std::numeric_limits<double>::max();
  double best_graph_dist = 0.0;
  for (const auto &kv : dist) {
    const TopoNode::Ptr &n = kv.first;
    if (!n || n == odom_node || n == virtual_goal)
      continue;
    const double d2 = (n->center_ - old_pos).squaredNorm();
    // 동점 tie-break 는 좌표 사전순 — unordered_map 순회 순서(=포인터 주소)에
    // 의존하면 같은 입력에도 실행마다 다른 EFP 가 나온다.
    bool better;
    if (!best)
      better = true;
    else if (d2 < best_d2 - kDistTieEps)
      better = true;
    else if (d2 > best_d2 + kDistTieEps)
      better = false;
    else
      better = lexLess(n->center_, best->center_);
    if (better) {
      best_d2 = d2;
      best = n;
      best_graph_dist = kv.second;
    }
  }
  if (!best) {
    publishEarlyFinishStatus("ORPHANED", "no rebind candidate; keeping old EFP");
    pushEarlyFinishProbe();
    return;
  }

  early_finish_node_ = best;
  early_finish_target_pos_ = best->center_;
  early_finish_graph_distance_ = best_graph_dist;
  early_finish_rebind_count_++;
  Eigen::Vector3f dir = early_finish_target_pos_ - fd_->odom_pos_;
  early_finish_yaw_ = (dir.head<2>().norm() > 1e-3)
                          ? std::atan2(dir.y(), dir.x())
                          : fd_->odom_yaw_;
  pushEarlyFinishProbe();

  char detail[224];
  snprintf(detail, sizeof(detail),
           "old=(%.2f, %.2f, %.2f) new=(%.2f, %.2f, %.2f) shift=%.2fm "
           "graph_dist=%.2fm count=%d",
           old_pos.x(), old_pos.y(), old_pos.z(), early_finish_target_pos_.x(),
           early_finish_target_pos_.y(), early_finish_target_pos_.z(),
           std::sqrt(best_d2), best_graph_dist, early_finish_rebind_count_);
  publishEarlyFinishStatus("EFP_REBOUND", detail);
  elog_.log("EARLY_FINISH", "EFP_REBOUND (topology link lost)", detail, 0.0,
            EventLogger::L_WARN, true);
  ROS_WARN("\033[33m[EARLY_FINISH] EFP rebound: %s\033[0m", detail);
}

void FastExplorationFSM::clearEarlyFinishPath(const string &status) {
  early_finish_active_ = false;
  early_finish_forced_attempt_ = false;
  early_finish_graph_distance_ = 0.0;
  early_finish_node_ = nullptr;
  early_finish_rebind_count_ = 0;
  pushEarlyFinishProbe();  // ed_ off + EFP 화살표 off
  // EFP leg 마커를 반드시 지운다 (유령 마커 방지). ns 가 다르므로 정상 tour("global")
  // 는 건드리지 않는다.
  if (planner_manager_ && planner_manager_->graph_visualizer_)
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::MAGNA,
                                                 "global_efp");
  publishEarlyFinishStatus(status);
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e) {
  pubState();

  // ---- reactive-avoidance hand-off (Phase 2) ------------------------------
  // The px4_ctrl_bridge MUX overrides EPIC's command with the reactive escape
  // setpoint while /FSM_flag_avoidance==1, so the drone leaves EPIC's planned
  // path. EPIC keeps planning throughout (this hook never stops it), but we
  // force every replan to anchor to the drone's ACTUAL pose (static_state_)
  // instead of a predicted point on the old trajectory. On release we force one
  // fresh replan so the trajectory handed back to PX4 starts where the drone
  // actually is -> no snap-back toward the obstacle.
  const bool avoiding =
      avoidance_enabled_ && have_avoid_flag_ && (avoid_flag_ == 1) &&
      ((ros::Time::now() - last_avoid_flag_stamp_).toSec() < avoid_flag_timeout_);
  const bool mission_active =
      (state_ == EXEC_TRAJ || state_ == PLAN_TRAJ_EXP || state_ == PLAN_TRAJ_RTH);
  if (avoiding && mission_active)
    fd_->static_state_ = true;
  if (avoiding_prev_ && !avoiding && fd_->trigger_ && mission_active) {
    fd_->static_state_ = true;
    EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
    transitState(next_state, "avoidance released: replan from current pose", true);
  }
  // Avoidance 이벤트: 발동/해제 에지에서 기록 (상태 지속 중 반복은 dedup이 억제).
  // 빠른 Activated<->Deactivated 플래핑은 로거의 사이클 억제가 걸러준다.
  if (avoiding && !avoiding_prev_) {
    avoid_on_t_ = ros::Time::now();
    char d[128];
    snprintf(d, sizeof(d),
             "obstacle close, reactive layer overrides cmd | pos=(%.2f, %.2f, %.2f)",
             fd_->odom_pos_.x(), fd_->odom_pos_.y(), fd_->odom_pos_.z());
    elog_.log("Avoidance:", "Activated", d, 0.0, EventLogger::L_WARN);
  } else if (!avoiding && avoiding_prev_) {
    char d[96];
    snprintf(d, sizeof(d), "released, replan from current pose | duration=%.1fs",
             (ros::Time::now() - avoid_on_t_).toSec());
    elog_.log("Avoidance:", "Deactivated", d);
  }
  avoiding_prev_ = avoiding;

  // STUCK 감시: 미션 상태인데 장시간(>8s) 제자리면 사유와 함께 이벤트.
  // (자동 회복은 하지 않음 — 진단 전용. INC1/2에서 조종자 개입 전 10~19s 무이동.)
  if (fd_->have_odom_ && fd_->trigger_) {
    const Eigen::Vector3d cur = fd_->odom_pos_.cast<double>();
    if (stuck_ref_t_.toSec() < 1e-6 || (cur - stuck_ref_pos_).norm() > 0.3) {
      stuck_ref_pos_ = cur;
      stuck_ref_t_ = ros::Time::now();
    } else if (mission_active || state_ == CAUTION) {
      const double still_s = (ros::Time::now() - stuck_ref_t_).toSec();
      if (still_s > 8.0) {
        char d[160];
        snprintf(d, sizeof(d), "pos=(%.2f, %.2f, %.2f) stationary_for=%.0fs", cur.x(),
                 cur.y(), cur.z(), still_s);
        elog_.log("STUCK",
                  "no motion >8s in " + fd_->state_str_[int(state_)] +
                      " | last-local: " + (local_reason_.empty() ? "OK" : local_reason_),
                  d, 5.0, EventLogger::L_ERROR);
      }
    }
  }

  // EFP 도달 감시 (사건 1). 상태 분기가 아니라 상시 감시자다 — EXEC_TRAJ 중에
  // 지나가며 닿는 경우까지 잡아야 하고, 도달 즉시 후보에서 빠져야 다음 전역계획이
  // 자연스럽게 "남은 vp / 없으면 종료" 흐름으로 넘어간다.
  checkEarlyFinishReached();

  if (early_finish_force_requested_) {
    early_finish_force_requested_ = false;
    early_finish_forced_attempt_ = true;
    explore_finished_ = false;
    finish_hover_start_ = ros::Time(0);
    transitState(EARLY_FINISH, "/srv_early_finish");
  }

  switch (state_) {
  case INIT: {
    if (!fd_->have_odom_) {
      ROS_WARN_THROTTLE(1.0, "no odom.");
      return;
    }
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    ROS_WARN_THROTTLE(5.0, "wait for trigger.");
    break;
  }

  case TAKEOFF_HOVER: {
    // Triggered -> climb to the configured altitude and hold, then auto-start
    // exploration once odom confirms the drone is stable near that altitude.
    if (!fd_->have_odom_)
      return;

    // Stream the hover setpoint (hold x,y,yaw; target altitude) at the FSM rate so
    // px4_ctrl_bridge keeps it "fresh" and forwards it to PX4.
    pubHoverCmd();

    double z_err = std::fabs((double)fd_->odom_pos_.z() - takeoff_anchor_.z());
    double speed = fd_->odom_vel_.norm();
    bool reached = (z_err < fp_->takeoff_reach_tol_) && (speed < fp_->takeoff_settle_vel_);

    ros::Time now = ros::Time::now();
    if (reached) {
      if (hover_stable_since_.toSec() < 1e-6)
        hover_stable_since_ = now;  // start the settle timer
      if ((now - hover_stable_since_).toSec() >= fp_->takeoff_settle_time_) {
        fd_->static_state_ = true;  // first exploration traj anchors to current pose
        leaveTakeoffHover("takeoff: altitude reached & stable");
        break;
      }
    } else {
      hover_stable_since_ = ros::Time(0);  // not stable -> reset settle timer
    }

    // Safety: never wait forever -- but never start exploration from a non-airborne
    // pose either. If we time out while roughly at altitude (allow up to 3x the reach
    // tolerance to absorb odom/LIO z drift) and not climbing fast, proceed. Otherwise the
    // climb genuinely failed (not armed / not OFFBOARD, thrust-limited stall, bad z odom)
    // -> keep holding the climb setpoint and shout, rather than commanding lateral motion
    // from the ground.
    if ((now - hover_enter_time_).toSec() > fp_->takeoff_timeout_) {
      const double relaxed_tol = 3.0 * fp_->takeoff_reach_tol_;
      if (z_err < relaxed_tol && speed < 2.0 * fp_->takeoff_settle_vel_) {
        ROS_WARN("[takeoff] timeout %.1fs, near altitude (z_err=%.2f m, v=%.2f m/s) -> explore",
                 fp_->takeoff_timeout_, z_err, speed);
        fd_->static_state_ = true;
        leaveTakeoffHover("takeoff: timeout (near altitude)");
      } else {
        ROS_ERROR_THROTTLE(2.0,
            "[takeoff] timeout %.1fs and NOT at altitude (z_err=%.2f m, v=%.2f m/s) -> "
            "holding hover, NOT exploring (check arming / OFFBOARD / thrust)",
            fp_->takeoff_timeout_, z_err, speed);
        // stay in TAKEOFF_HOVER; pubHoverCmd() keeps streaming the climb setpoint.
      }
    }
    break;
  }

  case FINISH: {
    // 조기 종료 구제: 탐사가 끝났다고 하는데 실제로 멀리 진출한 적이 없다면, 좁은
    // FOV 때문에 frontier 가 아예 안 잡힌 상태일 가능성이 높다. 도달 가능한 미방문
    // topo node 로 한 번 날아가 관측을 늘려보고, 그래도 frontier 가 없으면 그때
    // 정상 RTH 로 간다. (스냅샷/호버/MISSION 이벤트보다 먼저 판단해야 이벤트가
    // 두 번 찍히거나 불필요한 호버를 거치지 않는다.)
    // 판정은 하이브리드 2단이다:
    //   1단 (여기): max_displacement_ < thresh — 경로 적분이 아니라 최대 직선
    //      이탈거리. 적분은 근거리 셔플만으로 임계값을 소진해 구제가 죽는다.
    //   2단 (EARLY_FINISH 상태의 selectFarthestReachableNode): 원점거리가
    //      max_displacement_ + gain 을 넘는 미방문 후보가 실제로 있어야 probe.
    //      없으면 FAILED 경로로 정상 FINISH 에 합류한다.
    // - explore_finished_ 조건 필수: 수동 /srv_rth 로 온 FINISH 는 구제 대상이 아니다.
    // - max_retry 래치 필수: 구제 후 또 짧게 끝나면 무한 반복이 된다.
    if (early_finish_enable_ && explore_finished_ &&
        early_finish_count_ < early_finish_max_retry_ &&
        max_displacement_ < early_finish_dist_thresh_) {
      transitState(EARLY_FINISH, "FINISH: max displacement " +
                   to_string(max_displacement_) + "m < " +
                   to_string(early_finish_dist_thresh_) +
                   "m -> probe unvisited reachable topology");
      break;
    }

    // Snapshot the finish pose once, and stop extending the old trajectory so the
    // drone locks where it IS now (traj_server would otherwise keep holding the last
    // trajectory ENDPOINT, which may be a viewpoint ahead of the drone). This snapshot
    // is the fixed setpoint for both the plain hold and the auto-RTH hover.
    if (finish_hover_start_.toSec() < 1e-6) {
      finish_hover_start_ = ros::Time::now();
      finish_hover_pos_ = fd_->odom_pos_.cast<double>();
      finish_hover_yaw_ = fd_->odom_yaw_;
      stopTraj();
      if (traj_server_owns_finish_cmd_) {
        hover_cmd_pub_.shutdown();
        ROS_INFO("[FINISH] /position_cmd ownership transferred to traj_server.");
      }
      // 탐사 종료 요약. 클러스터가 남아있는데 끝났다면 "조기 종료 의심"을 명시
      // (INC1: clusters 17 / reach 0 로 FINISH -> 이게 이번 사고의 1번 원인이었음).
      auto ed = expl_manager_->ed_;
      char d[320];
      snprintf(d, sizeof(d),
               "pos=(%.2f, %.2f, %.2f) elapsed=%.0fs clusters_left=%d(reachable %d) %s",
               finish_hover_pos_.x(), finish_hover_pos_.y(), finish_hover_pos_.z(),
               ros::Time::now().toSec() - total_time_, ed->diag_num_clusters_,
               ed->diag_num_clusters_reachable_,
               expl_manager_->frontier_manager_ptr_->vp_stats_.str().c_str());
      const bool premature = ed->diag_num_clusters_ > 0;
      elog_.log("MISSION",
                premature ? "FINISH (premature? unreached clusters remain)"
                          : "FINISH (map fully explored)",
                d, 0.0, premature ? EventLogger::L_WARN : EventLogger::L_INFO, true);
    }

    const bool do_auto =
        auto_rth_land_ && explore_finished_ && fp_->takeoff_height_ > 0.0;

    // Plain hold (auto RTH+land disabled, no recorded home, or FINISH reached by a
    // manual /srv_rth rather than exploration ending). Stream a FIXED position
    // setpoint from the FSM so the drone locks the finish point and stays OFFBOARD,
    // instead of relying on the bridge's current-pose-follow hold.
    // NOTE: traj_server already holds last_pos_, so any residual sideways creep is
    // EKF/position-estimate drift (mag/EV), which a fixed setpoint cannot remove.
    if (!do_auto) {
      if (auto_rth_land_ && explore_finished_ && fp_->takeoff_height_ <= 0.0)
        ROS_WARN_THROTTLE(5.0, "[FINISH] auto_rth_land on but takeoff disabled "
                               "(no home recorded) -> position hold.");
      if (traj_server_owns_finish_cmd_) {
        ROS_WARN_THROTTLE(2.0, "Finished. traj_server owns /position_cmd hold.");
      } else {
        pubHoldCmd(finish_hover_pos_, finish_hover_yaw_);
        ROS_WARN_THROTTLE(2.0, "Finished. holding position.");
      }
      break;
    }

    // Auto sequence: hover at the finish point (fixed yaw -> no rotation -> no
    // yaw-divergence risk), then return home and land.
    ROS_INFO_THROTTLE(2.0, "\033[32m[FINISH] exploration done -> hover %.1fs, then "
                           "return home & land\033[0m", finish_hover_duration_);
    if (!traj_server_owns_finish_cmd_)
      pubHoldCmd(finish_hover_pos_, finish_hover_yaw_);

    if ((ros::Time::now() - finish_hover_start_).toSec() >= finish_hover_duration_) {
      goal_rth_ << takeoff_anchor_.x(), takeoff_anchor_.y(), takeoff_anchor_.z(),
          takeoff_yaw_;
      has_goal_rth_ = true;
      returning_home_ = true;       // routes the RTH goal-reached to LAND
      explore_finished_ = false;    // consume the latch (don't retrigger this sequence)
      fd_->static_state_ = true;    // first RTH traj anchors to current pose
      global_path_update_timer_.start();
      transitState(PLAN_TRAJ_RTH, "FINISH: hover done -> return home");
    }
    break;
  }

  case EARLY_FINISH: {
    const bool forced = early_finish_forced_attempt_;
    publishEarlyFinishStatus("SELECTING",
                             forced ? "forced request" : "short exploration");

    TopoNode::Ptr probe;
    double graph_distance = 0.0;
    if (!forced)
      early_finish_count_++;

    if (!selectFarthestReachableNode(probe, graph_distance, !forced) ||
        !installEarlyFinishProbe(probe, graph_distance)) {
      // 자동 경로에서는 "후보 없음" 외에 "gain 부족(이미 가본 범위 안뿐)" 도
      // 여기로 온다 — 둘 다 정상 FINISH 합류가 맞다.
      publishEarlyFinishStatus("FAILED", "no qualifying topology node");
      elog_.log("EARLY_FINISH", "no qualifying topology node",
                "max_disp=" + to_string(max_displacement_) + "m", 0.0,
                EventLogger::L_WARN, true);
      early_finish_forced_attempt_ = false;
      if (forced) {
        transitState(PLAN_TRAJ_EXP,
                     "EARLY_FINISH: forced request failed -> resume exploration");
      } else {
        explore_finished_ = true;
        transitState(FINISH, "EARLY_FINISH: no qualifying topology node");
      }
      break;
    }

    explore_finished_ = false;
    fd_->static_state_ = true;
    global_path_update_timer_.start();

    char d[224];
    snprintf(d, sizeof(d),
             "max_disp=%.2fm traveled=%.2fm efp=(%.2f, %.2f, %.2f) "
             "graph_dist=%.2fm yaw=%.2frad attempt=%s",
             max_displacement_, traveled_distance_, probe->center_.x(),
             probe->center_.y(), probe->center_.z(), graph_distance,
             early_finish_yaw_, forced ? "forced" : "automatic");
    elog_.log("EARLY_FINISH",
              "probe installed (stays a TSP candidate until reached)", d, 0.0,
              EventLogger::L_WARN, true);
    ROS_WARN("\033[33m[EARLY_FINISH] probe installed: "
             "efp=(%.2f, %.2f, %.2f), graph_dist=%.2fm\033[0m",
             probe->center_.x(), probe->center_.y(), probe->center_.z(),
             graph_distance);

    early_finish_forced_attempt_ = false;
    transitState(PLAN_TRAJ_EXP, "EARLY_FINISH: probe installed");
    break;
  }

  case PLAN_TRAJ_EXP: {
    if (!fd_->trigger_)
      return;
    if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
      return;

    // (EFP 도달 감시는 FSMCallback 상단에서 상태와 무관하게 상시 수행한다.)
    ros::Time start = ros::Time::now();
    // 要报min-step的case
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double time_to_end = info->duration_ - t_cur;
    if (!early_finish_active_ && expl_manager_->ed_->global_tour_.size() == 2) {
      Eigen::Vector3f goal = expl_manager_->ed_->global_tour_[1];
      if ((goal - fd_->odom_pos_).norm() < 1e-1) {
        explore_finished_ = true;  // genuine exploration end -> enable auto RTH+land
        transitState(FINISH, "fsm");
        return;
      }
    }
    ros::Time tplan = ros::Time::now();
    exec_timer_.stop();
    int res = callExplorationPlanner();
    exec_timer_.start();
    {
      const double t_ms = (ros::Time::now() - tplan).toSec() * 1000.0;
      const char *rs = res == SUCCEED ? "OK"
                       : res == FAIL  ? "FAIL"
                       : res == START_FAIL ? "START_FAIL" : "NO_FRONTIER";
      char d[96];
      snprintf(d, sizeof(d), "plan_time=%.1fms goal_dist=%.1fm", t_ms,
               (expl_manager_->ed_->next_goal_node_->center_ - fd_->odom_pos_).norm());
      std::string sig = std::string("explore ") + rs;
      if (!local_reason_.empty())
        sig += " | why: " + local_reason_;
      elog_.log("LOCAL", sig, d, res == SUCCEED ? 5.0 : 2.0,
                res == SUCCEED ? EventLogger::L_INFO : EventLogger::L_WARN);
    }

    if (res == SUCCEED) {
      // res 는 callExplorationPlanner() = "로컬 궤적 계획"의 결과다. 궤적이
      // 만들어졌다는 사실만으로 frontier 가 존재한다고 볼 수 없다 — EARLY_FINISH
      // 가 넣은 probe 는 뷰포인트가 아니라 평범한 토포 노드이므로, 그쪽으로
      // 궤적이 잘 나와도 frontier 는 여전히 0개일 수 있다. 여기서 무조건 latch
      // 하면 probe 로 향하는 로컬 계획이 성공하는 순간 warmup 보호가 풀리고,
      // 여정이 끝난 뒤 첫 NO_FRONTIER 한 번에 유예 없이 FINISH 로 떨어진다.
      // 기준은 "뷰포인트가 존재하는가"가 아니라 "실제로 경로가 나온 뷰포인트가
      // 있었는가"여야 한다 (viewpoints=1(path_reachable 0) 인 경우가 실제로 있다).
      if (expl_manager_->ed_->diag_num_reachable_vp_ > 0)
        frontiers_ever_seen_ = true;  // reachable viewpoint seen -> warmup done
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      if (fd_->use_bubble_a_star_) {
        transitState(EXEC_TRAJ,
                     "ParallelBubbleAstar plan success: new traj pub");
      } else {
        transitState(EXEC_TRAJ, "plan success: new traj pub");
      }
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == NO_FRONTIER) {
      // EFP 가 살아있으면 planGlobalPath 가 viewpoints 를 비우지 않으므로 여기까지
      // NO_FRONTIER 로 오지 않는다 (= RTH 봉쇄가 자동으로 성립한다). 별도 분기 없음.
      // if (planner_manager_->topo_graph_->global_view_points_.empty())
      if (explorationReallyFinished()) {
        explore_finished_ = true;  // genuine exploration end -> enable auto RTH+land
        transitState(FINISH, "PLAN_TRAJ_EXP: no frontier");
        fd_->static_state_ = true;
      } else {
        // Map/frontiers not ready yet (just triggered) -> keep trying, don't finish.
        transitState(PLAN_TRAJ_EXP, "PLAN_TRAJ_EXP: no frontier yet (warming up)", true);
      }
    } else if (res == FAIL) {
      // Still in PLAN_TRAJ_EXP state, keep replanning
      stopTraj();
      transitState(PLAN_TRAJ_EXP, "PLAN_TRAJ_EXP: plan failed", true);

    } else if (res == START_FAIL) {
      transitState(CAUTION, "PLAN_TRAJ_EXP: start failed", true);
    } else {
      cout << "330?" << endl;
    }
    break;
  }

  case PLAN_TRAJ_RTH: {
    if (!has_goal_rth_)
      return;
    if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
      return;

    // Check if goal reached. Auto return-home uses an xy-only tolerance (we want to
    // be above the takeoff point, then let AUTO.LAND handle the descent); a manual
    // /srv_rth uses the original 3D tolerance.
    Eigen::Vector3d cur = fd_->odom_pos_.cast<double>();
    Eigen::Vector3d gp = goal_rth_.head<3>();
    double dist, tol;
    if (returning_home_) {
      dist = (cur.head<2>() - gp.head<2>()).norm();  // xy distance to home
      tol = rth_land_xy_tol_;
    } else {
      dist = (cur - gp).norm();                      // 3D distance to service goal
      tol = goal_tolerance_;
    }
    // 진행상황: 0.5m 버킷이 바뀔 때만 이벤트 발행 (예전엔 사이클마다 INFO 스팸)
    {
      char sig[64], d[144];
      snprintf(sig, sizeof(sig), "dist_to_goal=%.1fm", std::floor(dist * 2.0) / 2.0);
      snprintf(d, sizeof(d), "%s dist=%.2fm tolerance=%.2fm goal=(%.2f, %.2f, %.2f)",
               returning_home_ ? "auto-home(xy)" : "srv-goal(3D)", dist, tol,
               gp.x(), gp.y(), gp.z());
      elog_.log("RTH", sig, d, 0.0);
    }

    if (dist < tol) {
      has_goal_rth_ = false;
      global_path_update_timer_.stop();  // Stop replanning timer

      // Publish RTH distance for metrics logging
      std_msgs::Float32 dist_msg;
      dist_msg.data = dist;
      rth_metrics_pub_.publish(dist_msg);

      if (returning_home_) {
        transitState(LAND, "RTH: home reached -> AUTO.LAND");
        ROS_INFO("\033[32m[RTH] Home reached (xy %.3f m) -> landing\033[0m", dist);
      } else {
        transitState(FINISH, "PLAN_TRAJ_RTH: goal reached");
        ROS_INFO("\033[32m[RTH] Goal reached! \033[0m");
      }
      return;
    }

    ros::Time tplan = ros::Time::now();
    exec_timer_.stop();
    int res = callGoalPlanner();
    exec_timer_.start();
    {
      const double t_ms = (ros::Time::now() - tplan).toSec() * 1000.0;
      const char *rs = res == SUCCEED ? "OK"
                       : res == FAIL  ? "FAIL"
                       : res == START_FAIL ? "START_FAIL" : "NO_FRONTIER";
      char d[64];
      snprintf(d, sizeof(d), "plan_time=%.1fms", t_ms);
      std::string sig = std::string("RTH ") + rs;
      if (!local_reason_.empty())
        sig += " | why: " + local_reason_;
      elog_.log("LOCAL", sig, d, res == SUCCEED ? 5.0 : 2.0,
                res == SUCCEED ? EventLogger::L_INFO : EventLogger::L_WARN);
    }

    if (res == SUCCEED) {
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      transitState(EXEC_TRAJ, "PLAN_TRAJ_RTH: plan success");
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == FAIL) {
      // Still in PLAN_TRAJ_RTH state, keep replanning
      stopTraj();
      transitState(PLAN_TRAJ_RTH, "PLAN_TRAJ_RTH: plan failed", true);

    } else if (res == START_FAIL) {
      transitState(CAUTION, "PLAN_TRAJ_RTH: start failed", true);
    }
    break;
  }

  case EXEC_TRAJ: {
    // collision check
    double collision_time;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      // Return to appropriate planning state
      EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
      transitState(
          next_state,
          "safetyCallback: not safe, time:" + to_string(collision_time), true);
      if (collision_time < fp_->replan_time_ + 0.2)
        stopTraj();
    } else if (!planner_manager_->checkTrajVelocity()) {
      EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
      transitState(next_state, "velocity too fast", true);
    } else {
      // Emergency control-error replan: continuous replan anchors the next traj to a
      // predicted point on the OLD trajectory (it never looks at the actual pose). If
      // the drone has drifted too far from the traj it is tracking (wind, controller
      // saturation, FAST-LIO pose jump), that assumption is broken -> force a replan
      // anchored to the current pose (static_state_=true). <= 0 disables the check.
      // Skip while reactive avoidance is active: the drone deliberately leaves the
      // planned path then, and the avoidance hook already forces a from-current-pose
      // replan -- firing here would only spam redundant replans against a path PX4 isn't
      // even tracking. (`avoiding` is computed at the top of FSMCallback.)
      LocalTrajData *info = &planner_manager_->local_data_;
      if (!avoiding && info->traj_id_ > 1 && fp_->emergency_replan_control_error > 0.0) {
        double t_cur = (ros::Time::now() - info->start_time_).toSec();
        if (t_cur >= 0.0 && t_cur <= info->duration_) {
          Eigen::Vector3d planned = info->minco_traj_.getPos(t_cur);
          double ctrl_err = (fd_->odom_pos_.cast<double>() - planned).norm();
          if (ctrl_err > fp_->emergency_replan_control_error) {
            ROS_WARN("\033[31m[EMERGENCY] control error %.2f m > %.2f m -> replan from "
                     "current pose\033[0m",
                     ctrl_err, fp_->emergency_replan_control_error);
            fd_->static_state_ = true;
            EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
            transitState(next_state, "emergency: control error", true);
            stopTraj();
          }
        }
      }
    }

    break;
  }

  case CAUTION: {
    stopTraj();
    exec_timer_.stop();
    bool success = planner_manager_->flyToSafeRegion(fd_->static_state_);
    if (success) {
      traj_utils::PolyTraj poly_traj_msg;
      auto info = &planner_manager_->local_data_;
      planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
      fd_->newest_traj_ = poly_traj_msg;

      // CAUTION creates only a position escape trajectory. Publish a matching
      // constant-yaw trajectory so traj_server can activate the new generation
      // atomically instead of continuing the previous yaw trajectory.
      traj_utils::PolyTraj hold_yaw_msg;
      hold_yaw_msg.drone_id = poly_traj_msg.drone_id;
      hold_yaw_msg.traj_id = poly_traj_msg.traj_id;
      hold_yaw_msg.start_time = poly_traj_msg.start_time;
      hold_yaw_msg.order = 5;
      hold_yaw_msg.duration.resize(1);
      hold_yaw_msg.duration[0] =
          std::max(0.1, static_cast<double>(info->duration_));
      hold_yaw_msg.coef_x.assign(6, 0.0);
      hold_yaw_msg.coef_y.assign(6, 0.0);
      hold_yaw_msg.coef_z.assign(6, 0.0);
      hold_yaw_msg.coef_x[5] = fd_->odom_yaw_;
      fd_->newest_yaw_traj_ = hold_yaw_msg;

      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      ros::Duration(0.2).sleep();
    }
    exec_timer_.start();
    double dis2occ =
        planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_);
    if (dis2occ > planner_manager_->gcopter_config_->dilateRadiusSoft) {
      EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
      transitState(next_state, "safe now");
    }
    break;
  }
  case LAND: {
    stopTraj();
    global_path_update_timer_.stop();
    // NOTE: exec_timer_ 는 멈추지 않는다 — 이 콜백이 계속 돌아야 AUTO.LAND 재요청과
    // 착지(disarm) 감지가 동작한다. (예전 exec_timer_.stop() 은 첫 틱에서 FSM 을
    // 정지시켜 "2Hz 재요청" 주석과 모순되는 잠재 버그였음.)
    //
    // 착지 완료: PX4 가 지면 감지 후 자동 disarm (조종자 수동 disarm 포함) -> LANDED.
    if (px4_seen_ && !px4_state_.armed) {
      transitState(LANDED, "LAND: touchdown & disarmed");
      break;
    }
    // Switch PX4 to AUTO.LAND: PX4 throttles down, descends, ground-detects and
    // auto-disarms. Non-blocking — request at ~2 Hz until /mavros/state confirms the
    // mode (the old while(1) froze the FSM thread and published to /px4ctrl/takeoff_land
    // which the real px4_ctrl_bridge does not subscribe to). The bridge keeps streaming
    // OFFBOARD setpoints meanwhile; PX4 ignores them while in AUTO.LAND, so no conflict.
    if (px4_state_.mode != "AUTO.LAND") {
      static ros::Time last_land_req(0);
      ros::Time now = ros::Time::now();
      if ((now - last_land_req).toSec() > 0.5) {
        last_land_req = now;
        mavros_msgs::SetMode land_mode;
        land_mode.request.custom_mode = "AUTO.LAND";
        if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent)
          ROS_WARN_THROTTLE(1.0, "\033[33m[LAND] AUTO.LAND requested\033[0m");
        else
          ROS_WARN_THROTTLE(1.0, "[LAND] AUTO.LAND request failed, retrying...");
      }
    } else {
      ROS_INFO_THROTTLE(2.0, "\033[32m[LAND] PX4 in AUTO.LAND -> descending & auto-disarm\033[0m");
    }
    break;
  }

  case YAW_ROTATE_INIT: {
    // 이륙 호버 지점을 유지한 채 제자리에서 360도 돌아 주변을 한 바퀴 관측한다.
    // 위치는 takeoff_anchor_ 로 고정하고 yaw 만 돌린다.
    if (!fd_->have_odom_)
      return;

    // 완료 판정은 "실제로 돈 각도"(odom)로만 한다. 명령 yaw 를 적분해 판정하면
    // 기체가 못 돌아도 다 돌았다고 보게 된다. 랩어라운드(+-pi)를 흡수하기 위해
    // 매 틱 증분을 [-pi, pi] 로 접어서 누적한다.
    if (yaw_rotate_valid_) {
      double d = (double)fd_->odom_yaw_ - (double)yaw_rotate_last_yaw_;
      while (d > M_PI)  d -= 2.0 * M_PI;
      while (d < -M_PI) d += 2.0 * M_PI;
      // 한 틱에 반 바퀴 넘게 튀면 odom 점프로 보고 버린다.
      if (std::fabs(d) < M_PI)
        yaw_rotate_accum_ += std::fabs(d);
    }
    yaw_rotate_last_yaw_ = fd_->odom_yaw_;
    yaw_rotate_valid_ = true;

    if (yaw_rotate_accum_ >= 2.0 * M_PI) {
      char d[96];
      snprintf(d, sizeof(d), "rotated %.0f deg at (%.2f, %.2f, %.2f)",
               yaw_rotate_accum_ * 180.0 / M_PI, takeoff_anchor_.x(),
               takeoff_anchor_.y(), takeoff_anchor_.z());
      elog_.log("YAW_ROTATE_INIT", "initial 360 scan complete -> explore", d, 0.0,
                EventLogger::L_INFO, true);
      startExplorationFromHover("yaw rotate: 360 scan complete");
      break;
    }

    // 명령 yaw 를 회전율만큼 적분해 보내고, yaw_dot 에 회전율을 실어준다.
    // px4_ctrl_bridge 는 use_yawrate=true 이므로 yaw_dot 을 실제로 사용한다.
    yaw_rotate_cmd_yaw_ += yaw_rotate_init_rate_ * 0.01;  // FSM 주기 100Hz
    while (yaw_rotate_cmd_yaw_ > M_PI)  yaw_rotate_cmd_yaw_ -= 2.0 * M_PI;
    while (yaw_rotate_cmd_yaw_ < -M_PI) yaw_rotate_cmd_yaw_ += 2.0 * M_PI;
    pubHoldCmd(takeoff_anchor_, yaw_rotate_cmd_yaw_, yaw_rotate_init_rate_);

    {
      std_msgs::String m;
      char b[128];
      snprintf(b, sizeof(b), "ROTATING %.0f/360 deg (rate %.2f rad/s)",
               yaw_rotate_accum_ * 180.0 / M_PI, yaw_rotate_init_rate_);
      m.data = b;
      yaw_rotate_state_pub_.publish(m);
      ROS_INFO_THROTTLE(2.0, "\033[36m[yaw-rotate] %s\033[0m", b);
    }
    break;
  }

  case LANDED: {
    // 미션 완전 종료 (착지 + disarm). record_on_goal 이 /planning/state 의 이 상태를
    // 보고 녹화를 마감한다. 여기서는 상태 발행(pubState)만 유지하며 대기.
    elog_.log("MISSION", "LANDED (mission complete, disarmed)", "", 0.0,
              EventLogger::L_INFO);
    break;
  }
  }
}

void FastExplorationFSM::pubHoldCmd(const Eigen::Vector3d &p, double yaw,
                                    double yaw_dot) {
  // Hold (p, yaw) as a position setpoint. px4_ctrl_bridge forwards this on
  // /position_cmd (it ignores the velocity field), so the drone holds pose. Yaw is
  // fixed (yaw_dot=0) -> no rotation. Mirrors traj_server's /position_cmd convention.
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "odom";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = 0;
  cmd.position.x = p.x();
  cmd.position.y = p.y();
  cmd.position.z = p.z();
  cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0;
  cmd.acceleration.x = cmd.acceleration.y = cmd.acceleration.z = 0.0;
  cmd.jerk.x = cmd.jerk.y = cmd.jerk.z = 0.0;
  cmd.yaw = yaw;
  cmd.yaw_dot = yaw_dot;   // 0 이면 회전 없음(기존 동작), 비0 이면 yawrate 회전
  hover_cmd_pub_.publish(cmd);
}

// TAKEOFF_HOVER 를 벗어나는 유일한 경로. 초기 회전이 켜져 있으면 YAW_ROTATE_INIT
// 를 한 번 거친다. 누적 이동거리 리셋은 "탐사가 실제로 시작되는" 시점에서 해야
// 하므로 여기서 하지 않고 startExplorationFromHover() 로 미룬다 — 회전 중의 odom
// 지터가 거리로 잡히면 EARLY_FINISH 판정이 왜곡된다.
void FastExplorationFSM::leaveTakeoffHover(const std::string &why) {
  if (yaw_rotate_init_enable_) {
    yaw_rotate_accum_ = 0.0;
    yaw_rotate_valid_ = false;
    yaw_rotate_cmd_yaw_ = fd_->odom_yaw_;
    char d[128];
    snprintf(d, sizeof(d), "hold (%.2f, %.2f, %.2f), rate %.2f rad/s",
             takeoff_anchor_.x(), takeoff_anchor_.y(), takeoff_anchor_.z(),
             yaw_rotate_init_rate_);
    elog_.log("YAW_ROTATE_INIT", "initial 360 scan started", d, 0.0,
              EventLogger::L_INFO, true);
    transitState(YAW_ROTATE_INIT, why + " -> initial 360 scan");
    return;
  }
  startExplorationFromHover(why);
}

// 이륙 상승분/회전 중 지터가 이동량 지표에 섞이면 EARLY_FINISH 판정이 왜곡되므로
// 탐사가 실제로 시작되는 이 지점에서 0 부터 다시 잰다. expl_origin_ 은
// max_displacement_ 와 EFP 원점거리 랭킹이 공유하는 기준점이라 여기서 앵커해야
// 트리거와 probe 선정이 같은 "원점" 을 본다.
void FastExplorationFSM::startExplorationFromHover(const std::string &why) {
  traveled_distance_ = 0.0;
  traveled_valid_ = false;
  max_displacement_ = 0.0;
  expl_origin_ = fd_->odom_pos_;
  yaw_rotate_accum_ = 0.0;
  yaw_rotate_valid_ = false;
  transitState(PLAN_TRAJ_EXP, why + " -> explore");
}

void FastExplorationFSM::pubHoverCmd() {
  // Climb to (x0, y0, target_z) with the heading captured at trigger time.
  pubHoldCmd(takeoff_anchor_, takeoff_yaw_);
}

void FastExplorationFSM::mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg) {
  // PX4 mode/armed 변화는 사고 분석의 1급 정보 (예: 미션 중 OFFBOARD->POSCTL
  // = 조종자 개입). 변화 시에만 이벤트.
  const bool mode_changed = px4_seen_ && (msg->mode != px4_state_.mode);
  const bool armed_changed = px4_seen_ && (msg->armed != px4_state_.armed);
  const std::string prev_mode = px4_state_.mode;
  px4_state_ = *msg;
  if (!px4_seen_) {
    px4_seen_ = true;
    elog_.log("PX4", "mode=" + msg->mode + " armed=" + (msg->armed ? "1" : "0"));
    return;
  }
  if (mode_changed || armed_changed) {
    // 미션 중 OFFBOARD 이탈/시동해제는 WARN 으로 격상
    const bool left_offboard =
        mode_changed && prev_mode == "OFFBOARD" && msg->mode != "OFFBOARD";
    const bool disarmed = armed_changed && !msg->armed;
    int lvl = (fd_ && fd_->trigger_ && (left_offboard || disarmed))
                  ? EventLogger::L_WARN
                  : EventLogger::L_INFO;
    std::string sig = "mode=" + msg->mode + " armed=" + (msg->armed ? "1" : "0");
    if (left_offboard)
      sig += " (LEFT OFFBOARD: pilot takeover or failsafe)";
    elog_.log("PX4", sig, "prev=" + prev_mode, 0.0, lvl);
  }
}

void FastExplorationFSM::init(ros::NodeHandle &nh,
                              FastExplorationManager::Ptr &explorer) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan", fp_->replan_thresh_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("bubble_astar/resolution_astar", fp_->bubble_a_star_resolution, 0.1);
  nh.param("fsm/debug_planner", debug_planner, false);
  // Default 1.5 matches the value previously hardcoded in algorithm.xml, so configs
  // that don't set this key keep their old (now-active) behaviour. real.yaml overrides it.
  nh.param("fsm/emergency_replan_control_error",
           fp_->emergency_replan_control_error, 1.5);
  // takeoff & hover-before-explore (see config yaml). Default DISABLED (<= 0): only
  // configs that explicitly set fsm/takeoff_height (e.g. real.yaml = 1.0) opt in, so the
  // sim configs keep the original "explore immediately on trigger" behaviour.
  nh.param("fsm/takeoff_height", fp_->takeoff_height_, -1.0);
  nh.param("fsm/takeoff_reach_tol", fp_->takeoff_reach_tol_, 0.15);
  nh.param("fsm/takeoff_settle_vel", fp_->takeoff_settle_vel_, 0.15);
  nh.param("fsm/takeoff_settle_time", fp_->takeoff_settle_time_, 1.0);
  nh.param("fsm/takeoff_timeout", fp_->takeoff_timeout_, 20.0);
  nh.param("fsm/replan_time_after_traj_start",
           fp_->replan_time_after_traj_start_, 0.5);
  nh.param("fsm/replan_time_before_traj_end", fp_->replan_time_before_traj_end_,
           0.5);
  nh.param("fsm/goal_tolerance", goal_tolerance_, 0.2);
  nh.param("fsm/avoid_flag_timeout", avoid_flag_timeout_, 0.5);
  nh.param("fsm/explore_warmup_timeout", explore_warmup_timeout_, 5.0);
  nh.param("fsm/auto_rth_land", auto_rth_land_, true);
  nh.param("fsm/traj_server_owns_finish_cmd", traj_server_owns_finish_cmd_, false);
  nh.param("fsm/finish_hover_duration", finish_hover_duration_, 3.0);
  nh.param("fsm/rth_land_xy_tol", rth_land_xy_tol_, 0.3);
  // EARLY_FINISH (조기 종료 구제). 기본값은 기존 동작과 같도록 잡되 enable 은 true —
  // 끄려면 fsm/early_finish_enable: false.
  nh.param("fsm/early_finish_enable", early_finish_enable_, true);
  // [YAW_ROTATE_INIT] 이륙 후 제자리 360도 초기 관측 회전
  nh.param("fsm/yaw_rotate_init_enable", yaw_rotate_init_enable_, true);
  nh.param("fsm/yaw_rotate_init_rate", yaw_rotate_init_rate_, 0.5);
  nh.param("fsm/early_finish_dist_thresh", early_finish_dist_thresh_, 3.0);
  nh.param("fsm/early_finish_probe_min_gain", early_finish_probe_min_gain_, 1.0);
  nh.param("fsm/early_finish_visited_radius", early_finish_visited_radius_, 1.5);
  nh.param("fsm/early_finish_max_retry", early_finish_max_retry_, 1);
  // [도달 허용치 통일] EFP 도달 판정은 frontier viewpoint 도달 판정과 같은
  // 파라미터를 읽는다. FSM 이 ViewpointManager/ 네임스페이스를 읽는 것은 의도된
  // 것이다 — 새 설계에서 EFP 는 실제로 TSP 의 viewpoint 후보 하나이므로
  // "viewpoint 도달 허용치" 를 공유하는 게 의미상 맞다. 옛
  // fsm/early_finish_reach_tol 은 폐지됐다.
  // EFP 는 **위치 조건만** 본다 (vp_reached_yaw_tol_deg 는 쓰지 않는다). FSM 이
  // EFP 접근 종단 yaw 를 atan2(EFP-odom)=접근 방향으로 잡는데 EFP 노드가 들고
  // 있는 yaw_ 는 그와 무관하므로, yaw 조건을 걸면 구조적으로 만족될 수 없어
  // EFP 은퇴 불가 -> RTH 영구 봉쇄(데드락)가 된다.
  nh.param("ViewpointManager/vp_reached_pos_tol", early_finish_reach_tol_, 0.6);
  nh.param("fsm/local_planning_max_hz", local_planning_max_hz_, 100.0);
  local_planning_min_period_ = 1.0 / local_planning_max_hz_;
  // 이벤트 로깅 관련: verbose_console=true 면 기존 타이밍 cout 유지(개발용),
  // battery_warn_voltage 미만이면 BATT 이벤트가 WARN 으로 격상.
  nh.param("fsm/verbose_console", verbose_console_, false);
  nh.param("fsm/battery_warn_voltage", battery_warn_voltage_, 21.0);
  // reactive local avoidance 마스터 스위치 (real.yaml). false 면 avoid 플래그 무시.
  nh.param("local_avoidance/enable", avoidance_enabled_, true);
  elog_.init(nh);
  elog_.setState("INIT");
  ROS_INFO("Local planning max Hz: %.1f (min period: %.4f s)", local_planning_max_hz_, local_planning_min_period_);
  /* Initialize main modules */
  // expl_manager_.reset(new FastExplorationManager);
  // expl_manager_->initialize(nh);
  expl_manager_ = explorer;
  planner_manager_ = expl_manager_->planner_manager_;

  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  // 순서는 EXPL_STATE enum 과 1:1 이어야 한다 (state_str_[int(state_)] 로 인덱싱).
  fd_->state_str_ = {"INIT",      "WAIT_TRIGGER", "PLAN_TRAJ_EXP", "PLAN_TRAJ_RTH",
                     "CAUTION",   "EXEC_TRAJ",    "FINISH",        "LAND",
                     "TAKEOFF_HOVER", "LANDED",   "EARLY_FINISH",
                     "YAW_ROTATE_INIT"};
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->use_bubble_a_star_ = false;
  has_goal_rth_ = false;
  battary_sub_ =
      nh.subscribe("/mavros/battery", 10, &FastExplorationFSM::battaryCallback,
                   this, ros::TransportHints().tcpNoDelay());

  /* Ros sub, pub and timer */
  // if (debug_planner) {
  //   exec_timer_ = nh.createTimer(ros::Duration(0.01),
  //   &FastExplorationFSM::PlannerDebugFSMCallback, this);
  // } else {
  exec_timer_ = nh.createTimer(ros::Duration(0.01),
                               &FastExplorationFSM::FSMCallback, this);
  // }
  global_path_update_timer_ = nh.createTimer(
      ros::Duration(0.2), &FastExplorationFSM::globalPathUpdateCallback, this);
  trigger_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1,
                              &FastExplorationFSM::triggerCallback, this);
  avoid_flag_sub_ = nh.subscribe("/FSM_flag_avoidance", 10,
                                 &FastExplorationFSM::avoidFlagCallback, this,
                                 ros::TransportHints().tcpNoDelay());
  // 좌표 지정 이동 (구 /srv_rth — 이름 변경. run_batch_experiment.sh 도 갱신됨)
  srv_goal_ = nh.advertiseService("/srv_goto", &FastExplorationFSM::goalServiceCallback, this);
  // rviz 없는 환경용 시작 트리거: rosservice call /srv_start
  srv_start_ = nh.advertiseService("/srv_start", &FastExplorationFSM::startServiceCallback, this);
  // 원터치 홈복귀+착륙: rosservice call /srv_rth (인자 없음)
  srv_rth_home_ = nh.advertiseService("/srv_rth", &FastExplorationFSM::rthServiceCallback, this);
  // Force the EARLY_FINISH recovery path for field/debug validation.
  srv_early_finish_ = nh.advertiseService(
      "/srv_early_finish", &FastExplorationFSM::earlyFinishServiceCallback, this);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);

  // AUTO.LAND at the end of the auto return-home sequence (and /mavros/state to
  // confirm the mode actually engaged). Absolute names = MAVROS defaults.
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_state_sub_ = nh.subscribe("/mavros/state", 10,
                                   &FastExplorationFSM::mavrosStateCallback, this);

  heartbeat_pub_ = nh.advertise<std_msgs::Empty>("/planning/heartbeat", 10);
  land_pub_ =
      nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);

  poly_traj_pub_ =
      nh.advertise<traj_utils::PolyTraj>("/planning/trajectory", 10);
  poly_yaw_traj_pub_ =
      nh.advertise<traj_utils::PolyTraj>("/planning/yaw_trajectory", 10);
  time_cost_pub_ = nh.advertise<std_msgs::Float32>("/time_cost", 10);
  static_pub_ = nh.advertise<std_msgs::Bool>("/planning/static", 10);
  state_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/state", 10);
  yaw_rotate_state_pub_ =
      nh.advertise<std_msgs::String>("/planning/yaw_rotate_init", 10, true);
  early_finish_state_pub_ =
      nh.advertise<std_msgs::String>("/planning/early_finish_state", 10, true);
  publishEarlyFinishStatus("IDLE");
  rth_metrics_pub_ = nh.advertise<std_msgs::Float32>("/planning/rth_distance", 10);
  // [feature: astar-profile]
  astar_profile_pub_ =
      nh.advertise<std_msgs::String>("/planning/timing/astar_profile", 10);
  nh.param("fsm/astar_profile_period", astar_profile_period_, 5.0);
  {
    double t = 0.0;
    nh.param("parallel_astar/update_connection_timeout", t, 0.0);
    astar_conn_timeout_ms_ = t * 1000.0;
    nh.param("parallel_astar/insert_node_timeout", t, 0.0);
    astar_insert_timeout_ms_ = t * 1000.0;
  }
  // exploration debug HUD (rviz text marker) + machine-readable string (bag/log)
  diag_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/expl_diag", 10);
  diag_str_pub_ = nh.advertise<std_msgs::String>("/planning/expl_diag_str", 10);
  // key=value 진단 (record_on_goal.py 가 파싱해 epic.log 세로 블록으로 기록)
  diag_kv_pub_ = nh.advertise<std_msgs::String>("/planning/expl_diag_kv", 10);
  // Hover setpoint stream during TAKEOFF_HOVER. Absolute topic name = traj_server's
  // /position_cmd; the two never publish at the same time (traj_server is silent until
  // a trajectory exists, and we only publish here before exploration starts).
  hover_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  // Global planning timing publishers
  update_topo_skeleton_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/update_topo_skeleton_cost", 10);
  update_odom_vertex_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/update_odom_vertex_cost", 10);
  vp_cluster_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/vp_cluster_cost", 10);
  remove_unreachable_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/remove_unreachable_cost", 10);
  select_vp_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/select_vp_cost", 10);
  insert_viewpoint_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/insert_viewpoint_cost", 10);
  calculate_tsp_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/calculate_tsp_cost", 10);
  lkh_solver_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/lkh_solver_cost", 10);
  call_planner_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/call_planner_cost", 10);
  ikd_tree_insert_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/ikd_tree_insert_cost", 10);
  update_frontier_clusters_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/update_frontier_clusters_cost", 10);
  fast_searcher_search_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/fast_searcher_search_cost", 10);
  bubble_astar_search_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/bubble_astar_search_cost", 10);

  string odom_topic, cloud_topic;
  // 토픽명은 config yaml(odometry_topic/cloud_topic)이 유일한 소스. 폴백 금지
  // — 없으면 즉시 종료 (잘못된 토픽으로 조용히 도는 것 방지).
  if (!nh.getParam("odometry_topic", odom_topic) || odom_topic.empty() ||
      !nh.getParam("cloud_topic", cloud_topic) || cloud_topic.empty()) {
    ROS_FATAL("[FSM] odometry_topic / cloud_topic not set in config yaml. "
              "REFUSING TO START - no fallback.");
    ros::shutdown();
    exit(1);
  }

  // Keep FSM frontier updates on the same cropped stream used by the LIO
  // interface whenever MID360-to-ML-X emulation is enabled.
  bool cloud_crop_enable = false;
  nh.param("cloud_crop/enable", cloud_crop_enable, false);
  if (cloud_crop_enable) {
    string cropped_topic;
    if (!nh.getParam("cloud_crop/output_topic", cropped_topic) ||
        cropped_topic.empty()) {
      ROS_FATAL("[FSM] cloud_crop/enable=true but "
                "cloud_crop/output_topic is not configured.");
      ros::shutdown();
      exit(1);
    }
    ROS_WARN("[FSM] cloud crop enabled: subscribing %s (raw: %s)",
             cropped_topic.c_str(), cloud_topic.c_str());
    cloud_topic = cropped_topic;
  }
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      nh, cloud_topic, 1));
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic, 5));
  sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
      SyncPolicyCloudOdom(10), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(
      boost::bind(&FastExplorationFSM::CloudOdomCallback, this, _1, _2));

  // ---- PARAM 이벤트 라인 캐시 구성 ----
  // 계획 성공/실패를 좌우하는 핵심 파라미터를 이벤트 스트림에 남긴다.
  // 기동 시 1회 + 트리거 시 재발행(레코딩이 goal 시점부터라 bag/log에 남도록).
  {
    auto getd = [&](const char *k, double def) {
      double v;
      nh.param(k, v, def);
      return v;
    };
    auto geti = [&](const char *k, int def) {
      int v;
      nh.param(k, v, def);
      return v;
    };
    std::vector<double> bx_dn, bx_up;
    nh.param("box_0/down", bx_dn, std::vector<double>());
    nh.param("box_0/up", bx_up, std::vector<double>());
    std::string odom_t, cloud_t;
    nh.param("odometry_topic", odom_t, std::string("?"));
    nh.param("cloud_topic", cloud_t, std::string("?"));

    char l[288];
    param_lines_.clear();
    if (bx_dn.size() == 3 && bx_up.size() == 3)
      snprintf(l, sizeof(l), "box | down=[%g, %g, %g] up=[%g, %g, %g]", bx_dn[0],
               bx_dn[1], bx_dn[2], bx_up[0], bx_up[1], bx_up[2]);
    else
      snprintf(l, sizeof(l), "box | (box_0 params missing)");
    param_lines_.push_back(l);

    snprintf(l, sizeof(l),
             "corridor | DilateRadiusSoft=%.2f DilateRadiusHard=%.2f "
             "MaxCorridorSize=%.1f MaxVelMag=%.1f max_traj_len=%.1f "
             "topology_safe_distance=%.2f",
             getd("DilateRadiusSoft", -1), getd("DilateRadiusHard", -1),
             getd("MaxCorridorSize", -1), getd("MaxVelMag", -1),
             getd("max_traj_len", -1),
             getd("bubble_astar/safe_distance", -1));
    param_lines_.push_back(l);

    snprintf(l, sizeof(l),
             "frontier | cell_size=%.2f cluster_size=[%.1f, %.1f] "
             "cluster_min_points=%d candidate_dphi=%.2f candidate_znum=%d "
             "good_observation_dir_score=%.2f",
             getd("FrontierManager/cell_size", -1),
             getd("FrontierManager/cluster_min_size", -1),
             getd("FrontierManager/cluster_max_size", -1),
             geti("FrontierManager/cluster_minmum_point_num", -1),
             getd("frontier/candidate_dphi", -1), geti("frontier/candidate_znum", -1),
             getd("FrontierManager/good_observation_direction_score", -1));
    param_lines_.push_back(l);

    snprintf(l, sizeof(l),
             "viewpoint | pillar_radius=[%.1f, %.1f]x%d pillar_height=[%.1f, %.1f]x%d "
             "circle_samples=%d min_obstacle_clearance=%.2f local_tsp_size=%d",
             getd("ViewpointManager/sample_pillar_min_radius", -1),
             getd("ViewpointManager/sample_pillar_max_radius", -1),
             geti("ViewpointManager/sample_pillar_radius_layer_num", -1),
             getd("ViewpointManager/sample_pillar_min_height", -1),
             getd("ViewpointManager/sample_pillar_max_height", -1),
             geti("ViewpointManager/sample_pillar_height_layer_num", -1),
             geti("ViewpointManager/sample_pillar_circle_sample_num", -1),
             getd("ViewpointManager/min_obstacle_clearance", -1),
             geti("ViewpointManager/local_tsp_size", -1));
    param_lines_.push_back(l);

    // [feature: topo-timeout] 같은 graphSearch 를 쓰는 세 예산을 한 줄에 모은다.
    // 뷰포인트가 대량으로 "도달 불가"가 된 비행을 사후에 볼 때, 기하 문제인지
    // 예산 문제인지 판단하려면 이 값들이 세션 스냅샷에 남아 있어야 한다.
    snprintf(l, sizeof(l),
             "topo_timeout | vp_reachability=%.4f tsp_cost=%.4f goal=%.4f "
             "astar_insert=%.4f astar_update=%.4f [s]",
             getd("ViewpointManager/reachability_search_timeout", 3e-4),
             getd("global_planning/topo_cost_search_timeout", 1e-2),
             getd("global_planning/goal_search_timeout", 0.1),
             getd("parallel_astar/insert_node_timeout", -1),
             getd("parallel_astar/update_connection_timeout", -1));
    param_lines_.push_back(l);

    snprintf(l, sizeof(l),
             "fsm | takeoff_height=%.2f goal_tolerance=%.2f replan_time=%.2f "
             "emergency_replan_error=%.1f local_plan_max_hz=%.0f "
             "avoid_flag_timeout=%.1f auto_rth_land=%d rth_land_xy_tol=%.2f "
             "avoidance_enabled=%d traj_server_owns_finish_cmd=%d",
             fp_->takeoff_height_, goal_tolerance_, fp_->replan_time_,
             fp_->emergency_replan_control_error, local_planning_max_hz_,
             avoid_flag_timeout_, auto_rth_land_ ? 1 : 0, rth_land_xy_tol_,
             avoidance_enabled_ ? 1 : 0,
             traj_server_owns_finish_cmd_ ? 1 : 0);
    param_lines_.push_back(l);

    snprintf(l, sizeof(l), "topics | odom=%s cloud=%s", odom_t.c_str(),
             cloud_t.c_str());
    param_lines_.push_back(l);
  }
  logParamsEvents(false);
}

// 캐시된 PARAM 라인들을 이벤트로 발행하고, 전체 스냅샷을 latched
// /epic/session_info 로도 발행 (늦게 시작한 rosbag/record_on_goal 도 수신).
void FastExplorationFSM::logParamsEvents(bool force) {
  std::string all;
  for (auto &l : param_lines_) {
    elog_.log("PARAM", l, "", 0.0, EventLogger::L_INFO, force);
    all += l + "\n";
  }
  // 현재 회피 상태도 함께 기록 (기동/트리거 시점 스냅샷; 이후는 에지 이벤트가 담당)
  elog_.log("Avoidance:",
            !avoidance_enabled_
                ? "Disabled (local_avoidance/enable=false)"
                : (avoiding_prev_ ? "Activated" : "Deactivated"),
            "", 0.0, EventLogger::L_INFO, force);
  char head[96];
  snprintf(head, sizeof(head), "EPIC session info | published_ros=%.3f\n",
           ros::Time::now().toSec());
  elog_.publishSessionInfo(std::string(head) + all);
}

void FastExplorationFSM::battaryCallback(
    const sensor_msgs::BatteryStateConstPtr &msg) {
  // 0.5V 버킷이 바뀔 때만 이벤트. 경고 전압 미만이면 WARN (자동 착륙은 안 함).
  if (!std::isfinite(msg->voltage) || msg->voltage <= 0.1)
    return;
  char sig[48], d[96];
  snprintf(sig, sizeof(sig), "%.1fV", std::floor(msg->voltage * 2.0) / 2.0);
  snprintf(d, sizeof(d), "voltage=%.2fV percent=%.0f%%", msg->voltage,
           msg->percentage >= 0 ? msg->percentage * 100.0 : -1.0);
  elog_.log("BATT", sig, d, 0.0,
            msg->voltage < battery_warn_voltage_ ? EventLogger::L_WARN
                                                 : EventLogger::L_INFO);
  // if(msg->voltage < 21.0){
  //   transitState(LAND, "battary low");
  // }
}

void FastExplorationFSM::updateTopoAndGlobalPath() {
  // WAIT_TRIGGER 제외: 트리거(2D Nav Goal) 전에는 토포/글로벌 경로 갱신을 하지 않는다.
  // (launch 직후 빈 토포그래프에서 odom_node 이웃이 없어 CAUTION으로 전이 -> flyToSafeRegion
  //  MINCO 최적화가 계속 실패("optimize failed")하는 것을 막음.) 트리거 후 TAKEOFF_HOVER ->
  //  PLAN_TRAJ_EXP 부터 planning 시작. WAIT_TRIGGER 에서는 viz만 한다.
  // EARLY_FINISH also needs one current topology update before selecting its path.
  if (!(state_ == PLAN_TRAJ_EXP || state_ == PLAN_TRAJ_RTH ||
        state_ == EXEC_TRAJ || state_ == FINISH || state_ == EARLY_FINISH)) {
    global_path_update_timer_.stop();
    expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
  expl_manager_->frontier_manager_ptr_->vizBestViewpoint();
    global_path_update_timer_.start();
    return;
  }
  static int cnt = 0;
  cnt++;

  global_path_update_timer_.stop();
  ros::Time t2 = ros::Time::now();
  planner_manager_->topo_graph_->getRegionsToUpdate();
  // cout << "getRegionsToUpdate time cost:" << (ros::Time::now() - t2).toSec()
  // * 1000 << "ms" << endl;
  planner_manager_->topo_graph_->updateSkeleton();

  ros::Time t3 = ros::Time::now();
  planner_manager_->topo_graph_->updateOdomNode(fd_->odom_pos_, fd_->odom_yaw_);
  planner_manager_->topo_graph_->updateHistoricalOdoms();

  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) {
    double time;
    if (planner_manager_->local_data_.traj_id_ > 1) {
      bool safe = planner_manager_->checkTrajCollision(time);
      if (!safe) {
        transitState(CAUTION, "odom_node no nbrs");
      } else {
        global_path_update_timer_.start();

        return;
      }
    } else {
      transitState(CAUTION, "odom_node no nbrs");
    }
    global_path_update_timer_.start();
    return;
  }
  if (planner_manager_->local_data_.traj_id_ > 1) {

    double curr_time =
        (ros::Time::now() - planner_manager_->local_data_.start_time_).toSec();
    double time;
    bool safe = planner_manager_->checkTrajCollision(time);
    double total_time = planner_manager_->local_data_.duration_;
    double time2end = total_time - curr_time;

    if (safe && curr_time < fp_->replan_time_after_traj_start_ &&
        time2end > fp_->replan_time_before_traj_end_) {
      global_path_update_timer_.start();
      return;
    }
  }

  // Handle RTH mode and exploration mode separately
  if (has_goal_rth_) {
    // RTH mode: call planGoalPath and transition to PLAN_TRAJ_RTH
    ros::Time t_rth = ros::Time::now();
    int res = expl_manager_->planGoalPath(goal_rth_.head<3>(), goal_rth_(3));
    last_plan_ms_ = (ros::Time::now() - t_rth).toSec() * 1000.0;
    logGlobalPlanEvent(res, last_plan_ms_);
    publishExplDiag(); // RTH 중에도 HUD/진단 유지 (예전엔 여기서 끊겨 분석 공백)
    if (res == SUCCEED && state_ != WAIT_TRIGGER) {
      transitState(PLAN_TRAJ_RTH, "planGoalPath: succeed");
    }
    expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
  expl_manager_->frontier_manager_ptr_->vizBestViewpoint();
    global_path_update_timer_.start();
    return;
  }

  // Exploration mode: use TSP-based global planning
  if (verbose_console_) {
    cout << endl << endl;
    cout << "\033[1;33m------------- <" << cnt
         << "> Plan Global Path start---------------" << "\033[0m" << endl;
  }
  planner_manager_->topo_graph_->log << "<" << cnt << ">" << endl;
  ros::Time t4 = ros::Time::now();
  if (verbose_console_)
    ROS_INFO("update topo skeleton cost: %fms, update odom vertex cost:%fms ",
             (t3 - t2).toSec() * 1000, (t4 - t3).toSec() * 1000);
  Eigen::Vector3d vel = fd_->odom_vel_.cast<double>();
  Eigen::Vector3d odom = fd_->odom_pos_.cast<double>();
  // EFP 수명 관리는 여기 한 곳에서만 한다 (도달 판정 -> 연결성 확인/재바인딩 ->
  // planner 로 push). 방금 updateSkeleton/updateOdomNode 를 돌린 직후라 그래프가
  // 최신이고, planGlobalPath 가 그 값을 바로 읽는다.
  updateEarlyFinishProbe();
  int res = expl_manager_->planGlobalPath(odom, vel);
  ros::Time t5 = ros::Time::now();

  planner_manager_->graph_visualizer_->vizBox(planner_manager_->topo_graph_);
  if(expl_manager_->ep_->view_graph_)
    planner_manager_->graph_visualizer_->vizGraph(planner_manager_->topo_graph_);
  std_msgs::Float32 time_cost;
  double time_cost_now = (t5 - t2).toSec() * 1000;
  time_cost.data = time_cost_now;
  time_cost_pub_.publish(time_cost);

  logGlobalPlanEvent(res, time_cost_now);
  if (verbose_console_)
    cout << "total time cost: " << time_cost_now << "ms" << endl;
  if (res == NO_FRONTIER && state_ != WAIT_TRIGGER) {
    // EFP 가 outstanding 이면 viewpoints 가 비지 않아 여기까지 오지 않는다
    // (사건 2: 새 vp 가 생겨도 EFP 는 후보로 남고, EFP 만 남아도 tour 가 선다).
    // Only finish if the map/frontiers were actually ready (warmup elapsed or
    // frontiers seen before); otherwise this is a startup artifact -> wait.
    if (explorationReallyFinished()) {
      explore_finished_ = true;  // genuine exploration end -> enable auto RTH+land
      transitState(FINISH, "planGlobalPath: no frontier");
    }
  } else if (res == SUCCEED && state_ != WAIT_TRIGGER) {
    // "실제로 경로가 나온 frontier 뷰포인트" 가 있었을 때만 warmup 을 푼다.
    // EFP 는 frontier 가 아니므로 diag_num_reachable_vp_ 에서 제외돼 있다
    // (planGlobalPath). EFP 로 향하는 계획이 성공했다는 이유로 warmup 보호가
    // 풀리면, EFP 여정이 끝난 뒤 첫 NO_FRONTIER 한 번에 유예 없이 FINISH 로
    // 떨어진다. EFP 가 없을 때는 SUCCEED == reachable vp>0 이라 동작 불변이다.
    if (expl_manager_->ed_->diag_num_reachable_vp_ > 0)
      frontiers_ever_seen_ = true;  // frontiers confirmed to exist -> warmup done
    transitState(PLAN_TRAJ_EXP, "planGlobalPath: succeed");
  }

  last_plan_ms_ = time_cost_now;
  publishExplDiag();  // 클러스터/뷰포인트 수 + 사유를 rviz HUD + string 으로 발행

  expl_manager_->frontier_manager_ptr_->viz_pocc();
  expl_manager_->frontier_manager_ptr_->visfrtcluster();
  expl_manager_->frontier_manager_ptr_->vizBestViewpoint();
  // [feature: astar-profile] 상한 대비 실제 소요시간 분포를 주기적으로 발행.
  {
    static ros::WallTime t_prof = ros::WallTime::now();
    if ((ros::WallTime::now() - t_prof).toSec() > astar_profile_period_ &&
        ParallelBubbleAstar::profile_.count() > 0) {
      t_prof = ros::WallTime::now();
      const std::string rep = ParallelBubbleAstar::profile_.report(true);
      std_msgs::String m;
      m.data = rep;
      astar_profile_pub_.publish(m);
      ROS_INFO("[astar-profile] %s (caps: conn=%.1fms insert=%.1fms)",
               rep.c_str(), astar_conn_timeout_ms_, astar_insert_timeout_ms_);
    }
  }

  static ros::Time t_p = ros::Time::now();
  if ((ros::Time::now() - t_p).toSec() > 5.0) {
    expl_manager_->frontier_manager_ptr_->printMemoryCost();
    t_p = ros::Time::now();
  }
  global_path_update_timer_.start();
  if (verbose_console_)
    cout << "viz&&print cost:" << (ros::Time::now() - t5).toSec() * 1000 << "ms"
         << endl;
}

// GLOBAL 계획 결과를 이벤트로 발행. sig=결과 분류(+사유), detail=카운트/투어/시간.
// 카운트만 바뀌는 변화는 1s 코얼레싱, 결과 분류가 바뀌면 즉시 발행된다.
void FastExplorationFSM::logGlobalPlanEvent(int res, double t_ms) {
  auto ed = expl_manager_->ed_;
  double tour_len = 0.0;
  for (size_t i = 1; i < ed->global_tour_.size(); ++i)
    tour_len += (ed->global_tour_[i] - ed->global_tour_[i - 1]).norm();
  const Eigen::Vector3f goalp =
      ed->next_goal_node_ ? ed->next_goal_node_->center_ : Eigen::Vector3f::Zero();
  char d[400];
  snprintf(d, sizeof(d),
           "clusters=%d(reachable %d) viewpoints=%d(path_reachable %d) %s "
           "tour=%zu_nodes/%.1fm next_goal=(%.1f, %.1f, %.1f) plan_time=%.0fms",
           ed->diag_num_clusters_, ed->diag_num_clusters_reachable_,
           ed->diag_num_viewpoints_, ed->diag_num_reachable_vp_,
           expl_manager_->frontier_manager_ptr_->vp_stats_.str().c_str(),
           ed->global_tour_.size(), tour_len, goalp.x(), goalp.y(), goalp.z(), t_ms);
  std::string sig = "result=" + ed->diag_result_;
  if (ed->diag_result_ != "OK" && ed->diag_reason_ != "OK" && !ed->diag_reason_.empty())
    sig += " | why: " + ed->diag_reason_;
  const bool bad = (res != SUCCEED);
  elog_.log("GLOBAL", sig, d, 1.0, bad ? EventLogger::L_WARN : EventLogger::L_INFO);
}

void FastExplorationFSM::publishExplDiag() {
  auto ed = expl_manager_->ed_;

  // --- 파생 지표 계산 ---
  const double speed = fd_->odom_vel_.norm();
  const double yaw_deg = fd_->odom_yaw_ * 180.0 / M_PI;

  // 토포 그래프 연결성: odom 노드 이웃 수 (0 이면 계획 자체가 막힘)
  int odom_nbr = -1;
  if (planner_manager_->topo_graph_ && planner_manager_->topo_graph_->odom_node_)
    odom_nbr = (int)planner_manager_->topo_graph_->odom_node_->neighbors_.size();

  // global tour: 노드 수 + 전체 길이 + 다음 홉 거리
  const int tour_nodes = (int)ed->global_tour_.size();
  double tour_len = 0.0;
  for (size_t i = 1; i < ed->global_tour_.size(); ++i)
    tour_len += (ed->global_tour_[i] - ed->global_tour_[i - 1]).norm();

  // 현재 목표 노드(planner 가 세팅) 와 거기까지 직선거리
  Eigen::Vector3f goalp = ed->next_goal_node_ ? ed->next_goal_node_->center_
                                              : Eigen::Vector3f::Zero();
  const double goal_dist = (fd_->odom_pos_ - goalp).norm();

  const bool avoiding = (avoid_flag_ != 0);

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "EPIC  state=" << fd_->state_str_[state_]
     << "   plan " << std::setprecision(1) << last_plan_ms_ << "ms\n"
     << std::setprecision(2)
     << "pos [" << fd_->odom_pos_.x() << ", " << fd_->odom_pos_.y() << ", "
     << fd_->odom_pos_.z() << "]  yaw " << std::setprecision(0) << yaw_deg
     << std::setprecision(2) << "  v " << speed << " m/s\n"
     << "clusters " << ed->diag_num_clusters_ << " (reach "
     << ed->diag_num_clusters_reachable_ << ")   vp " << ed->diag_num_viewpoints_
     << " (reach " << ed->diag_num_reachable_vp_ << ")\n"
     << "topo odom_nbr " << odom_nbr << "   tour " << tour_nodes << " nodes / "
     << std::setprecision(1) << tour_len << " m\n" << std::setprecision(2)
     << "goal [" << goalp.x() << ", " << goalp.y() << ", " << goalp.z()
     << "]  d " << goal_dist << " m\n"
     << "trig " << (fd_->trigger_ ? 1 : 0) << "  static "
     << (fd_->static_state_ ? 1 : 0) << "  avoid " << (avoiding ? 1 : 0)
     << "  rth " << (has_goal_rth_ ? 1 : 0) << "   fail bb "
     << fd_->bb_astar_fail_cnt_ << " fs " << fd_->fast_search_fial_cnt_ << "\n"
     << "global: " << ed->diag_result_ << "\n"
     << "local: " << (local_reason_.empty() ? "OK" : local_reason_);
  const std::string txt = ss.str();

  // 1) 기록용 문자열 (rosbag). rosout 은 이벤트 로거([EV] ...)가 담당하므로
  //    여기서는 스로틀 로그를 찍지 않는다 (중복 스팸 방지).
  std_msgs::String smsg;
  smsg.data = txt;
  diag_str_pub_.publish(smsg);

  // 3) 기계 파싱용 key=value 진단 -> /planning/expl_diag_kv
  //    record_on_goal.py 가 파싱해 epic.log 세로 블록으로 기록한다.
  //    포맷 계약: "key=value" 를 ';' 로 연결한 한 줄. 자유 텍스트(global/local
  //    사유)는 ';' 를 ',' 로 치환해 구분자 충돌을 막는다.
  {
    auto sanitize = [](std::string s) {
      for (auto &c : s)
        if (c == ';' || c == '\n') c = ',';
      return s;
    };
    const auto &ps = expl_manager_->frontier_manager_ptr_->vp_stats_;
    std::ostringstream kv;
    kv << std::fixed << std::setprecision(2)
       << "t=" << ros::Time::now().toSec()
       << ";state=" << fd_->state_str_[state_]
       << ";mode=" << sanitize(px4_seen_ ? px4_state_.mode : std::string("?"))
       << ";armed=" << ((px4_seen_ && px4_state_.armed) ? 1 : 0)
       << ";plan_ms=" << std::setprecision(1) << last_plan_ms_
       << std::setprecision(2)
       << ";clusters=" << ed->diag_num_clusters_
       << ";clusters_reach=" << ed->diag_num_clusters_reachable_
       << ";vp=" << ed->diag_num_viewpoints_
       << ";vp_reach=" << ed->diag_num_reachable_vp_
       << ";pipe_total=" << ps.total
       << ";pipe_dormant=" << ps.dormant
       << ";pipe_unreachable_pre=" << ps.unreachable_pre
       << ";pipe_considered=" << ps.considered
       << ";pipe_no_candidates=" << ps.no_candidates
       << ";pipe_topo_unreachable=" << ps.topo_unreachable
       << ";pipe_no_visibility=" << ps.no_visibility
       << ";pipe_ok=" << ps.ok
       << ";frt_cells=" << expl_manager_->frontier_manager_ptr_->frontierCellCount()
       << ";tsp_nodes=" << tour_nodes
       << ";tour_len=" << std::setprecision(1) << tour_len
       << std::setprecision(2)
       << ";goal=" << goalp.x() << "," << goalp.y() << "," << goalp.z()
       << ";goal_dist=" << goal_dist
       << ";global=" << sanitize(ed->diag_result_)
       << ";local=" << sanitize(local_reason_.empty() ? "OK" : local_reason_);
    std_msgs::String kmsg;
    kmsg.data = kv.str();
    diag_kv_pub_.publish(kmsg);
  }

  // 2) rviz HUD: 드론 위에 떠다니는 텍스트 마커 (frontier 마커와 같은 "odom" 프레임)
  visualization_msgs::Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = ros::Time::now();
  m.ns = "expl_diag";
  m.id = 0;
  m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = fd_->odom_pos_.x();
  m.pose.position.y = fd_->odom_pos_.y();
  m.pose.position.z = fd_->odom_pos_.z() + 1.8;
  m.pose.orientation.w = 1.0;
  m.scale.z = 0.35;  // 글자 높이 [m] (줄이 많아 조금 작게)
  m.color.a = 1.0;
  const bool bad = (ed->diag_result_.find("NO_") != std::string::npos ||
                    ed->diag_result_.find("FAIL") != std::string::npos);  // 실패=빨강
  m.color.r = bad ? 1.0f : 0.2f;
  m.color.g = bad ? 0.2f : 1.0f;
  m.color.b = 0.2f;
  m.text = txt;
  diag_pub_.publish(m);
}

void FastExplorationFSM::globalPathUpdateCallback(const ros::TimerEvent &e) {
  updateTopoAndGlobalPath();
}

// rviz 2D Nav Goal 대체: 터미널에서 `rosservice call /srv_start` 한 줄로 미션 시작.
// 토픽 방식과 달리 성공/실패와 이유를 응답으로 돌려준다.
bool FastExplorationFSM::startServiceCallback(std_srvs::Trigger::Request &req,
                                              std_srvs::Trigger::Response &res) {
  (void)req;
  if (startMission("/srv_start")) {
    res.success = true;
    res.message = "mission started (state -> " + fd_->state_str_[int(state_)] + ")";
  } else {
    res.success = false;
    res.message = "ignored: FSM not in WAIT_TRIGGER (current: " +
                  fd_->state_str_[int(state_)] + ")";
  }
  return true;
}

bool FastExplorationFSM::earlyFinishServiceCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  (void)req;
  if (!fd_->have_odom_) {
    res.success = false;
    res.message = "rejected: odometry is not available";
    return true;
  }
  if (early_finish_active_ || early_finish_force_requested_ ||
      state_ == EARLY_FINISH) {
    res.success = false;
    res.message = "rejected: EARLY_FINISH is already active or queued";
    return true;
  }

  const bool eligible =
      state_ == PLAN_TRAJ_EXP || state_ == EXEC_TRAJ ||
      state_ == CAUTION || state_ == FINISH;
  if (!eligible) {
    res.success = false;
    res.message = "rejected in state " + fd_->state_str_[int(state_)];
    return true;
  }

  early_finish_force_requested_ = true;
  res.success = true;
  res.message = "EARLY_FINISH queued; watch /planning/early_finish_state";
  return true;
}

// 원터치 홈복귀: `rosservice call /srv_rth` (인자 없음).
// 비행 중 어느 상태에서든 즉시 이륙 지점(takeoff anchor)으로 복귀하고,
// 홈 xy 반경(rth_land_xy_tol) 도달 시 자동 AUTO.LAND -> LANDED (자동 RTH와 동일 경로).
bool FastExplorationFSM::rthServiceCallback(std_srvs::Trigger::Request &req,
                                            std_srvs::Trigger::Response &res) {
  (void)req;
  if (state_ == PLAN_TRAJ_RTH && returning_home_) {
    res.success = false;
    res.message = "already returning home";
    return true;
  }
  const bool flying =
      (state_ == TAKEOFF_HOVER || state_ == PLAN_TRAJ_EXP || state_ == EXEC_TRAJ ||
       state_ == CAUTION || state_ == FINISH || state_ == PLAN_TRAJ_RTH);
  if (!flying) {
    res.success = false;
    res.message = "ignored: not flying (current: " + fd_->state_str_[int(state_)] + ")";
    return true;
  }

  // 홈 = 이륙 앵커 (x0, y0, takeoff_height). takeoff 비활성 구성이면 (0,0,현재고도) 폴백.
  Eigen::Vector3d home = takeoff_anchor_;
  double home_yaw = takeoff_yaw_;
  if (fp_->takeoff_height_ <= 0.0) {
    home = Eigen::Vector3d(0.0, 0.0, std::max(0.5, (double)fd_->odom_pos_.z()));
    home_yaw = fd_->odom_yaw_;
  }
  goal_rth_ << home.x(), home.y(), home.z(), home_yaw;
  has_goal_rth_ = true;
  returning_home_ = true;     // 홈 xy 도달 -> LAND(AUTO.LAND) -> LANDED
  explore_finished_ = false;
  // 수동 RTH 는 모든 것을 덮어쓰는 중단 명령이다. EFP 를 들고 가면 마커/자주색
  // leg 만 유령으로 남으므로 여기서 명시적으로 포기한다.
  // (자동 RTH 는 EFP 가 이미 소멸한 뒤에만 도달하므로 이 처리가 필요 없다.)
  if (early_finish_active_)
    clearEarlyFinishPath("ABORTED_BY_RTH");
  // 현재 포즈에 앵커해 재계획 (TAKEOFF_HOVER 처럼 아직 궤적이 없는 상태에서도 안전;
  // 비행 중이면 짧은 감속 후 홈으로 전환 — 중단 명령이므로 예측 가능성 우선)
  fd_->static_state_ = true;
  global_path_update_timer_.start();

  char d[128];
  snprintf(d, sizeof(d), "home=(%.2f, %.2f, %.2f) from pos=(%.2f, %.2f, %.2f)",
           home.x(), home.y(), home.z(), fd_->odom_pos_.x(), fd_->odom_pos_.y(),
           fd_->odom_pos_.z());
  elog_.log("EVENT", "RTH requested (/srv_rth)", d, 0.0, EventLogger::L_WARN, true);
  transitState(PLAN_TRAJ_RTH, "/srv_rth: return home");

  char m[128];
  snprintf(m, sizeof(m), "returning home to (%.2f, %.2f, %.2f); AUTO.LAND on arrival",
           home.x(), home.y(), home.z());
  res.success = true;
  res.message = m;
  return true;
}

bool FastExplorationFSM::goalServiceCallback(epic_planner::GoalService::Request& req,
                                             epic_planner::GoalService::Response& res) {
  goal_rth_ << req.x, req.y, req.z, req.yaw;
  has_goal_rth_ = true;

  char d[128];
  snprintf(d, sizeof(d), "goal=(%.2f, %.2f, %.2f) yaw=%.2f", req.x, req.y, req.z,
           req.yaw);
  elog_.log("EVENT", "/srv_goto goal received", d, 0.0, EventLogger::L_INFO, true);

  // Trigger state transition
  if (state_ == WAIT_TRIGGER || state_ == EXEC_TRAJ || state_ == PLAN_TRAJ_EXP) {
    transitState(PLAN_TRAJ_RTH, "Goal service called");
  }

  res.success = true;
  res.message = "Goal received, navigating to position";
  return true;
}

int FastExplorationFSM::callGoalPlanner() {
  ros::Time planning_start_time = ros::Time::now();

  // Check prerequisites
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) {
    local_reason_ = "odom node has no topo neighbors";
    return START_FAIL;
  }
  if (expl_manager_->ed_->global_tour_.size() < 2) {
    local_reason_ = "no global tour yet (RTH global plan pending)";
    return FAIL;
  }

  Eigen::Vector3d goal_pos = goal_rth_.head<3>();
  double goal_yaw = goal_rth_(3);

  // Call exploration manager's goal planning function to generate global_tour_
  int res = expl_manager_->planGoalPath(goal_pos, goal_yaw);
  if (res != SUCCEED) {
    local_reason_ = "RTH global: " + expl_manager_->ed_->diag_reason_;
    return res;
  }

  // Update next_goal_node_ from global_tour_
  expl_manager_->updateGoalNode();

  // Generate local trajectory using fast_searcher
  vector<Eigen::Vector3f> path_next_goal;
  res = planner_manager_->fast_searcher_->search(
      planner_manager_->topo_graph_->odom_node_,
      fd_->odom_vel_,
      expl_manager_->ed_->next_goal_node_,
      0.2, path_next_goal);

  if (res == ParallelBubbleAstar::NO_PATH) {
    local_reason_ = "fast-searcher: no path odom->next RTH node";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", local_reason_.c_str());
    return FAIL;
  } else if (res == ParallelBubbleAstar::START_FAIL) {
    local_reason_ = "fast-searcher: start(odom) in occupancy";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", local_reason_.c_str());
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    local_reason_ = "fast-searcher: RTH node in occupancy";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", local_reason_.c_str());
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    local_reason_ = "fast-searcher: timeout";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", local_reason_.c_str());
    return FAIL;
  }

  // Handle replanning from current trajectory
  auto info = &planner_manager_->local_data_;
  if (!fd_->static_state_) {
    double plan_finish_time_exp = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
    if (plan_finish_time_exp > info->duration_) {
      plan_finish_time_exp = info->duration_;
    }
    Eigen::Vector3d start_exp = info->minco_traj_.getPos(plan_finish_time_exp);
    path_next_goal.insert(path_next_goal.begin(), start_exp.cast<float>());
  }

  // Resample path to avoid too long segments
  vector<Eigen::Vector3f> path_next_goal_tmp;
  path_next_goal_tmp.push_back(path_next_goal[0]);
  for (int i = 1; i < path_next_goal.size();) {
    Eigen::Vector3f end_pt = path_next_goal_tmp.back();
    if ((path_next_goal[i] - end_pt).norm() > 1.0) {
      Eigen::Vector3f dir = (path_next_goal[i] - end_pt).normalized();
      path_next_goal_tmp.push_back(end_pt + 1.0 * dir);
    } else if ((path_next_goal[i] - end_pt).norm() < 0.01) {
      i++;
    } else {
      path_next_goal_tmp.push_back(path_next_goal[i]);
      i++;
    }
  }

  expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);

  // Plan trajectory
  int result;
  if (planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_, fd_->static_state_)) {
    traj_utils::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;

    traj_utils::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;

    local_reason_ = planner_manager_->last_plan_was_escape_ ? "escape traj (flyToSafeRegion)" : "";
    result = SUCCEED;
  } else {
    // 사유는 planner_manager 가 세팅 (예: "start not in corridor ..."). 이벤트
    // 로거가 변화 시에만 내보내므로 여기선 스로틀 콘솔만. (INC2에서 이 지점이
    // 1174회 ROS_ERROR 스팸이었음)
    local_reason_ = planner_manager_->last_plan_fail_reason_.empty()
                        ? "traj optimization failed"
                        : planner_manager_->last_plan_fail_reason_;
    ROS_WARN_THROTTLE(2.0, "[RTH] local plan failed: %s", local_reason_.c_str());
    result = FAIL;
  }

  // Block until minimum planning period has elapsed
  double elapsed = (ros::Time::now() - planning_start_time).toSec();
  if (elapsed < local_planning_min_period_) {
    double hold_time = local_planning_min_period_ - elapsed;
    ROS_DEBUG("[Planning Hz Limit] Holding for %.3f ms (planning took %.3f ms, min period %.3f ms)",
              hold_time * 1000.0, elapsed * 1000.0, local_planning_min_period_ * 1000.0);
    ros::Duration(hold_time).sleep();
  }
  return result;
}
