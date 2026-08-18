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
#include <frontier_manager/global_log.h>
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

bool FastExplorationFSM::insidePlanningBox(const Eigen::Vector3f &p) const {
  const auto &lp = planner_manager_->lidar_map_interface_->lp_;
  if (lp->planning_box_num_ <= 0)
    return true;
  for (int i = 0; i < lp->planning_box_num_; i++) {
    const Eigen::Vector3f &lo = lp->planning_box_min_boundary_vec_[i];
    const Eigen::Vector3f &hi = lp->planning_box_max_boundary_vec_[i];
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
      if (!insidePlanningBox(n->center_))
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
  EPIC_LOG_DEBUG(1, 1, "global.connectivity",
           "early-finish search reached=%zu origin_dist=%.2fm graph_dist=%.2fm max_disp=%.2fm",
           dist.size(), best_origin_dist, best_graph_dist, max_displacement_);
  if (!best)
    return false;
  if (require_gain &&
      best_origin_dist < max_displacement_ + early_finish_probe_min_gain_) {
    EPIC_LOG_DEBUG(1, 1, "global.evaluation", "early-finish candidate gains nothing "
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
  // [race: name-collision-shutdown] this is the node's FIRST publish and is
  // called from inside init() (right after early_finish_state_pub_ is
  // advertised) -- if a same-name node collision triggered an async shutdown
  // while init() is still running, the Publisher can already be invalid here.
  // See exploration_node.cpp for the full race.
  if (!ros::ok()) return;
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
      ROS_LOG_THROTTLE(1.0, ::ros::console::levels::Debug, "execution.fsm",
                       "[execution.fsm] waiting for odometry");
      return;
    }
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    ROS_LOG_THROTTLE(5.0, ::ros::console::levels::Debug, "execution.fsm",
                     "[execution.fsm] waiting for mission trigger");
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
        EPIC_LOG_WARN("execution.fsm", "takeoff timeout %.1fs near altitude "
                 "(z_err=%.2fm speed=%.2fm/s); starting exploration",
                 fp_->takeoff_timeout_, z_err, speed);
        fd_->static_state_ = true;
        leaveTakeoffHover("takeoff: timeout (near altitude)");
      } else {
        EPIC_LOG_ERROR_THROTTLE(2.0, "execution.fsm",
            "takeoff timeout %.1fs and not at altitude (z_err=%.2fm speed=%.2fm/s); "
            "holding hover, NOT exploring (check arming / OFFBOARD / thrust)",
            fp_->takeoff_timeout_, z_err, speed);
        // stay in TAKEOFF_HOVER; pubHoverCmd() keeps streaming the climb setpoint.
      }
    }
    break;
  }

  case FINISH: {
    // 조기 종료 구제: 자동 탐사 종료는 (retry 래치 안에서) 항상 EARLY_FINISH 를
    // 한 번 거쳐 selectFarthestReachableNode 에 판정을 맡긴다 — "가본 범위
    // (max_displacement_) + gain 을 넘는 도달 가능 미방문 노드가 있으면" probe,
    // 없으면 FAILED 경로로 즉시 정상 FINISH 에 합류한다.
    // 예전의 거리 게이트(max_displacement_ < thresh)는 폐지했다: 절대 임계는 환경
    // 규모를 모른다 — 0804 실비행 2회 모두 3~4.5m 진출 후 조기 종료(클러스터 15개
    // 잔존)했는데 3.0m 게이트가 닫혀 구제가 아예 안 떴다. 규모 인지형 판정은
    // gain 조건이 이미 하고 있으므로 게이트는 오탐 차단 역할이 없다.
    // (스냅샷/호버/MISSION 이벤트보다 먼저 판단해야 이벤트가 두 번 찍히거나
    // 불필요한 호버를 거치지 않는다.)
    // - explore_finished_ 조건 필수: 수동 /srv_rth 로 온 FINISH 는 구제 대상이 아니다.
    // - max_retry 래치 필수: 구제 후 또 짧게 끝나면 무한 반복이 된다. FAILED 로
    //   돌아온 경우에도 count 는 이미 올라가 있어 재진입하지 않는다.
    if (early_finish_enable_ && explore_finished_ &&
        early_finish_count_ < early_finish_max_retry_) {
      transitState(EARLY_FINISH, "FINISH: probe eligibility check (max_disp=" +
                   to_string(max_displacement_) + "m)");
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
        EPIC_LOG_INFO(1, 1, "execution.fsm",
                      "mission finish: trajectory server owns position hold; "
                      "FSM hold publisher kept ready for later recovery");
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
        EPIC_LOG_WARN_THROTTLE(5.0, "execution.fsm", "auto-RTH-land on but takeoff disabled "
                               "(no home recorded) -> position hold.");
      if (traj_server_owns_finish_cmd_) {
        EPIC_LOG_INFO_THROTTLE(2.0, 1, 1, "execution.fsm",
                               "mission finished; trajectory server holding position");
      } else {
        pubHoldCmd(finish_hover_pos_, finish_hover_yaw_);
        EPIC_LOG_INFO_THROTTLE(2.0, 1, 1, "execution.fsm",
                               "mission finished; holding position");
      }
      break;
    }

    // Auto sequence: hover at the finish point (fixed yaw -> no rotation -> no
    // yaw-divergence risk), then return home and land.
    EPIC_LOG_INFO_THROTTLE(2.0, 1, 1, "execution.fsm",
                           "exploration complete; hover %.1fs then RTH+land",
                           finish_hover_duration_);
    if (!traj_server_owns_finish_cmd_)
      pubHoldCmd(finish_hover_pos_, finish_hover_yaw_);

    if ((ros::Time::now() - finish_hover_start_).toSec() >= finish_hover_duration_) {
      goal_rth_ << takeoff_anchor_.x(), takeoff_anchor_.y(), rthApproachZ(),
          fd_->odom_yaw_;
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
      if (res == SUCCEED) {
        EPIC_LOG_INFO_THROTTLE(
            1.0, 0, 0, "local.cycle",
            "mode=EXPLORE result=%s plan_time=%.1fms goal_dist=%.1fm",
            rs, t_ms,
            (expl_manager_->ed_->next_goal_node_->center_ - fd_->odom_pos_)
                .norm());
      } else {
        EPIC_LOG_WARN_THROTTLE(
            2.0, "local.cycle",
            "mode=EXPLORE result=%s plan_time=%.1fms reason=%s", rs, t_ms,
            local_reason_.empty() ? "unspecified" : local_reason_.c_str());
      }
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
        if (retryWithUnreachableAmnesty()) {
          // 사면 후 재계획이 성공 -> FINISH 로 가지 않고 탐사를 계속한다.
          transitState(PLAN_TRAJ_EXP,
                       "PLAN_TRAJ_EXP: no frontier -> unreachable amnesty recovered",
                       true);
        } else {
          explore_finished_ = true;  // genuine exploration end -> enable auto RTH+land
          transitState(FINISH, "PLAN_TRAJ_EXP: no frontier");
          fd_->static_state_ = true;
        }
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
    if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) {
      noteRthFailure("odom node has no topology neighbor");
      return;
    }

    // 도달 판정. 홈복귀는 홈 xy와 RTH 접근 고도(1.0m AGL)를 둘 다
    // 만족해야 LAND로 넘어간다. /srv_goto 같은 일반 위치 이동은 3D 거리를 쓴다.
    Eigen::Vector3d cur = fd_->odom_pos_.cast<double>();
    Eigen::Vector3d gp = goal_rth_.head<3>();
    double dist, tol;
    double z_error = 0.0;
    bool goal_reached = false;
    if (returning_home_) {
      dist = (cur.head<2>() - gp.head<2>()).norm();  // xy distance to home
      tol = rth_land_xy_tol_;
      z_error = std::fabs(cur.z() - gp.z());
      goal_reached = dist < tol && z_error <= land_z_tol_;
    } else {
      dist = (cur - gp).norm();                      // 3D distance to service goal
      tol = goal_tolerance_;
      goal_reached = dist < tol;
    }
    // 진행상황: 0.5m 버킷이 바뀔 때만 이벤트 발행 (예전엔 사이클마다 INFO 스팸)
    {
      char sig[64], d[192];
      snprintf(sig, sizeof(sig), "dist_to_goal=%.1fm", std::floor(dist * 2.0) / 2.0);
      if (returning_home_) {
        snprintf(d, sizeof(d),
                 "auto-home xy=%.2fm/%.2fm z_error=%.2fm/%.2fm goal=(%.2f, %.2f, %.2f)",
                 dist, tol, z_error, land_z_tol_, gp.x(), gp.y(), gp.z());
      } else {
        snprintf(d, sizeof(d), "srv-goal(3D) dist=%.2fm tolerance=%.2fm "
                 "goal=(%.2f, %.2f, %.2f)", dist, tol, gp.x(), gp.y(), gp.z());
      }
      elog_.log("RTH", sig, d, 0.0);
    }

    if (goal_reached) {
      has_goal_rth_ = false;
      global_path_update_timer_.stop();  // Stop replanning timer

      // Publish RTH distance for metrics logging
      std_msgs::Float32 dist_msg;
      dist_msg.data = dist;
      rth_metrics_pub_.publish(dist_msg);

      if (returning_home_) {
        transitState(LAND, "RTH: home reached -> offboard descent");
        EPIC_LOG_INFO(1, 1, "execution.fsm",
                      "RTH home reached xy_error=%.3fm z_error=%.3fm; "
                      "aligning takeoff yaw", dist, z_error);
      } else {
        transitState(FINISH, "PLAN_TRAJ_RTH: goal reached");
        EPIC_LOG_INFO(1, 1, "execution.fsm", "RTH goal reached");
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
      if (res == SUCCEED)
        EPIC_LOG_INFO_THROTTLE(1.0, 0, 0, "local.cycle",
                               "mode=RTH result=%s plan_time=%.1fms", rs,
                               t_ms);
      else
        EPIC_LOG_WARN_THROTTLE(
            2.0, "local.cycle",
            "mode=RTH result=%s plan_time=%.1fms reason=%s", rs, t_ms,
            local_reason_.empty() ? "unspecified" : local_reason_.c_str());
    }

    if (res == SUCCEED) {
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      transitState(EXEC_TRAJ, "PLAN_TRAJ_RTH: plan success");
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == FAIL) {
      if (noteRthFailure(local_reason_.empty() ? "local planner failed"
                                               : local_reason_))
        break;
      // Still in PLAN_TRAJ_RTH state, keep replanning
      stopTraj();
      transitState(PLAN_TRAJ_RTH, "PLAN_TRAJ_RTH: plan failed", true);

    } else if (res == START_FAIL) {
      if (noteRthFailure(local_reason_.empty() ? "RTH start failed"
                                               : local_reason_))
        break;
      transitState(CAUTION, "PLAN_TRAJ_RTH: start failed", true);
    }
    break;
  }

  case EXEC_TRAJ: {
    // collision check
    double collision_time = planner_manager_->local_data_.duration_;
    const bool rotate_hold = planner_manager_->isRotateInPlaceHoldActive();
    bool safe = rotate_hold ||
                planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      // Return to appropriate planning state
      const bool rotate_needs_escape =
          planner_manager_->local_data_.rotate_in_place_ &&
          planner_manager_->lidar_map_interface_->getDisToOcc(
              fd_->odom_pos_) <=
              planner_manager_->gcopter_config_->dilateRadiusSoft;
      EXPL_STATE next_state = rotate_needs_escape
                                  ? CAUTION
                                  : (has_goal_rth_ ? PLAN_TRAJ_RTH
                                                   : PLAN_TRAJ_EXP);
      const double trajectory_elapsed = std::max(
          0.0, (ros::Time::now() - planner_manager_->local_data_.start_time_)
                   .toSec());
      const double collision_eta =
          std::max(0.0, collision_time - trajectory_elapsed);
      if (has_goal_rth_ && !rotate_needs_escape &&
          noteRthFailure("executing RTH trajectory became unsafe"))
        break;
      transitState(next_state,
                   "safetyCallback: not safe, traj_time:" +
                       to_string(collision_time) +
                       " eta:" + to_string(collision_eta),
                   true);
      if (!rotate_needs_escape &&
          collision_eta < fp_->replan_time_ + 0.2)
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
            EPIC_LOG_WARN("execution.trajectory",
                     "control error %.2fm > %.2fm; replanning from "
                     "current pose",
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
    exec_timer_.stop();

    // CAUTION owns its escape until the published trajectory actually ends.
    // Global planning already excludes CAUTION in updateTopoAndGlobalPath(),
    // and CloudOdomCallback does not transition out of CAUTION.  This substate
    // closes the remaining hole: CAUTION itself used to test clearance 0.2 s
    // after publication and hand control back to planning mid-trajectory.
    if (caution_phase_ == CAUTION_EXEC_ESCAPE) {
      const LocalTrajData &escape = planner_manager_->local_data_;
      const double elapsed =
          (ros::Time::now() - escape.start_time_).toSec();
      const bool same_trajectory =
          escape.traj_id_ == caution_escape_traj_id_;
      const bool trajectory_valid =
          same_trajectory && std::isfinite(elapsed) &&
          std::isfinite(escape.duration_) && escape.duration_ >= 0.0;

      if (trajectory_valid && elapsed < escape.duration_) {
        exec_timer_.start();
        break;
      }

      if (!trajectory_valid) {
        EPIC_LOG_WARN_THROTTLE(
            2.0, "local.collision",
            "CAUTION escape ownership lost/invalid; rebuilding from odometry");
      } else {
        const double dis2occ =
            planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_);
        const bool inside_planning_height =
            std::isfinite(escape.escape_planning_max_z_) &&
            fd_->odom_pos_.z() <= escape.escape_planning_max_z_ + 1.0e-5;
        if (dis2occ > planner_manager_->gcopter_config_->dilateRadiusSoft &&
            inside_planning_height) {
          EXPL_STATE next_state =
              has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
          transitState(next_state, "CAUTION: escape finished and safe");
          exec_timer_.start();
          break;
        }
      }

      // The owned trajectory ended but the actual odometry is still unsafe
      // (or ownership was invalidated). Re-enter IDLE and solve a fresh strict
      // observed-space FIRI from the current pose.
      caution_phase_ = CAUTION_IDLE;
      caution_escape_traj_id_ = 0;
      ++caution_escape_fail_count_;
    }

    const double dis2occ =
        planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_);
    // Obstacle clearance alone is not enough: tracking overshoot can leave the
    // vehicle outside the configured vehicle-center planning box. CAUTION owns
    // recovery back into that explicit volume.
    const bool inside_planning_box =
        planner_manager_->lidar_map_interface_->IsInPlanningBox(fd_->odom_pos_);
    if (dis2occ > planner_manager_->gcopter_config_->dilateRadiusSoft &&
        inside_planning_box) {
      EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
      transitState(next_state, "CAUTION: current odom is already safe");
      exec_timer_.start();
      break;
    }

    const ros::Time caution_now = ros::Time::now();
    const double caution_elapsed =
        caution_enter_time_.isZero()
            ? 0.0
            : (caution_now - caution_enter_time_).toSec();
    const bool caution_timed_out =
        caution_map_reset_timeout_ > 0.0 &&
        caution_elapsed >= caution_map_reset_timeout_;
    const bool caution_attempts_exhausted =
        caution_map_reset_failure_count_ > 0 &&
        caution_escape_fail_count_ >= caution_map_reset_failure_count_;
    if (caution_map_reset_enable_ &&
        (caution_timed_out || caution_attempts_exhausted)) {
      char reason[192];
      snprintf(reason, sizeof(reason),
               "CAUTION escalation elapsed=%.2fs failed_escapes=%d",
               caution_elapsed, caution_escape_fail_count_);
      // Reaching an occupancy epoch reset means the exploration map can no
      // longer be trusted as mission history.  Rebuild enough live geometry to
      // move safely, then abort exploration and return home.  A CAUTION escape
      // that succeeds before this point still resumes its original mission.
      beginMapRebuild(reason, true);
      exec_timer_.start();
      break;
    }

    // A failed strict-FIRI solve is expensive and cannot improve until more
    // sensor data arrives. Bound retry rate so the failure counter represents
    // distinct map observations instead of a 100 Hz busy loop.
    if (!caution_last_attempt_time_.isZero() &&
        (caution_now - caution_last_attempt_time_).toSec() <
            caution_retry_period_) {
      exec_timer_.start();
      break;
    }

    stopTraj();
    // Recovery must be anchored at the actual current odometry, never a
    // predicted point on the trajectory that just failed.
    caution_last_attempt_time_ = caution_now;
    bool success = planner_manager_->flyToSafeRegion(true);
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
      caution_phase_ = CAUTION_EXEC_ESCAPE;
      caution_escape_traj_id_ = info->traj_id_;
    } else {
      ++caution_escape_fail_count_;
    }
    exec_timer_.start();
    break;
  }
  case MAP_REBUILD: {
    // Keep the arrival heading: map recovery never commands a yaw turn.
    pubHoldCmd(map_rebuild_hold_pos_, map_rebuild_hold_yaw_);
    auto lio = planner_manager_->lidar_map_interface_;
    const uint64_t scans = lio->ld_->map_update_seq_ -
                           map_rebuild_start_update_seq_;
    const int points = lio->mapPointCount();
    const bool topology_ready = planner_manager_->topo_graph_->odom_node_ &&
        !planner_manager_->topo_graph_->odom_node_->neighbors_.empty();
    const double elapsed =
        (ros::Time::now() - map_rebuild_start_time_).toSec();
    const bool ready = elapsed >= map_rebuild_min_duration_ &&
                       scans >= static_cast<uint64_t>(map_rebuild_min_scans_) &&
                       points > 0 && topology_ready;
    if (ready) {
      char detail[192];
      snprintf(detail, sizeof(detail),
               "epoch=%llu elapsed=%.2fs scans=%llu points=%d topo_neighbors=%zu",
               static_cast<unsigned long long>(lio->ld_->map_epoch_), elapsed,
               static_cast<unsigned long long>(scans), points,
               planner_manager_->topo_graph_->odom_node_->neighbors_.size());
      elog_.log("MAP_RESET", "rebuild complete; planning resumed", detail,
                0.0, EventLogger::L_INFO, true);
      EPIC_LOG_INFO(1, 1, "execution.fsm", "MAP_REBUILD complete %s", detail);
      publishMapRebuildStatus("COMPLETE", detail);
      fd_->static_state_ = true;
      transitState(map_rebuild_resume_state_,
                   "MAP_REBUILD: fresh map/topology ready");
    } else {
      const ros::Time now = ros::Time::now();
      if (map_rebuild_last_status_time_.isZero() ||
          (now - map_rebuild_last_status_time_).toSec() >= 1.0) {
        char status[192];
        snprintf(status, sizeof(status),
                 "epoch=%llu elapsed=%.2fs scans=%llu/%d points=%d topo=%d",
                 static_cast<unsigned long long>(lio->ld_->map_epoch_), elapsed,
                 static_cast<unsigned long long>(scans), map_rebuild_min_scans_,
                 points, topology_ready ? 1 : 0);
        publishMapRebuildStatus("WAITING", status);
        map_rebuild_last_status_time_ = now;
      }
      EPIC_LOG_INFO_THROTTLE(
          1.0, 1, 1, "execution.fsm",
          "MAP_REBUILD waiting elapsed=%.2fs scans=%llu/%d points=%d topo=%d",
          elapsed, static_cast<unsigned long long>(scans),
          map_rebuild_min_scans_, points, topology_ready ? 1 : 0);
    }
    break;
  }
  case LAND: {
    stopTraj();
    global_path_update_timer_.stop();
    // NOTE: exec_timer_ 는 멈추지 않는다 — 이 콜백이 계속 돌아야 하강 셋포인트 발행과
    // 착지(disarm) 감지가 동작한다. (예전 exec_timer_.stop() 은 첫 틱에서 FSM 을
    // 정지시켜 "2Hz 재요청" 주석과 모순되는 잠재 버그였음.)
    //
    // 착지 완료: PX4 가 지면 감지 후 자동 disarm (조종자 수동 disarm 포함) -> LANDED.
    if (px4_seen_ && !px4_state_.armed) {
      transitState(LANDED, "LAND: touchdown & disarmed");
      break;
    }
    // 하강 본체. OFFBOARD 를 유지한 채 직접 내려간다. AUTO.LAND 전환 경로는 없다.
    runOffboardLanding();
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
      EPIC_LOG_INFO_THROTTLE(2.0, 1, 1, "execution.fsm", "%s", b);
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

  case PILOT_OVERRIDE: {
    // Terminal latch. PX4/pilot owns the vehicle now; keep publishing the state
    // heartbeat, but never plan, replace a trajectory, or leave this state.
    global_path_update_timer_.stop();
    break;
  }
  }
}

void FastExplorationFSM::clearRthFailures() {
  rth_failure_times_.clear();
  last_rth_failure_time_ = ros::Time(0);
}

bool FastExplorationFSM::noteRthFailure(const std::string &reason) {
  if (!rth_map_reset_enable_ || !has_goal_rth_ || state_ == MAP_REBUILD ||
      rth_map_reset_failure_count_ <= 0)
    return false;

  const ros::Time now = ros::Time::now();
  if (!last_rth_failure_time_.isZero() &&
      (now - last_rth_failure_time_).toSec() < rth_failure_min_interval_)
    return false;
  last_rth_failure_time_ = now;

  while (!rth_failure_times_.empty() &&
         (now - rth_failure_times_.front()).toSec() > rth_failure_window_)
    rth_failure_times_.pop_front();
  rth_failure_times_.push_back(now);

  char detail[224];
  snprintf(detail, sizeof(detail), "count=%zu/%d window=%.1fs reason=%s",
           rth_failure_times_.size(), rth_map_reset_failure_count_,
           rth_failure_window_, reason.c_str());
  elog_.log("RTH", "repeated planning/safety failure", detail, 0.5,
            EventLogger::L_WARN);
  if (static_cast<int>(rth_failure_times_.size()) <
      rth_map_reset_failure_count_)
    return false;

  return beginMapRebuild(std::string("RTH repeated failures: ") + reason);
}

bool FastExplorationFSM::beginMapRebuild(const std::string &reason,
                                         bool force_return_home) {
  if (!fd_->have_odom_ || state_ == PILOT_OVERRIDE || state_ == LAND ||
      state_ == LANDED || state_ == MAP_REBUILD)
    return false;

  auto lio = planner_manager_->lidar_map_interface_;
  const int points_before = lio->mapPointCount();
  map_rebuild_hold_pos_ = fd_->odom_pos_.cast<double>();
  map_rebuild_hold_yaw_ = fd_->odom_yaw_;

  if (force_return_home) {
    // A CAUTION severe enough to discard the occupancy epoch is a degraded
    // mission, not a reason to rediscover the cleared frontier history.  Arm
    // the same return-home-and-land path as /srv_rth. RTH preserves the current
    // heading; LAND restores takeoff_yaw_ after the 1.0m approach is complete.
    Eigen::Vector3d home = takeoff_anchor_;
    if (fp_->takeoff_height_ <= 0.0) {
      home = Eigen::Vector3d(
          0.0, 0.0, std::max(0.5, static_cast<double>(fd_->odom_pos_.z())));
    } else {
      home.z() = rthApproachZ();
    }
    goal_rth_ << home.x(), home.y(), home.z(), fd_->odom_yaw_;
    has_goal_rth_ = true;
    returning_home_ = true;
    explore_finished_ = false;
    if (early_finish_active_)
      clearEarlyFinishPath("ABORTED_BY_CAUTION_MAP_RESET");

    char rth_detail[192];
    snprintf(rth_detail, sizeof(rth_detail),
             "map reset degraded mission; RTH armed home=(%.2f, %.2f, %.2f)",
             home.x(), home.y(), home.z());
    elog_.log("RTH", "CAUTION map reset forced return home", rth_detail,
              0.0, EventLogger::L_WARN, true);
    EPIC_LOG_WARN("execution.fsm", "%s", rth_detail);
  }

  // CAUTION escalation above is latched to RTH. An RTH-triggered reset already
  // has has_goal_rth_ set and therefore also resumes RTH. Manual resets issued
  // during takeoff/yaw/finish retain their owning state instead of unexpectedly
  // starting exploration.
  if (has_goal_rth_)
    map_rebuild_resume_state_ = PLAN_TRAJ_RTH;
  else if (state_ == TAKEOFF_HOVER || state_ == YAW_ROTATE_INIT ||
           state_ == FINISH || state_ == EARLY_FINISH)
    map_rebuild_resume_state_ = state_;
  else
    map_rebuild_resume_state_ = PLAN_TRAJ_EXP;

  stopTraj();
  global_path_update_timer_.stop();
  fd_->static_state_ = true;
  caution_phase_ = CAUTION_IDLE;
  caution_escape_traj_id_ = 0;

  // Old global/local plans point into frontier/topology decisions made in the
  // previous occupancy epoch. Do not let a stale goal run while the map fills.
  auto ed = expl_manager_->ed_;
  ed->local_tour_.clear();
  ed->global_tour_.clear();
  ed->path_next_goal_.clear();
  // next_goal_node_ is a persistent virtual-goal adapter allocated once by
  // FastExplorationManager.  planGlobalPath()/updateGoalNode() repositions and
  // reconnects this same object for every new tour.  Clearing the pointer here
  // makes the first post-rebuild PLAN_TRAJ_EXP tick dereference null before a
  // new local trajectory is published.  Clearing global_tour_ is sufficient to
  // prevent the old goal from being executed while the new epoch is warming.
  ed->diag_result_ = "MAP_REBUILD";
  ed->diag_reason_ = reason;

  expl_manager_->frontier_manager_ptr_->resetForMapRebuild();
  planner_manager_->topo_graph_->prepareForMapRebuild(
      fd_->odom_pos_, fd_->odom_yaw_);
  const int deleted = lio->resetOccupancyMap();
  const int points_after = lio->mapPointCount();
  map_rebuild_start_update_seq_ = lio->ld_->map_update_seq_;
  map_rebuild_start_time_ = ros::Time::now();
  map_rebuild_last_status_time_ = ros::Time(0);
  ++map_reset_count_;
  clearRthFailures();

  char detail[320];
  snprintf(detail, sizeof(detail),
           "reason=%s reset_count=%d epoch=%llu points=%d->%d deleted=%d "
           "hold=(%.2f,%.2f,%.2f) yaw=%.2f resume=%s",
           reason.c_str(), map_reset_count_,
           static_cast<unsigned long long>(lio->ld_->map_epoch_),
           points_before, points_after, deleted, map_rebuild_hold_pos_.x(),
           map_rebuild_hold_pos_.y(), map_rebuild_hold_pos_.z(),
           map_rebuild_hold_yaw_,
           fd_->state_str_[int(map_rebuild_resume_state_)].c_str());
  elog_.log("MAP_RESET", "occupancy epoch reset; rebuilding", detail, 0.0,
            EventLogger::L_WARN, true);
  EPIC_LOG_WARN("execution.fsm", "MAP_RESET %s", detail);
  publishMapRebuildStatus("STARTED", detail);

  transitState(MAP_REBUILD, reason, true);
  // Cloud callbacks refill map/frontiers. The global timer only rebuilds
  // topology while MAP_REBUILD owns the vehicle.
  global_path_update_timer_.start();
  return true;
}

void FastExplorationFSM::publishMapRebuildStatus(
    const std::string &phase, const std::string &detail) {
  std_msgs::String msg;
  msg.data = "phase=" + phase + " " + detail;
  map_rebuild_status_pub_.publish(msg);
}

void FastExplorationFSM::pubHoldCmd(const Eigen::Vector3d &p, double yaw,
                                    double yaw_dot) {
  // FINISH may hand command ownership to traj_server, but later transitions
  // (EARLY_FINISH -> exploration -> MAP_REBUILD, for example) can require the
  // FSM hold stream again. Never let a stale/shutdown publisher turn a recovery
  // request into a roscpp assertion. Keeping the original publisher alive is
  // the normal path; this is a defensive repair for any future shutdown.
  if (!hover_cmd_pub_) {
    ros::NodeHandle nh;
    hover_cmd_pub_ =
        nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    EPIC_LOG_WARN("execution.fsm",
                  "hold publisher was invalid; re-advertised /position_cmd");
  }

  // Hold (p, yaw) as a position setpoint. px4_ctrl_bridge forwards this on
  // /position_cmd (it ignores the velocity field), so the drone holds pose. Yaw is
  // yaw is an angle setpoint (production bridge uses use_yawrate=false), so a new
  // angle rotates the vehicle and then holds it. yaw_dot remains zero here.
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

/* ===================== [offboard landing] ==================================
   배경: AUTO.LAND 착륙 시 "옆으로 샌다"는 현장 보고. 증상은 실재하나 원인 미확정.
   후보 A(MPC_TILTMAX_LND 12도)는 파라미터가 실재하지만 v1.13 소스상 적용 조건이
   TakeoffState 라 하강 중에는 45도일 가능성이 있어 판정 보류. 후보 B(위치 추정
   무효화 -> FlightTaskDescend, xy 제어 없음)가 원인이면 이 변경으로는 고쳐지지
   않는다. 판별 방법과 근거는 Update_log-0810.txt [1], [8]-5 참조.

   원인과 무관하게 얻는 것: xy/yaw를 이륙 기준에 계속 묶고,
   1.0m/0.5m/지면 z 위치 셋포인트를 직접 명령한 뒤 접지 즉시 강제 disarm 한다.

   AUTO.LAND 로의 전환/폴백 경로는 이 파일에 존재하지 않는다 (의도적으로 삭제).
   착륙 중 무언가 잘못되면 코드가 모드를 바꾸는 대신 하강 명령과 disarm 재시도를
   유지하면서 ERROR 로 조종자 개입을 요구한다. 그 아래에는 PX4 자체 failsafe
   (offboard 상실, 위치 상실)가 최종 안전망으로 그대로 남아 있다.

   강제 disarm 판정:
     AGL <= land_touch_alt_             AGL = odom z - 이륙 시점 지면 z
   또는 PX4 자체 land detector(/mavros/extended_state 의 ON_GROUND) -> 즉시.

   하강속도(|vz|) 조건은 쓰지 않는다. 현장 경험상 그 조건이 성립하는 구간이
   너무 빡빡해 접지 판정을 놓친다. 대신 z odom 을 신뢰한다 — 기체 z 는 PX4 가
   하방 ToF 와 융합해 내보내는 값이라 이 축의 절대 정확도가 확보되어 있다.
   ON_GROUND 분기는 그대로 남겨둔다: AGL 기준선이 어긋나 (1)이 영영 성립하지
   않는 경우의 유일한 대비책이다.
   ========================================================================== */

void FastExplorationFSM::mavrosExtendedStateCallback(
    const mavros_msgs::ExtendedState::ConstPtr &msg) {
  px4_landed_state_ = msg->landed_state;
  px4_ext_seen_ = true;
}

double FastExplorationFSM::rthApproachZ() const {
  if (land_ground_z_valid_)
    return land_ground_z_ + rth_approach_alt_;
  return takeoff_anchor_.z();
}

// AGL = 지면으로부터의 높이. 기준선 land_ground_z_ 는 트리거 시점(기체가 땅에
// 앉아 있을 때)의 odom z 이므로, AGL 0 은 "이륙 직전 자세로 정확히 복귀"를 뜻한다.
bool FastExplorationFSM::landAgl(double &agl) const {
  if (!land_ground_z_valid_)
    return false;
  agl = static_cast<double>(fd_->odom_pos_.z()) - land_ground_z_;
  return true;
}

// 착륙 위치 셋포인트. xy/yaw는 이륙 좌표/방향에 고정하고,
// z는 1.0m AGL, 0.5m AGL, 이륙 지면 z를 단계별로 직접 명령한다.
void FastExplorationFSM::pubLandSetpoint(double target_z) {
  pubHoldCmd(Eigen::Vector3d(land_xy_anchor_.x(), land_xy_anchor_.y(), target_z),
             land_yaw_);
}

// MAV_CMD_COMPONENT_ARM_DISARM(400) + param2=21196(force magic).
// 매직값은 PX4 의 사전검사를 전부 우회한다 — 공중에서 부르면 그대로 낙하하므로
// 호출부는 반드시 접지 판정을 통과한 뒤에만 부른다.
bool FastExplorationFSM::forceDisarm() {
  mavros_msgs::CommandLong c;
  c.request.broadcast = false;
  c.request.command = 400;      // MAV_CMD_COMPONENT_ARM_DISARM
  c.request.confirmation = 0;
  c.request.param1 = 0.0;       // 0 = disarm
  c.request.param2 = 21196.0;   // force magic
  return command_client_.call(c) && c.response.success;
}

void FastExplorationFSM::logLandingAlignment(
    const std::string &phase, const Eigen::Vector3d &current, double agl,
    double target_agl, double held, const std::string &timer_state,
    bool force_now) {
  const ros::Time now = ros::Time::now();
  const double dx = current.x() - land_xy_anchor_.x();
  const double dy = current.y() - land_xy_anchor_.y();
  const double target_z = land_ground_z_ + target_agl;
  const double dz = current.z() - target_z;
  const double xy_err = std::hypot(dx, dy);

  const double yaw_delta = static_cast<double>(fd_->odom_yaw_) - land_yaw_;
  const double yaw_err = std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
  const bool yaw_ok = std::fabs(yaw_err) <= land_yaw_tol_;
  const bool xyz_ok = xy_err <= land_precision_xy_tol_ &&
                      std::fabs(dz) <= land_z_tol_;

  const bool status_changed =
      !land_alignment_log_seen_ || phase != land_alignment_log_phase_ ||
      yaw_ok != land_alignment_log_yaw_ok_ ||
      xyz_ok != land_alignment_log_xyz_ok_;
  const bool periodic = land_alignment_log_last_.isZero() ||
                        (now - land_alignment_log_last_).toSec() >= 1.0;
  if (!force_now && !status_changed && !periodic)
    return;

  const char *yaw_status = yaw_ok ? "SATISFIED" : "UNSATISFIED";
  const char *xyz_status = xyz_ok ? "SATISFIED" : "UNSATISFIED";
  char yaw_detail[256];
  snprintf(yaw_detail, sizeof(yaw_detail),
           "current=%.2fdeg target=%.2fdeg error=%.2fdeg abs_error=%.2fdeg "
           "tol=%.2fdeg",
           static_cast<double>(fd_->odom_yaw_) * 180.0 / M_PI,
           land_yaw_ * 180.0 / M_PI, yaw_err * 180.0 / M_PI,
           std::fabs(yaw_err) * 180.0 / M_PI,
           land_yaw_tol_ * 180.0 / M_PI);
  elog_.log("LAND_YAW", phase + " status=" + yaw_status, yaw_detail, 0.0,
            EventLogger::L_INFO, true);

  char xyz_detail[512];
  if (held >= 0.0) {
    snprintf(xyz_detail, sizeof(xyz_detail),
             "current=(%.3f,%.3f,%.3f)m target=(%.3f,%.3f,%.3f)m "
             "error=(dx=%.3f,dy=%.3f,dz=%.3f)m agl=%.3fm target_agl=%.3fm "
             "xy_norm=%.3fm tol=(xy=%.3fm,z=%.3fm) hold=%.2f/%.2fs timer=%s",
             current.x(), current.y(), current.z(), land_xy_anchor_.x(),
             land_xy_anchor_.y(), target_z, dx, dy, dz, agl, target_agl,
             xy_err, land_precision_xy_tol_, land_z_tol_, held,
             land_precision_hold_, timer_state.c_str());
  } else {
    snprintf(xyz_detail, sizeof(xyz_detail),
             "current=(%.3f,%.3f,%.3f)m target=(%.3f,%.3f,%.3f)m "
             "error=(dx=%.3f,dy=%.3f,dz=%.3f)m agl=%.3fm target_agl=%.3fm "
             "xy_norm=%.3fm tol=(xy=%.3fm,z=%.3fm) hold=INACTIVE timer=%s",
             current.x(), current.y(), current.z(), land_xy_anchor_.x(),
             land_xy_anchor_.y(), target_z, dx, dy, dz, agl, target_agl,
             xy_err, land_precision_xy_tol_, land_z_tol_, timer_state.c_str());
  }
  elog_.log("LAND_XYZ", phase + " status=" + xyz_status, xyz_detail, 0.0,
            EventLogger::L_INFO, true);

  land_alignment_log_last_ = now;
  land_alignment_log_phase_ = phase;
  land_alignment_log_seen_ = true;
  land_alignment_log_yaw_ok_ = yaw_ok;
  land_alignment_log_xyz_ok_ = xyz_ok;
}

void FastExplorationFSM::runOffboardLanding() {
  const ros::Time now = ros::Time::now();
  const double precision_z = land_ground_z_ + land_precision_alt_;
  const double touchdown_z = land_ground_z_;

  switch (land_phase_) {

  case LAND_PHASE_ALIGN_YAW_AT_APPROACH: {
    const double approach_z = rthApproachZ();
    pubHoldCmd(Eigen::Vector3d(land_xy_anchor_.x(), land_xy_anchor_.y(), approach_z),
               land_yaw_);

    if (!fd_->have_odom_) {
      EPIC_LOG_ERROR_THROTTLE(1.0, "execution.px4",
          "odometry lost while aligning landing yaw — holding approach setpoint");
      return;
    }

    double agl = 0.0;
    if (!landAgl(agl)) {
      EPIC_LOG_ERROR_THROTTLE(1.0, "execution.px4",
          "no AGL reference — holding approach setpoint, landing blocked");
      return;
    }

    const Eigen::Vector3d cur = fd_->odom_pos_.cast<double>();
    const double xy_err = (cur.head<2>() - land_xy_anchor_.head<2>()).norm();
    const double z_err = std::fabs(agl - rth_approach_alt_);
    const double yaw_delta = static_cast<double>(fd_->odom_yaw_) - land_yaw_;
    const double yaw_err = std::fabs(std::atan2(std::sin(yaw_delta),
                                                std::cos(yaw_delta)));
    const bool aligned = xy_err <= land_precision_xy_tol_ &&
                         z_err <= land_z_tol_ &&
                         yaw_err <= land_yaw_tol_;
    logLandingAlignment("APPROACH_1.0M", cur, agl, rth_approach_alt_, -1.0,
                        "INACTIVE");

    if (aligned) {
      land_phase_ = LAND_PHASE_DESCEND_TO_PRECISION;
      char detail[176];
      snprintf(detail, sizeof(detail),
               "xy_err=%.3fm z_err=%.3fm yaw_err=%.2fdeg; descend to %.2fm AGL",
               xy_err, z_err, yaw_err * 180.0 / M_PI, land_precision_alt_);
      elog_.log("LAND", "takeoff yaw aligned at approach altitude", detail,
                0.0, EventLogger::L_INFO, true);
    }

    EPIC_LOG_INFO_THROTTLE(
        1.0, 1, 1, "execution.px4",
        "landing align: agl=%.2fm xy_err=%.3fm yaw_err=%.2fdeg "
        "limits=(%.2fm, %.2fdeg)",
        agl, xy_err, yaw_err * 180.0 / M_PI, land_precision_xy_tol_,
        land_yaw_tol_ * 180.0 / M_PI);
    break;
  }

  case LAND_PHASE_DESCEND_TO_PRECISION: {
    if (!fd_->have_odom_ || !land_ground_z_valid_) {
      pubHoldCmd(Eigen::Vector3d(land_xy_anchor_.x(), land_xy_anchor_.y(),
                                 rthApproachZ()), land_yaw_);
      EPIC_LOG_ERROR_THROTTLE(1.0, "execution.px4",
          "odometry/AGL reference lost before precision hover — descent paused");
      return;
    }

    double agl = 0.0;
    landAgl(agl);
    const Eigen::Vector3d cur = fd_->odom_pos_.cast<double>();
    const double xy_err = (cur.head<2>() - land_xy_anchor_.head<2>()).norm();
    pubLandSetpoint(precision_z);
    logLandingAlignment("DESCEND_TO_0.5M", cur, agl, land_precision_alt_,
                        -1.0, "INACTIVE");

    if (std::fabs(agl - land_precision_alt_) <= land_z_tol_) {
      land_phase_ = LAND_PHASE_HOLD_AT_PRECISION;
      land_precision_since_ = ros::Time(0);
      char detail[144];
      snprintf(detail, sizeof(detail), "agl=%.3fm xy_err=%.3fm target=%.2fm",
               agl, xy_err, land_precision_alt_);
      elog_.log("LAND", "precision hover altitude reached", detail, 0.0,
                EventLogger::L_INFO, true);
    }

    EPIC_LOG_INFO_THROTTLE(1.0, 1, 1, "execution.px4",
                           "landing descend-to-precision: agl=%.2fm "
                           "target=%.2fm xy_err=%.3f",
                           agl, land_precision_alt_, xy_err);
    break;
  }

  case LAND_PHASE_HOLD_AT_PRECISION: {
    pubHoldCmd(Eigen::Vector3d(land_xy_anchor_.x(), land_xy_anchor_.y(), precision_z),
               land_yaw_);
    if (!fd_->have_odom_ || !land_ground_z_valid_) {
      land_precision_since_ = ros::Time(0);
      EPIC_LOG_ERROR_THROTTLE(1.0, "execution.px4",
          "odometry/AGL reference lost during precision hover — landing blocked");
      return;
    }

    double agl = 0.0;
    landAgl(agl);
    const Eigen::Vector3d cur = fd_->odom_pos_.cast<double>();
    const double xy_err = (cur.head<2>() - land_xy_anchor_.head<2>()).norm();
    const bool altitude_ok =
        std::fabs(agl - land_precision_alt_) <= land_z_tol_;
    const bool xy_ok = xy_err <= land_precision_xy_tol_;
    double held = 0.0;

    // xy 허용오차와 0.5m 호버 조건이 동시에 연속으로 유지된
    // 시간만 인정한다. 한 틱이라도 벗어나면 1.5초 타이머를 다시 시작한다.
    if (altitude_ok && xy_ok) {
      if (land_precision_since_.toSec() < 1e-6)
        land_precision_since_ = now;
      held = (now - land_precision_since_).toSec();
    } else {
      land_precision_since_ = ros::Time(0);
    }

    logLandingAlignment("PRECISION_0.5M", cur, agl, land_precision_alt_, held,
                        (altitude_ok && xy_ok) ? "RUNNING" : "RESET",
                        held >= land_precision_hold_);

    if (held >= land_precision_hold_) {
      land_phase_ = LAND_PHASE_FINAL_DESCEND;
      land_final_descent_start_ = now;
      char detail[160];
      snprintf(detail, sizeof(detail),
               "xy_err=%.3fm <= %.3fm held=%.2fs; final descent",
               xy_err, land_precision_xy_tol_, held);
      elog_.log("LAND", "precision position stable", detail, 0.0,
                EventLogger::L_INFO, true);
    }

    EPIC_LOG_INFO_THROTTLE(1.0, 1, 1, "execution.px4",
                           "landing precision hold: agl=%.2fm xy_err=%.3fm "
                           "within=%d held=%.2f/%.2fs",
                           agl, xy_err, (altitude_ok && xy_ok) ? 1 : 0, held,
                           land_precision_hold_);
    break;
  }

  case LAND_PHASE_FINAL_DESCEND: {
    if (!fd_->have_odom_) {
      pubLandSetpoint(touchdown_z);
      EPIC_LOG_ERROR_THROTTLE(1.0, "execution.px4",
          "odometry lost during final descent — holding touchdown setpoint, "
          "take manual control");
      return;
    }
    double agl = 0.0;
    if (!landAgl(agl)) {
      pubLandSetpoint(precision_z);
      EPIC_LOG_ERROR_THROTTLE(1.0, "execution.px4",
          "no AGL reference during final descent — landing blocked at precision altitude");
      return;
    }

    const Eigen::Vector3d cur = fd_->odom_pos_.cast<double>();
    const double xy_err = (cur.head<2>() - land_xy_anchor_.head<2>()).norm();
    const double elapsed =
        land_final_descent_start_.isZero()
            ? 0.0
            : (now - land_final_descent_start_).toSec();
    // 아래 둘은 경고일 뿐 하강을 중단시키지 않는다. 착륙 도중 제어를 놓는 것이
    // 밀린 채로 내려앉는 것보다 위험하다.
    if (xy_err > land_xy_err_warn_)
      EPIC_LOG_ERROR_THROTTLE(1.0, "execution.px4",
          "landing xy tracking error large: %.2fm (controller not holding the anchor)",
          xy_err);
    if (elapsed > land_timeout_)
      EPIC_LOG_ERROR_THROTTLE(2.0, "execution.px4",
                              "descent taking too long: %.1fs agl=%.2fm", elapsed, agl);

    pubLandSetpoint(touchdown_z);

    // 강제 kill 판정: 현재 z <= 이륙 시점 지면 z + land_touch_alt_ 이면 즉시.
    // 또는 PX4 자체 land detector 가 ON_GROUND 를 선언하면 즉시 인정한다
    // (AGL 기준선이 어긋났을 때의 유일한 대비책).
    const bool px4_on_ground =
        px4_ext_seen_ &&
        px4_landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    const bool below_kill_height = (agl <= land_touch_alt_);
    const double kill_margin = agl - land_touch_alt_;
    const bool kill_ready = below_kill_height || px4_on_ground;
    const char *kill_reason = below_kill_height
                                  ? "HEIGHT"
                                  : (px4_on_ground ? "PX4_ON_GROUND" : "NONE");
    char kill_detail[256];
    snprintf(kill_detail, sizeof(kill_detail),
             "current_z=%.3fm kill_z=%.3fm agl=%.3fm kill_limit=%.3fm "
             "margin=%.3fm px4_on_ground=%d reason=%s elapsed=%.2fs",
             cur.z(), land_ground_z_ + land_touch_alt_, agl,
             land_touch_alt_, kill_margin, px4_on_ground ? 1 : 0,
             kill_reason, elapsed);
    elog_.log("LAND_KILL", kill_ready ? "READY" : "WAITING", kill_detail,
              1.0, kill_ready ? EventLogger::L_WARN : EventLogger::L_INFO);

    if (kill_ready) {
      land_phase_ = LAND_PHASE_DISARM;
      land_touchdown_confirmed_ = true;
      land_disarm_since_ = now;
      land_last_req_ = ros::Time(0);
      // vz 는 판정에 쓰지 않지만 사후 분석을 위해 남긴다.
      const double vz_meas = std::fabs(static_cast<double>(fd_->odom_vel_.z()));
      char d[192];
      snprintf(d, sizeof(d),
               "agl=%.3fm kill_limit=%.3fm vz=%.3fm/s px4_on_ground=%d xy_err=%.3fm",
               agl, land_touch_alt_, vz_meas, px4_on_ground ? 1 : 0, xy_err);
      elog_.log("LAND", "force-disarm height reached", d, 0.0,
                EventLogger::L_WARN, true);
      EPIC_LOG_WARN("execution.px4", "force-disarm condition met (%s)", d);
    }

    EPIC_LOG_INFO_THROTTLE(1.0, 1, 1, "execution.px4",
                           "offboard landing: agl=%.2fm z_target=%.2fm "
                           "xy_err=%.2f t=%.1fs",
                           agl, touchdown_z, xy_err, elapsed);
    break;
  }

  case LAND_PHASE_DISARM: {
    // 시뮬에는 mavros 가 없어 arming 서비스가 아예 존재하지 않는다. 접지 판정까지
    // 온 이상 그것이 착륙 완료다.
    if (!px4_seen_) {
      transitState(LANDED, "LAND: touchdown (sim: no mavros)");
      return;
    }

    // 접지 후에도 하강 명령을 유지한다. 명령을 끊으면 px4_ctrl_bridge 가
    // land_cmd_timeout 뒤 "현재 포즈 hold" 로 떨어져, 지면에 앉은 채 위치제어기가
    // 기체를 붙잡으려 들며 모터가 계속 돈다(전복 위험).
    pubLandSetpoint(touchdown_z);

    // 접지 판정 즉시 강제 disarm. 일반 disarm 은 PX4 land detector 의 동의
    // (LNDMC_TRIG_TIME, 기본 1.0s)가 있어야 통과하는데, 그만큼 모터가 지면에서
    // 더 도는 것이 전복 위험을 키운다.
    if ((now - land_last_req_).toSec() > 0.25) {
      land_last_req_ = now;
      if (forceDisarm())
        EPIC_LOG_WARN_THROTTLE(1.0, "execution.px4", "force disarm requested");
      else
        EPIC_LOG_WARN_THROTTLE(1.0, "execution.px4",
                               "force disarm not accepted; retrying");
    }

    // 강제 disarm 은 추정기를 참조하지 않는 경로다. 수 초간 실패한다면 원인은
    // mavros<->FCU 링크 단절이거나 해당 PX4 버전이 매직값을 안 받는 것뿐이다.
    // 어느 쪽도 코드가 스스로 풀 수 없으므로 조종자를 부른다.
    const double since = (now - land_disarm_since_).toSec();
    if (since > 3.0)
      EPIC_LOG_ERROR_THROTTLE(2.0, "execution.px4",
          "ON GROUND BUT STILL ARMED for %.1fs — take manual control (kill switch). "
          "mode=%s", since, px4_state_.mode.c_str());
    break;
  }

  default:
    // 도달 불가. 방어적으로 1.0m yaw 정렬부터 다시 확인한다.
    land_phase_ = LAND_PHASE_ALIGN_YAW_AT_APPROACH;
    break;
  }
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
  const bool prev_armed = px4_state_.armed;
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

    // OFFBOARD 이탈은 PX4/조종자가 제어권을 가져갔다는 뜻이다. 미션 중에만
    // terminal PILOT_OVERRIDE 로 latch 한다. LAND 는 의도적으로 하강 셋포인트로
    // mode 를 바꾸는 정상 경로이므로 아래 active-state 목록에서 제외한다.
    const bool lost_offboard_control =
        prev_mode == "OFFBOARD" &&
        (msg->mode != "OFFBOARD" || (prev_armed && !msg->armed));
    const bool planner_owned_flight =
        state_ == TAKEOFF_HOVER || state_ == YAW_ROTATE_INIT ||
        state_ == PLAN_TRAJ_EXP || state_ == EXEC_TRAJ || state_ == CAUTION ||
        state_ == FINISH || state_ == EARLY_FINISH || state_ == PLAN_TRAJ_RTH ||
        state_ == MAP_REBUILD;
    if (lost_offboard_control && fd_ && fd_->trigger_ && planner_owned_flight) {
      // Cut the trajectory server's current command short. PX4 already ignores
      // it outside OFFBOARD, and after the command becomes stale the bridge's
      // fallback is current-pose hold if OFFBOARD is ever selected again.
      stopTraj();
      global_path_update_timer_.stop();
      fd_->static_state_ = true;
      fd_->trigger_ = false;
      has_goal_rth_ = false;
      returning_home_ = false;
      explore_finished_ = false;
      caution_phase_ = CAUTION_IDLE;
      caution_escape_traj_id_ = 0;
      early_finish_force_requested_ = false;
      if (early_finish_active_)
        clearEarlyFinishPath("ABORTED_BY_PILOT_OVERRIDE");
      have_avoid_flag_ = false;
      avoid_flag_ = 0;

      transitState(PILOT_OVERRIDE,
                   "PX4 left OFFBOARD: planner frozen by pilot override", true);
      elog_.log("MISSION", "ABORTED/PILOT_OVERRIDE",
                "PX4 control left OFFBOARD; restart planner node to run a new mission",
                0.0, EventLogger::L_WARN, true);
    }
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
  nh.param("fsm/rth_approach_alt", rth_approach_alt_, 1.0);
  nh.param("fsm/land_z_tol", land_z_tol_, 0.05);

  /* [offboard landing] PX4 AUTO.LAND 대신 OFFBOARD 로 직접 하강 */
  double land_yaw_tol_deg = 3.0;
  nh.param("fsm/land_yaw_tol_deg",      land_yaw_tol_deg,       3.0);
  land_yaw_tol_ = land_yaw_tol_deg * M_PI / 180.0;
  nh.param("fsm/land_precision_alt",    land_precision_alt_,    0.5);
  nh.param("fsm/land_precision_xy_tol", land_precision_xy_tol_, 0.10);
  nh.param("fsm/land_precision_hold",   land_precision_hold_,   1.5);
  nh.param("fsm/land_touch_alt",    land_touch_alt_,    0.10);
  nh.param("fsm/land_xy_err_warn",  land_xy_err_warn_,  0.80);
  nh.param("fsm/land_timeout",      land_timeout_,      25.0);
  nh.param("fsm/caution_map_reset_enable", caution_map_reset_enable_, true);
  nh.param("fsm/caution_map_reset_timeout", caution_map_reset_timeout_, 6.0);
  nh.param("fsm/caution_map_reset_failure_count",
           caution_map_reset_failure_count_, 10);
  nh.param("fsm/caution_retry_period", caution_retry_period_, 0.5);
  nh.param("fsm/rth_map_reset_enable", rth_map_reset_enable_, true);
  nh.param("fsm/rth_map_reset_failure_count", rth_map_reset_failure_count_, 5);
  nh.param("fsm/rth_failure_window", rth_failure_window_, 3.0);
  nh.param("fsm/rth_failure_min_interval", rth_failure_min_interval_, 0.25);
  nh.param("fsm/map_rebuild_min_duration", map_rebuild_min_duration_, 1.0);
  nh.param("fsm/map_rebuild_min_scans", map_rebuild_min_scans_, 10);
  caution_map_reset_timeout_ = std::max(0.0, caution_map_reset_timeout_);
  caution_map_reset_failure_count_ =
      std::max(0, caution_map_reset_failure_count_);
  caution_retry_period_ = std::max(0.05, caution_retry_period_);
  rth_map_reset_failure_count_ = std::max(0, rth_map_reset_failure_count_);
  rth_failure_window_ = std::max(0.1, rth_failure_window_);
  rth_failure_min_interval_ = std::max(0.05, rth_failure_min_interval_);
  map_rebuild_min_duration_ = std::max(0.0, map_rebuild_min_duration_);
  map_rebuild_min_scans_ = std::max(1, map_rebuild_min_scans_);
  land_touch_alt_ = std::max(0.0, land_touch_alt_);
  land_precision_alt_ = std::max(land_touch_alt_ + 0.05, land_precision_alt_);
  rth_approach_alt_ = std::max(land_precision_alt_ + 0.05, rth_approach_alt_);
  land_z_tol_ = std::max(0.01, land_z_tol_);
  land_yaw_tol_ = std::min(M_PI, std::max(0.0, land_yaw_tol_));
  land_precision_xy_tol_ = std::max(0.01, land_precision_xy_tol_);
  land_precision_hold_ = std::max(0.0, land_precision_hold_);
  // EARLY_FINISH (조기 종료 구제). 기본값은 기존 동작과 같도록 잡되 enable 은 true —
  // 끄려면 fsm/early_finish_enable: false.
  nh.param("fsm/early_finish_enable", early_finish_enable_, true);
  // [YAW_ROTATE_INIT] 이륙 후 제자리 360도 초기 관측 회전
  nh.param("fsm/yaw_rotate_init_enable", yaw_rotate_init_enable_, true);
  nh.param("fsm/yaw_rotate_init_rate", yaw_rotate_init_rate_, 0.5);
  nh.param("fsm/early_finish_probe_min_gain", early_finish_probe_min_gain_, 1.0);
  nh.param("fsm/early_finish_visited_radius", early_finish_visited_radius_, 1.5);
  nh.param("fsm/early_finish_max_retry", early_finish_max_retry_, 1);
  // FINISH-time amnesty (도달불가 래치 사면). global_planning.cpp 의 is_reachable_
  // 게이트는 한 번 false 가 되면 재평가 경로가 없는 단방향 래치라, FINISH 진입
  // 직전 딱 한 번만 전부 풀고 전역계획을 재시도한다. 자세한 재발화 방지 근거는
  // retryWithUnreachableAmnesty() 주석 참조.
  nh.param("fsm/unreachable_amnesty_enable", unreachable_amnesty_enable_, true);
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
  EPIC_LOG_INFO(1, 1, "execution.fsm",
                "configured local_plan_rate=%.1fHz min_period=%.4fs",
                local_planning_max_hz_, local_planning_min_period_);
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
                     "YAW_ROTATE_INIT", "PILOT_OVERRIDE", "MAP_REBUILD"};
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
  srv_reset_map_ = nh.advertiseService(
      "/srv_reset_map", &FastExplorationFSM::resetMapServiceCallback, this);
  map_rebuild_status_pub_ = nh.advertise<std_msgs::String>(
      "/planning/map_rebuild_status", 10, true);
  publishMapRebuildStatus("IDLE", "reset_count=0");
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);

  // /mavros/state 로 armed/mode 를 관측한다 (착륙 완료 판정: armed == false).
  mavros_state_sub_ = nh.subscribe("/mavros/state", 10,
                                   &FastExplorationFSM::mavrosStateCallback, this);

  /* [offboard landing] 강제 disarm 경로와 PX4 자체 land detector.
     - /mavros/cmd/command    : MAV_CMD_COMPONENT_ARM_DISARM + 매직값 21196.
     - /mavros/extended_state : PX4 land detector 의 ON_GROUND.
     시뮬에는 mavros 가 없어 서비스 호출이 실패하는데, 그 경우는
     runOffboardLanding() 이 접지 판정만으로 LANDED 로 마무리한다. */
  command_client_ = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  mavros_ext_state_sub_ =
      nh.subscribe("/mavros/extended_state", 10,
                   &FastExplorationFSM::mavrosExtendedStateCallback, this);

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
    EPIC_LOG_DEBUG(1, 1, "sensor.lidar",
                   "FSM subscribes cropped cloud=%s raw=%s",
                   cropped_topic.c_str(), cloud_topic.c_str());
    cloud_topic = cropped_topic;
  }
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      nh, cloud_topic, 1));
  double odom_sync_max_stamp_skew = 5.0;
  nh.param("fsm/odom_sync_max_stamp_skew", odom_sync_max_stamp_skew, 5.0);
  odom_sub_.reset(new StampNormalizingOdomFilter());
  odom_sub_->subscribe(nh, odom_topic, 5, odom_sync_max_stamp_skew);
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
    nh.param("planning_box_0/down", bx_dn, std::vector<double>());
    nh.param("planning_box_0/up", bx_up, std::vector<double>());
    std::string odom_t, cloud_t;
    nh.param("odometry_topic", odom_t, std::string("?"));
    nh.param("cloud_topic", cloud_t, std::string("?"));

    char l[288];
    param_lines_.clear();
    if (bx_dn.size() == 3 && bx_up.size() == 3)
      snprintf(l, sizeof(l), "box | down=[%g, %g, %g] up=[%g, %g, %g]", bx_dn[0],
               bx_dn[1], bx_dn[2], bx_up[0], bx_up[1], bx_up[2]);
    else
      snprintf(l, sizeof(l), "planning_box | (planning_box_0 params missing)");
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

    snprintf(l, sizeof(l),
             "landing_stages | approach=%.2fm z_tol=%.2fm yaw_tol=%.1fdeg "
             "precision=%.2fm xy_tol=%.2fm hold=%.2fs",
             rth_approach_alt_, land_z_tol_,
             land_yaw_tol_ * 180.0 / M_PI, land_precision_alt_,
             land_precision_xy_tol_, land_precision_hold_);
    param_lines_.push_back(l);

    snprintf(l, sizeof(l),
             "map_recovery | caution_enable=%d timeout=%.1fs failures=%d "
             "retry=%.2fs rth_enable=%d failures=%d/%.1fs "
             "rebuild_min=%.1fs/%dscans",
             caution_map_reset_enable_ ? 1 : 0,
             caution_map_reset_timeout_, caution_map_reset_failure_count_,
             caution_retry_period_, rth_map_reset_enable_ ? 1 : 0,
             rth_map_reset_failure_count_, rth_failure_window_,
             map_rebuild_min_duration_,
             map_rebuild_min_scans_);
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

// FINISH 직전 마지막 방어선 — "도달불가" 낙인(ClusterInfo::is_reachable_=false)을
// 한 번 사면(赦免)하고 전역계획을 딱 한 번 재시도한다.
//
// 배경: global_planning.cpp:63 의 게이트(`if (!cluster->is_reachable_) continue;`)
// 때문에 is_reachable_ 은 한 번 false 가 되면(frontier_manager.cpp 의
// CR_NO_CANDIDATE / CR_NO_VISIBILITY / CR_ODOM_DRIFT / CR_TOPO_UNREACHABLE) 재평가
// 경로가 없는 단방향 래치다. 실비행(0804, bag 1970-01-01-09-10-25)에서 15개 중
// 8개 클러스터가 7초 안에 이 낙인을 먹었고, 결국 GLOBAL result=NO_REACHABLE_VP 로
// 도달 가능한 프론티어가 남았는데도 조기 FINISH 됐다.
//
// 래치 자체(단방향 문제)는 여기서 고치지 않는다 — FINISH 진입을 막기 위한 1회성
// 구제일 뿐이다. 별도의 시간 기반 rate limit 은 두지 않는다: 사면 직후 라운드는
// 모든 클러스터가 is_reachable_=true 상태로 평가되므로 vp_stats_.unreachable_pre
// 가 0 이 된다. 따라서 prev_unreachable > 0 게이트만으로 "직전 사면 결과에 대고
// 연달아 또 사면하는" 경우가 자연히 차단된다. 다시 사면이 발화하려면 클러스터가
// 실제로 한 번 더 낙인을 먹어야 한다. 시간 기반 rate limit 을 두면 오히려 "3초
// 안에 두 번째 NO_FRONTIER 가 오면 사면 없이 그대로 FINISH" 라는, 이 기능이
// 막으려던 조기종료를 다시 만들어내므로 두지 않는다.
bool FastExplorationFSM::retryWithUnreachableAmnesty() {
  if (!unreachable_amnesty_enable_)
    return false;

  auto frt = expl_manager_->frontier_manager_ptr_;
  // 새 카운터를 두지 않고 마지막 라운드의 pipeline 통계(prev_unreachable)를 그대로
  // 재사용한다 (publishExplDiag()/logGlobalPlanEvent() 가 쓰는 것과 같은 값).
  const int prev_unreachable = frt->vp_stats_.unreachable_pre;
  if (prev_unreachable <= 0)
    return false;  // 용서할 클러스터가 없음 -> 계획 주기를 낭비하지 않는다

  int forgiven = 0;
  for (auto &cluster : frt->cluster_list_) {
    if (!cluster->is_reachable_) {
      cluster->is_reachable_ = true;
      forgiven++;
    }
  }

  Eigen::Vector3d odom = fd_->odom_pos_.cast<double>();
  Eigen::Vector3d vel = fd_->odom_vel_.cast<double>();
  int res = expl_manager_->planGlobalPath(odom, vel);
  const bool ok = (res != NO_FRONTIER);

  auto ed = expl_manager_->ed_;
  char d[288];
  snprintf(d, sizeof(d),
           "forgiven=%d(prev_unreachable=%d) re-run=%s clusters=%d(reachable %d) "
           "viewpoints=%d(path_reachable %d)",
           forgiven, prev_unreachable, ok ? "OK" : "NO_FRONTIER",
           ed->diag_num_clusters_, ed->diag_num_clusters_reachable_,
           ed->diag_num_viewpoints_, ed->diag_num_reachable_vp_);
  elog_.log("AMNESTY",
            ok ? "unreachable latch cleared -> global plan recovered, resume exploring"
               : "unreachable latch cleared -> still no frontier, proceeding to FINISH",
            d, 0.0, ok ? EventLogger::L_INFO : EventLogger::L_WARN, true);

  return ok;
}

void FastExplorationFSM::updateTopoAndGlobalPath() {
  // PILOT_OVERRIDE is terminal: unlike other inactive states, do not restart
  // this timer after visualization housekeeping.
  if (state_ == PILOT_OVERRIDE) {
    global_path_update_timer_.stop();
    return;
  }

  // WAIT_TRIGGER 제외: 트리거(2D Nav Goal) 전에는 토포/글로벌 경로 갱신을 하지 않는다.
  // TAKEOFF_HOVER 중에는 map/frontier(CloudOdomCallback)와 함께 topology
  // skeleton/odom-node 연결도 미리 유지한다. 단, 아래 topology 갱신
  // 직후 return하여 global planning, CAUTION 전이, trajectory 발행은 hover
  // 완료 후까지 금지한다. 예전에는 hover 중 topology를 비워 두어
  // 09-06-08에서 hover 종료 10 ms 후 `odom_node no nbrs` CAUTION이 발생했다.
  // EARLY_FINISH also needs one current topology update before selecting its path.
  if (!(state_ == TAKEOFF_HOVER || state_ == PLAN_TRAJ_EXP || state_ == PLAN_TRAJ_RTH ||
        state_ == EXEC_TRAJ || state_ == FINISH || state_ == EARLY_FINISH ||
        state_ == MAP_REBUILD)) {
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

  // Topology-only warmup: TAKEOFF_HOVER owns /position_cmd and must not be
  // interrupted by exploration planning or a transient orphaned odom node.
  if (state_ == TAKEOFF_HOVER || state_ == MAP_REBUILD) {
    if (expl_manager_->ep_->view_graph_)
      planner_manager_->graph_visualizer_->vizGraph(planner_manager_->topo_graph_);
    global_path_update_timer_.start();
    return;
  }

  // The ordinary global update intentionally replaces a trajectory after the
  // short replan window (~0.5 s). A rotate recovery needs its full smooth yaw
  // duration (up to 8 s), otherwise every replacement restarts at zero yaw
  // rate and the vehicle barely turns. Keep map/topology maintenance above,
  // but defer path replacement until this bounded hold finishes.
  if (planner_manager_->isRotateInPlaceHoldActive()) {
    global_path_update_timer_.start();
    return;
  }

  // Rotation changes visibility, not position. If the freshly observed map
  // still places the occupied current point inside the configured soft margin,
  // use the FSM's existing positional recovery instead of publishing an
  // ordinary trajectory that its t=0 collision check must immediately reject.
  // CAUTION exits only after this same configured margin is cleared.
  if (planner_manager_->local_data_.rotate_in_place_ &&
      planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) <=
          planner_manager_->gcopter_config_->dilateRadiusSoft) {
    transitState(CAUTION, "rotate complete: current point needs positional escape");
    global_path_update_timer_.start();
    return;
  }

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
    // Home return has no terminal heading requirement. Preserve the heading at
    // each replanning instant instead of rotating back to the takeoff yaw.
    const double goal_yaw = returning_home_ ? fd_->odom_yaw_ : goal_rth_(3);
    int res = expl_manager_->planGoalPath(goal_rth_.head<3>(), goal_yaw);
    last_plan_ms_ = (ros::Time::now() - t_rth).toSec() * 1000.0;
    logGlobalPlanEvent(res, last_plan_ms_);
    publishExplDiag(); // RTH 중에도 HUD/진단 유지 (예전엔 여기서 끊겨 분석 공백)
    if (res == SUCCEED && state_ != WAIT_TRIGGER) {
      transitState(PLAN_TRAJ_RTH, "planGoalPath: succeed");
    } else if (res != SUCCEED) {
      if (noteRthFailure("global RTH path failed")) {
        global_path_update_timer_.start();
        return;
      }
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
    EPIC_LOG_DEBUG(1, 1, "global.connectivity",
             "topology update=%.2fms odom_vertex=%.2fms",
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
      if (retryWithUnreachableAmnesty()) {
        // 사면 후 재계획이 성공 -> 정상 SUCCEED 경로와 동일하게 탐사를 계속한다.
        if (expl_manager_->ed_->diag_num_reachable_vp_ > 0)
          frontiers_ever_seen_ = true;
        transitState(PLAN_TRAJ_EXP,
                     "planGlobalPath: no frontier -> unreachable amnesty recovered");
      } else {
        explore_finished_ = true;  // genuine exploration end -> enable auto RTH+land
        transitState(FINISH, "planGlobalPath: no frontier");
      }
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
      EPIC_LOG_DEBUG(1, 1, "global.connectivity.profile",
                     "%s caps[conn=%.1fms insert=%.1fms]", rep.c_str(),
                     astar_conn_timeout_ms_, astar_insert_timeout_ms_);
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

bool FastExplorationFSM::resetMapServiceCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  (void)req;
  const bool eligible =
      state_ == TAKEOFF_HOVER || state_ == YAW_ROTATE_INIT ||
      state_ == PLAN_TRAJ_EXP || state_ == PLAN_TRAJ_RTH ||
      state_ == EXEC_TRAJ || state_ == CAUTION || state_ == FINISH ||
      state_ == EARLY_FINISH;
  if (!eligible || !fd_->have_odom_) {
    res.success = false;
    res.message = "rejected in state " + fd_->state_str_[int(state_)] +
                  (fd_->have_odom_ ? "" : ": no odometry");
    return true;
  }

  if (!beginMapRebuild("manual /srv_reset_map")) {
    res.success = false;
    res.message = "map reset could not start";
    return true;
  }
  auto lio = planner_manager_->lidar_map_interface_;
  char message[192];
  snprintf(message, sizeof(message),
           "MAP_REBUILD started: epoch=%llu points_now=%d; waiting for >=%d "
           "fresh scans and topology reconnection",
           static_cast<unsigned long long>(lio->ld_->map_epoch_),
           lio->mapPointCount(), map_rebuild_min_scans_);
  res.success = true;
  res.message = message;
  return true;
}

// 원터치 홈복귀: `rosservice call /srv_rth` (인자 없음).
// 비행 중 어느 상태에서든 즉시 이륙 지점(takeoff anchor)으로 복귀하고,
// 홈 xy 반경(rth_land_xy_tol) 도달 시 LAND(OFFBOARD 하강) -> LANDED (자동 RTH와 동일 경로).
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

  // 홈 = 이륙 앵커 (x0, y0, 지면+RTH 접근고도). takeoff 비활성
  // 구성이면 지면 기준을 기록하지 못했으므로 (0,0,현재고도) 폴백.
  Eigen::Vector3d home = takeoff_anchor_;
  double home_yaw = fd_->odom_yaw_;
  if (fp_->takeoff_height_ <= 0.0) {
    home = Eigen::Vector3d(0.0, 0.0, std::max(0.5, (double)fd_->odom_pos_.z()));
  } else {
    home.z() = rthApproachZ();
  }
  goal_rth_ << home.x(), home.y(), home.z(), home_yaw;
  has_goal_rth_ = true;
  clearRthFailures();
  returning_home_ = true;     // 홈 xy 도달 -> LAND(OFFBOARD 하강) -> LANDED
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
  snprintf(m, sizeof(m), "returning home to (%.2f, %.2f, %.2f); offboard descent on arrival",
           home.x(), home.y(), home.z());
  res.success = true;
  res.message = m;
  return true;
}

bool FastExplorationFSM::goalServiceCallback(epic_planner::GoalService::Request& req,
                                             epic_planner::GoalService::Response& res) {
  goal_rth_ << req.x, req.y, req.z, req.yaw;
  has_goal_rth_ = true;
  clearRthFailures();
  // /srv_goto is a positional goal with an explicit terminal yaw, not the
  // return-home-and-land flow. Clear any stale RTH latch before planning it.
  returning_home_ = false;
  explore_finished_ = false;

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
  // RTH keeps the current/arrival heading; /srv_goto still honors its explicit
  // requested yaw because returning_home_ is false for that path.
  double goal_yaw = returning_home_ ? fd_->odom_yaw_ : goal_rth_(3);

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
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return FAIL;
  } else if (res == ParallelBubbleAstar::START_FAIL) {
    local_reason_ = "fast-searcher: start(odom) in occupancy";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    local_reason_ = "fast-searcher: RTH node in occupancy";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    local_reason_ = "fast-searcher: timeout";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return FAIL;
  }

  // planExploreTraj anchors the complete new plan to one live-odometry
  // snapshot. A predicted sample from the previous command must not be
  // prepended, otherwise the RTH guide and optimizer use different starts.
  auto info = &planner_manager_->local_data_;

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
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle",
                           "RTH local plan failed: %s", local_reason_.c_str());
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
