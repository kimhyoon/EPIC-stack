/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-04-14 21:44:58
 * @LastEditTime: 2024-04-15 13:30:53
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <frontier_manager/frontier_manager.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
size_t ByteArrayRaw::size = 0;
void FrontierManager::init(ros::NodeHandle &nh, LIOInterface::Ptr &lio_interface,
                           TopoGraph::Ptr graph) {
  nh_ = nh;
  graph_ = graph;
  lidar_map_interface_ = lio_interface;
  nh.getParam("FrontierManager/cell_size", frtp_.cell_size_);
  if (!std::isfinite(frtp_.cell_size_) ||
      frtp_.cell_size_ <= lidar_map_interface_->lp_->vector_norm_eps_) {
    ROS_FATAL("[FrontierManager] FrontierManager/cell_size must be finite and "
              "greater than numerical/vector_norm_eps.");
    throw std::runtime_error("invalid FrontierManager/cell_size");
  }
  frtp_.inv_cell_size_ = 1.0 / frtp_.cell_size_;
  nh.getParam("FrontierManager/noise_cell_range", frtp_.noise_cell_range_);
  nh.getParam("FrontierManager/good_observation_direction_score",
              frtp_.good_observation_direction_score_);
  nh.getParam("FrontierManager/good_observation_trust_length",
              frtp_.good_observation_trust_length_);
  nh.getParam("FrontierManager/good_observation_force_trust_length",
              frtp_.good_observation_force_trust_length_);
  // [feature: split-trust-length] 뷰포인트 평가 전용 거리. 미설정 시 기존처럼
  // good_observation_force_trust_length 를 그대로 써서 동작이 바뀌지 않게 한다.
  frtp_.viewpoint_dir_trust_length_ = frtp_.good_observation_force_trust_length_;
  nh.param("FrontierManager/viewpoint_dir_trust_length",
           frtp_.viewpoint_dir_trust_length_,
           frtp_.good_observation_force_trust_length_);
  nh.param("FrontierManager/viewpoint_min_visible_cells",
           frtp_.viewpoint_min_visible_cells_, 3);
  ROS_INFO("[FrontierManager] trust lengths: mapping(force)=%.2fm "
           "viewpoint(FRONTIER_DIR)=%.2fm  min_visible_cells=%d",
           frtp_.good_observation_force_trust_length_,
           frtp_.viewpoint_dir_trust_length_,
           frtp_.viewpoint_min_visible_cells_);
  nh.getParam("FrontierManager/update_length", frtp_.update_length_);
  // [feature: box-margin] yaml 로만 활성화 (키 없으면 0 = 기존 동작).
  nh.param("FrontierManager/box_boundary_margin", frtp_.box_boundary_margin_, 0);
  nh.getParam("FrontierManager/view_frt", frtp_.view_frt_);
  nh.getParam("FrontierManager/view_cluster", frtp_.view_cluster_);

  nh.getParam("FrontierManager/cluster_min_radius", frtp_.cluster_min_radius_);
  nh.getParam("FrontierManager/cluster_min_size", frtp_.cluster_min_size_);
  nh.getParam("FrontierManager/cluster_max_size", frtp_.cluster_radius_);
  nh.getParam("FrontierManager/cluster_direction_radius",
              frtp_.cluster_direction_radius_);
  nh.getParam("FrontierManager/cluster_minmum_point_num",
              frtp_.cluster_minmum_point_num_);

  // frt_cluster_ptr_.reset(new FrontierCluster);
  // frt_cluster_ptr_->init(nh);

  nh.getParam("ViewpointManager/sample_pillar_height_layer_num",
              vpp_.sample_pillar_height_layer_num_);
  nh.getParam("ViewpointManager/sample_pillar_radius_layer_num",
              vpp_.sample_pillar_radius_layer_num_);
  nh.getParam("ViewpointManager/sample_pillar_circle_sample_num",
              vpp_.sample_pillar_circle_sample_num_);
  nh.getParam("ViewpointManager/sample_pillar_max_height",
              vpp_.sample_pillar_max_height_);
  nh.getParam("ViewpointManager/sample_pillar_min_height",
              vpp_.sample_pillar_min_height_);
  nh.getParam("ViewpointManager/sample_pillar_min_radius",
              vpp_.sample_pillar_min_radius_);
  nh.getParam("ViewpointManager/sample_pillar_max_radius",
              vpp_.sample_pillar_max_radius_);
  // 뷰포인트-장애물 최소 클리어런스. 없으면 기존 하드코딩값 0.9 유지(동작 불변).
  nh.param("ViewpointManager/min_obstacle_clearance",
           vpp_.min_obstacle_clearance_, 0.9f);

  nh.getParam("ViewpointManager/consider_range", vpp_.consider_range_);
  nh.getParam("ViewpointManager/global_recluster_size",
              vpp_.global_recluster_size_);
  nh.getParam("ViewpointManager/local_tsp_size", vpp_.local_tsp_size_);
  // [feature: vp-viz] 후보 시각화 (viewpoint_candidates 토픽)
  nh.param("ViewpointManager/viz_candidates", vpp_.viz_candidates_, true);
  nh.param("ViewpointManager/viz_max_yaw_arrows", vpp_.viz_max_yaw_arrows_, 300);
  nh.param("ViewpointManager/viz_max_points_per_status",
           vpp_.viz_max_points_per_status_, 4000);
  // [feature: topo-timeout] 뷰포인트 도달성 판정의 topo A* 예산 [s].
  // 키가 없으면 3e-4 = 기존 하드코딩 값이라 동작이 바뀌지 않는다.
  nh.param("ViewpointManager/reachability_search_timeout",
           vpp_.reachability_search_timeout_, 3e-4);
  // [feature: vp-reached-clear] 도달-후-미해소 클러스터 강제 해소.
  nh.param("ViewpointManager/vp_reached_clear_enable",
           vpp_.vp_reached_clear_enable_, true);
  nh.param("ViewpointManager/vp_reached_pos_tol", vpp_.vp_reached_pos_tol_,
           0.3f);
  float vp_reached_yaw_deg = 20.0f;
  nh.param("ViewpointManager/vp_reached_yaw_tol_deg", vp_reached_yaw_deg,
           20.0f);
  vpp_.vp_reached_yaw_tol_ = vp_reached_yaw_deg * static_cast<float>(M_PI) / 180.0f;
  ROS_INFO("[FrontierManager] vp-reached clear: %s  pos_tol=%.2fm yaw_tol=%.1fdeg",
           vpp_.vp_reached_clear_enable_ ? "on" : "off",
           vpp_.vp_reached_pos_tol_, vp_reached_yaw_deg);

  nh.getParam("lidar_perception/fov_viewpoint_up", vpp_.fov_up_);
  nh.getParam("lidar_perception/lidar_pitch", vpp_.lidar_pitch_);
  nh.getParam("lidar_perception/fov_viewpoint_down", vpp_.fov_down_);
  // 수평 FOV 제한 라이다(예: 전방 120도) 지원. 파라미터가 없으면 360
  // (= 수평 검사 항상 통과 = 기존 전방위 동작 그대로).
  float fov_vp_horizontal = 360.0f;
  nh.param("lidar_perception/fov_viewpoint_horizontal", fov_vp_horizontal,
           360.0f);
  nh.param("lidar_perception/lidar_yaw", vpp_.lidar_yaw_, 0.0f);

  // [feature: cone-clip] limited-FOV LiDAR boundary model for the data-driven
  // yaw FOV-edge scan. yaw_fov is read in degrees and stored in radians.
  nh.param("lidar_perception/is_360lidar", frtp_.is_360_lidar_, true);
  float yaw_fov_deg = 360.0f;
  nh.param("lidar_perception/yaw_fov", yaw_fov_deg, 360.0f);
  frtp_.yaw_fov_ = yaw_fov_deg * static_cast<float>(M_PI) / 180.0f;

  vpp_.view_direction_range_ = cos(vpp_.view_direction_range_ * M_PI / 180.0);
  vpp_.fov_up_ = vpp_.fov_up_ * M_PI / 180.0;
  vpp_.fov_down_ = vpp_.fov_down_ * M_PI / 180.0;
  vpp_.fov_h_half_ = fov_vp_horizontal / 2.0f * M_PI / 180.0;
  ROS_INFO("[FrontierManager] viewpoint FOV: up %.1f deg, down %.1f deg, "
           "horizontal %.1f deg, lidar mount pitch %.1f / yaw %.1f deg",
           vpp_.fov_up_ * 180.0 / M_PI, vpp_.fov_down_ * 180.0 / M_PI,
           fov_vp_horizontal, vpp_.lidar_pitch_, vpp_.lidar_yaw_);
  checkPerceptionConfig(nh);
  frtp_.map_min_ =
      lidar_map_interface_->lp_->global_map_min_boundary_.cast<float>();
  frtp_.map_max_ =
      lidar_map_interface_->lp_->global_map_max_boundary_.cast<float>();
  frtp_.cell_max_cnt_ =
      ((frtp_.map_max_ - frtp_.map_min_).array() / frtp_.cell_size_)
          .cast<int>()
          .matrix() +
      Eigen::Vector3i::Ones();
  frtp_.bits_need_.x() = std::ceil(std::log2(frtp_.cell_max_cnt_.x()));
  frtp_.bits_need_.y() = std::ceil(std::log2(frtp_.cell_max_cnt_.y()));
  frtp_.bits_need_.z() = std::ceil(std::log2(frtp_.cell_max_cnt_.z()));
  frtp_.idx_byte_size_ =
      (frtp_.bits_need_.x() + frtp_.bits_need_.y() + frtp_.bits_need_.z() + 7) /
      8;

  // [feature: vp-sampling] 셀 중심(cell-center) 샘플링.
  //
  // 이전에는 하한에서 시작해 step 씩 더하며 `<= 상한 - 1e-3` 로 끊었다. 그 결과
  // 하한은 포함되고 상한은 배제되어 격자가 아래(안쪽)로 치우쳤다:
  //   height [-1.0, 1.0] x5 -> -1.0, -0.6, -0.2, 0.2, 0.6   (상한 1.0 미샘플)
  //   radius [ 1.0, 3.0] x8 ->  1.00 ... 2.75               (상한 3.0 미샘플)
  // 특히 "frontier 와 같은 높이"(pitch=0, 어떤 FOV 든 통과)인 height=0 층이
  // 아예 없었고, 반대로 수직 FOV 상 어떤 반경에서도 볼 수 없는 -1.0 층이
  // 후보의 1/5 를 차지했다.
  //
  // 셀 중심 방식은 구간을 n 등분한 각 칸의 중앙을 찍으므로 상·하한에 대칭이고
  // 홀수 층이면 정중앙(0)이 반드시 포함된다:
  //   height -> -0.8, -0.4, 0.0, 0.4, 0.8
  //   radius ->  1.125 ... 2.875
  // 인덱스 기반 루프라 float 누적 오차도 없고, 각 방향 개수가 설정값과 정확히
  // 일치한다 (기존 각도 루프는 경계 조건 때문에 n+1 개를 만들기도 했다).
  const int n_h = std::max(1, vpp_.sample_pillar_height_layer_num_);
  const int n_r = std::max(1, vpp_.sample_pillar_radius_layer_num_);
  const int n_d = std::max(1, vpp_.sample_pillar_circle_sample_num_);
  const float degree_step = 2 * M_PI / (float)n_d;
  // 층마다 방위를 조금씩 돌려 후보가 방사형으로 정렬되지 않게 한다(기존 의도 유지).
  const float start_degree_step = degree_step / (float)n_h;
  const float height_step =
      (vpp_.sample_pillar_max_height_ - vpp_.sample_pillar_min_height_) / (float)n_h;
  const float radius_step =
      (vpp_.sample_pillar_max_radius_ - vpp_.sample_pillar_min_radius_) / (float)n_r;
  float start_degree = 0;
  for (int hi = 0; hi < n_h; hi++) {
    const float height = vpp_.sample_pillar_min_height_ + (hi + 0.5f) * height_step;
    for (int ri = 0; ri < n_r; ri++) {
      const float radius = vpp_.sample_pillar_min_radius_ + (ri + 0.5f) * radius_step;
      start_degree += start_degree_step;
      for (int di = 0; di < n_d; di++) {
        const float degree = start_degree + di * degree_step;
        Eigen::Vector3f vp(radius * cos(degree), radius * sin(degree), height);
        origin_viewpoints_.push_back(vp);
      }
    }
  }
  frtd_ = FrontierData(frtp_.idx_byte_size_);
  frtd_.label_map_.max_load_factor(1.5);
  
  // Initialize timing publishers
  vp_cluster_cost_pub_ = nh.advertise<std_msgs::Float32>("/global_planning/vp_cluster_cost", 10);
  remove_unreachable_cost_pub_ = nh.advertise<std_msgs::Float32>("/global_planning/remove_unreachable_cost", 10);
  select_vp_cost_pub_ = nh.advertise<std_msgs::Float32>("/global_planning/select_vp_cost", 10);
  explored_cell_count_pub_ = nh.advertise<std_msgs::Int32>("/frontier_manager/explored_cell_count", 10);
}

// [feature: perception-config-check] lidar_perception/* 정합성 검사.
//
// 이 그룹은 파라미터가 12개인데 소비자가 셋(lidar_map / frontier_manager /
// planner_manager)으로 흩어져 있고, 물리값 계열(fov_*)과 계획용 보수값 계열
// (fov_viewpoint_*)이 짝을 이룬다. 그래서 서로 모순되는 조합을 넣어도 아무도
// 불평하지 않고 조용히 이상하게 동작한다. 실제로 프로파일들에 이런 것이 있다:
//   - mid360/mid360_mlx : is_360lidar=true 인데 yaw_fov=120  (전방위? 120도?)
//   - mid360            : fov_viewpoint_up(48) > fov_up(37.5) — 보수값이 물리값보다 넓음
//   - real              : cloud_frame_mode=world 인데 sensor_mount_* 를 설정 (무효)
// 시작 시 한 번 훑어 경고만 남긴다 (동작은 바꾸지 않는다 — 값 판단은 사람의 몫).
void FrontierManager::checkPerceptionConfig(ros::NodeHandle &nh) {
  const std::string P = "lidar_perception/";
  auto getd = [&](const std::string &k, double def) {
    double v = def;
    nh.param(P + k, v, def);
    return v;
  };
  int warns = 0;

  // 1) is_360lidar 와 yaw_fov 는 "같은 사실"이 아니라 독립된 두 스위치다.
  //      yaw_fov     -> corridor cone clip 각도 (planner_manager)
  //      is_360lidar -> 수평 FOV-edge 스캔 on/off (frontier_manager)
  //    게이트가 `!is_360lidar && yaw_fov < 360` 라서, is_360lidar=true 는
  //    yaw_fov 와 무관하게 스캔만 끈다. mid360/mid360_mlx 가 실제로 이 조합을
  //    의도적으로 쓴다("disable the horizontal FOV-edge scan"). 따라서 이 조합을
  //    모순으로 경고하면 정상 설정에 오탐이 난다 — 대신 실제 효과를 알려준다.
  bool is360 = true;
  nh.param(P + "is_360lidar", is360, true);
  const double yaw_fov = getd("yaw_fov", 360.0);
  if (is360 && yaw_fov < 359.9) {
    ROS_INFO("[perception-config] is_360lidar=true disables the horizontal "
             "FOV-edge scan; yaw_fov=%.1fdeg is still used for corridor cone "
             "clipping. (Set is_360lidar=false to enable the scan.)", yaw_fov);
  }
  if (!is360 && yaw_fov > 359.9) {
    ROS_WARN("[perception-config] is_360lidar=false but yaw_fov=360deg, so the "
             "horizontal FOV-edge scan never runs (its gate needs yaw_fov<360). "
             "Set yaw_fov to the real horizontal FOV to enable it.");
    warns++;
  }

  // 2) 계획용 보수값은 물리값 안에 들어와야 한다 (⊆ 관계)
  const double fov_up = getd("fov_up", -0.1), fov_down = getd("fov_down", -0.1);
  const double vp_up = getd("fov_viewpoint_up", -0.1),
               vp_down = getd("fov_viewpoint_down", -0.1);
  if (fov_up > -0.05 && vp_up > fov_up + 1e-3) {
    ROS_WARN("[perception-config] fov_viewpoint_up(%.1f) > fov_up(%.1f): the planning "
             "value is WIDER than the physical FOV, so viewpoints get assigned "
             "to cells the sensor cannot actually see.", vp_up, fov_up);
    warns++;
  }
  if (fov_down < -0.05 && vp_down < fov_down - 1e-3) {
    ROS_WARN("[perception-config] fov_viewpoint_down(%.1f) < fov_down(%.1f): the planning "
             "value is WIDER than the physical FOV.", vp_down, fov_down);
    warns++;
  }
  const double fov_h = getd("fov_horizontal", 360.0),
               vp_h = getd("fov_viewpoint_horizontal", 360.0);
  if (vp_h > fov_h + 1e-3) {
    ROS_WARN("[perception-config] fov_viewpoint_horizontal(%.1f) > fov_horizontal(%.1f): "
             "the planning value is WIDER than the physical FOV.",
             vp_h, fov_h);
    warns++;
  }

  // 3) 폐지된 키가 yaml 에 남아 있는지 (이제 코드가 읽지 않는다)
  if (nh.hasParam(P + "sensor_mount_pitch_deg") ||
      nh.hasParam(P + "sensor_mount_yaw_deg")) {
    ROS_WARN("[perception-config] sensor_mount_pitch_deg/sensor_mount_yaw_deg "
             "are no longer read. The mount rotation now comes solely from "
             "lidar_pitch/lidar_yaw; remove the stale keys to avoid confusion "
             "(sensor_mount_offset is still used for the translation).");
    warns++;
  }

  // 4) 장착각이 0 인데 FOV 가 수평 대칭이 아니면, 기울어진 센서를 0 으로
  //    적어둔 전형적인 실수일 수 있다 (반대로 대칭인데 0 이면 정상).
  const double pitch = getd("lidar_pitch", 0.0);
  if (fabs(pitch) < 1e-3 && fov_up > -0.05 && fov_down > -0.05) {
    ROS_WARN("[perception-config] lidar_pitch=0 but the vertical FOV [%.1f, %.1f] lies "
             "entirely above horizontal. If the sensor is physically tilted, "
             "put the real mount angle in lidar_pitch or viewpoint visibility "
             "will be wrong.",
             fov_down, fov_up);
    warns++;
  }

  ROS_INFO("[perception-config] checked lidar_perception/*: %d warning(s)", warns);
}

void FrontierManager::pos2idx(const PointType &pt, Eigen::Vector3i &idx) {
  //
  idx = ((pt.getVector3fMap() - frtp_.map_min_) * frtp_.inv_cell_size_)
            .array()
            .floor()
            .cast<int>();
}

void FrontierManager::pos2idx(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  //
  idx = ((pt - frtp_.map_min_) * frtp_.inv_cell_size_)
            .array()
            .floor()
            .cast<int>();
}

void FrontierManager::idx2bytes(const Eigen::Vector3i &idx,
                                ByteArrayRaw &bytes) {
  // 无需resize，因为ByteArrayRaw在构造时已分配好固定大小
  uint64_t value = (static_cast<uint64_t>(idx.x())
                    << (frtp_.bits_need_.y() + frtp_.bits_need_.z())) |
                   (static_cast<uint64_t>(idx.y()) << frtp_.bits_need_.z()) |
                   static_cast<uint64_t>(idx.z());

  for (int i = 0; i < frtp_.idx_byte_size_; ++i) {
    bytes.data[i] = static_cast<uint8_t>(value >> (i * 8));
  }
}

void FrontierManager::bytes2pos(const ByteArrayRaw &bytes, PointType &pt) {
  Eigen::Vector3i idx;

  uint64_t value = 0;
  for (int i = frtp_.idx_byte_size_ - 1; i >= 0; --i) {
    value = (value << 8) | static_cast<uint64_t>(bytes.data[i]);
  }

  idx.z() = static_cast<int>(value & ((1 << frtp_.bits_need_.z()) - 1));
  value >>= frtp_.bits_need_.z();

  idx.y() = static_cast<int>(value & ((1 << frtp_.bits_need_.y()) - 1));
  value >>= frtp_.bits_need_.y();

  idx.x() = static_cast<int>(value);

  Eigen::Vector3f pt_v3f =
      (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ +
      frtp_.map_min_;
  pt.x = pt_v3f.x();
  pt.y = pt_v3f.y();
  pt.z = pt_v3f.z();
}

void FrontierManager::pos2bytes(const PointType &pt, ByteArrayRaw &bytes) {
  Eigen::Vector3i idx =
      ((pt.getVector3fMap() - frtp_.map_min_) * frtp_.inv_cell_size_)
          .array()
          .floor()
          .cast<int>();
  idx2bytes(idx, bytes);
}

CELL_STATE FrontierManager::get_state(const PointType &pt) {
  Eigen::Vector3i idx;
  pos2idx(pt, idx);
  return get_state(idx);
}

CELL_STATE FrontierManager::get_state(const Eigen::Vector3i &idx) {
  ByteArrayRaw bytes;
  idx2bytes(idx, bytes);
  if (frtd_.label_map_.find(bytes) == frtd_.label_map_.end())
    return UNKNOWN;
  else
    return (CELL_STATE)frtd_.label_map_[bytes];
}

// [feature: cone-clip] world-point observation state for the local planner.
CELL_STATE FrontierManager::getCellState(const Eigen::Vector3f &p) {
  PointType pt;
  pt.x = p.x();
  pt.y = p.y();
  pt.z = p.z();
  return get_state(pt);
}

void FrontierManager::get_cells_2_update(
    const PointVector &points, vector<Eigen::Vector3i> &cells_2_update) {
  cells_2_update.clear();
  std::unordered_set<Eigen::Vector3i, Vector3i_Hash> cells_2_update_set;
  std::unordered_set<Eigen::Vector3i, Vector3i_Hash> updated;
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  for (auto &pt : points) {
    if (!lidar_map_interface_->IsInBox(pt))
      continue;
    if ((pt.getVector3fMap() -
         lidar_map_interface_->ld_->lidar_pose_.cast<float>())
            .norm() > frtp_.update_length_)
      continue;
    Eigen::Vector3i idx;
    pos2idx(pt, idx);
    if (updated.count(idx))
      continue;
    cells_2_update_set.insert(idx);
    updated.insert(idx);
    if (is_gap_point(pt) || is_fov_edge(pt)) {
      continue;
    }
    // 下面这块是为了去除噪声，也可以改成raycast
    for (int i = -frtp_.noise_cell_range_; i <= frtp_.noise_cell_range_; i++)
      for (int j = -frtp_.noise_cell_range_; j <= frtp_.noise_cell_range_; j++)
        for (int k = -frtp_.noise_cell_range_; k <= frtp_.noise_cell_range_;
             k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          Eigen::Vector3i cell = idx + Eigen::Vector3i(i, j, k);
          if (cells_2_update_set.count(cell))
            continue;
          ByteArrayRaw bytes;
          idx2bytes(cell, bytes);
          if (!frtd_.label_map_.count(bytes))
            continue;
          if (frtd_.label_map_[bytes] == DENSE)
            continue;
          cells_2_update_set.insert(cell);
        }
  }
  cells_2_update.insert(cells_2_update.end(), cells_2_update_set.begin(),
                        cells_2_update_set.end());
}

void FrontierManager::get_pts_in_cells(
    const vector<Eigen::Vector3i> &cells_2_update,
    vector<PointVector> &pts_inside) {
  ros::Time t1 = ros::Time::now();
  pts_inside.clear();
  pts_inside.resize(cells_2_update.size());
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto i = 0; i < cells_2_update.size(); i++) {
    if (get_state(cells_2_update[i]) == DENSE)
      continue;
    PointType pt;
    idx2pos(cells_2_update[i], pt);
    vector<float> t;
    lidar_map_interface_->KNN(pt, 15, pts_inside[i], t);
  }
  // ROS_INFO("get_pts_in_cells time cost: %f", (ros::Time::now() - t1).toSec()
  // * 1000.0);
}

bool FrontierManager::is_gap_point(const PointType &pt) {

  return frtd_.is_gap_[surface_pos2idx(pt)];
  // return false;
}

void FrontierManager::update_lidar_pt_gap(const vector<float> &depth) {

  frtd_.is_gap_ = vector<bool>(20000, false);
  auto viz_img = [&](cv::Mat img, string name) {
    cv::Mat image_8u, upsampled;
    cv::normalize(img, image_8u, 0, 255, cv::NORM_MINMAX);
    image_8u.convertTo(image_8u, CV_8UC1);
    cv::resize(image_8u, upsampled, cv::Size(400, 800));
    cv::imshow(name, upsampled);
    cv::waitKey(1);
  };
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  frtd_.direction_score_ = vector<float>(20000, 0.0f);
  static vector<Eigen::Vector2i> diff_lis{{-1, 0}, {0, -1}, {0, 1}, {1, 0}};
  static vector<Eigen::Vector2i> diff_lis2{{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                           {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 200; j++) {
      float dis1 = depth[i * 200 + j];
      for (auto &diff : diff_lis2) {
        if (!(i + diff[0] >= 0 && i + diff[0] < 100 && j + diff[1] >= 0 &&
              j + diff[1] < 200))
          continue;
        float dis2 = depth[(i + diff[0]) * 200 + (j + diff[1])];
        if (dis1 < 0 || dis2 < 0) {
          frtd_.is_gap_[i * 200 + j] = true;
          break;
        }
        float score = dis1 / dis2;
        if (score > 1.0)
          score = 1 / score;
        // float score = dis2 / dis1;
        frtd_.direction_score_[i * 200 + j] =
            frtd_.direction_score_[i * 200 + j] <= 1e-6
                ? score
                : min(frtd_.direction_score_[i * 200 + j], score);
        if (frtd_.direction_score_[i * 200 + j] <
            frtp_.good_observation_direction_score_) {
          frtd_.is_gap_[i * 200 + j] = true;
          break;
          // if (fabs(dis1 - dis2) > 0.5) {
          //   is_gap_[i * 200 + j] = true;
          //   break;
        }
      }
    }
  }
  vector<bool> is_gap_tmp = frtd_.is_gap_;
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 200; j++) {
      if (is_gap_tmp[i * 200 + j]) {
        for (auto &diff : diff_lis2) {
          if (!(i + diff[0] >= 0 && i + diff[0] < 100 && j + diff[1] >= 0 &&
                j + diff[1] < 200)) {
            continue;
          }
          frtd_.is_gap_[(i + diff[0]) * 200 + (j + diff[1])] = true;
        }
      }
    }
  }
  ros::Time t4 = ros::Time::now();

  cv::Mat img_gap = cv::Mat::zeros(100, 200, CV_8UC1);
  cv::Mat img_depth = cv::Mat::zeros(100, 200, CV_32FC1);
  for (int i = 0; i < 100; i++)
    for (int j = 0; j < 200; j++) {
      img_gap.at<uchar>(i, j) = frtd_.is_gap_[i * 200 + j] ? 255 : 0;
      img_depth.at<float>(i, j) = depth[i * 200 + j];
    }
}

void FrontierManager::cluster_frts(const PointVector &frt_new,
                                   vector<ClusterInfo::Ptr> &new_clusters,
                                   vector<int> &cluster_removed) {
  cluster_removed.clear();
  PointVector frts2cluster;
  // static int frt_cluser_id = 0;
  // cout << "SF_list size: " << frt_cluster_ptr_->SF_list.size() << endl;
  vector<Eigen::Vector3f> frts_norm;
  if (cluster_list_.size() != 0) {
    cluster_list_.remove_if([this, &frts2cluster, &frts_norm,
                             &cluster_removed](ClusterInfo::Ptr &cluster) {
      // if (!cluster->is_reachable_)
      //   return false;
      if (force_recluster_.count(cluster->id_) ||
          (force_recluster_.empty() &&
           has_overlap(cluster->box_max_, cluster->box_min_))) {
        for (auto &pt : cluster->cells_) {
          if (get_state(pt) == FRONTIER_DIS || get_state(pt) == FRONTIER_DIR) {
            frts2cluster.push_back(pt);
            ByteArrayRaw bytes;
            pos2bytes(pt, bytes);
            Eigen::Vector3f norm = frtd_.frt_map_[bytes];
            // debug:
            if (std::isnan(norm[0]) || std::isnan(norm[1]) ||
                std::isnan(norm[2])) {
              std::cout << "At least one element in the norm is NaN."
                        << std::endl;
              exit(1);
            }
            frts_norm.push_back(norm);
          }
        }
        cluster_removed.push_back(cluster->id_);
        return true;
      }
      return false;
    });
  }

  frts2cluster.insert(frts2cluster.end(), frt_new.begin(), frt_new.end());
  // debug:
  // int nan_count = 0;
  for (auto &pt : frt_new) {
    ByteArrayRaw bytes;
    pos2bytes(pt, bytes);
    Eigen::Vector3f norm = frtd_.frt_map_[bytes];
    if (std::isnan(norm[0]) || std::isnan(norm[1]) || std::isnan(norm[2])) {
      // nan_count++;
      // ROS_WARN("[DEBUG cluster_frts] NaN norm detected at point (%f, %f, %f), skipping", pt.x, pt.y, pt.z);
      continue;  // Skip NaN instead of exiting
    }
    frts_norm.push_back(norm);
  }

  // ROS_INFO("[DEBUG cluster_frts] frt_new=%lu, frts2cluster=%lu, frts_norm=%lu, nan_count=%d, min_required=%d",
  //          frt_new.size(), frts2cluster.size(), frts_norm.size(), nan_count, frtp_.cluster_minmum_point_num_);

  // 重新聚类:
  if (frts2cluster.size() < frtp_.cluster_minmum_point_num_) {
    // ROS_WARN("[DEBUG cluster_frts] Not enough points to cluster: %lu < %d",
    //          frts2cluster.size(), frtp_.cluster_minmum_point_num_);
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr frt_pc(
      new pcl::PointCloud<pcl::PointXYZ>);
  frt_pc->points = frts2cluster;
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(frt_pc);
  auto getNbrs = [&](int norm_idx, int idx, vector<int> &nbr_idxs) -> int {
    nbr_idxs.clear();
    std::vector<int> indices;
    std::vector<float> squared_distances;
    if (kdtree.radiusSearch(idx, frtp_.cluster_min_radius_, indices,
                            squared_distances) < 3)
      return 0;
    Eigen::Vector3f norm = frts_norm[norm_idx];
    for (int nbr_idx : indices) {
      Eigen::Vector3f nbr_norm = frts_norm[nbr_idx];
      if (norm.dot(nbr_norm) > (frtp_.cluster_direction_radius_)) {
        nbr_idxs.push_back(nbr_idx);
      }
    }
    return nbr_idxs.size();
  };
  std::vector<int> labels;
  labels.resize(frt_pc->points.size(), -1); // 初始化标签，-1 表示未访问
  int cluster_id = 0;                       // 聚类ID
  for (size_t i = 0; i < frts2cluster.size(); i++) {
    if (labels[i] != -1)
      continue;
    std::vector<int> indices;
    if (getNbrs(i, i, indices) < 1)
      continue;
    cluster_id++;           // 分配新的聚类ID
    labels[i] = cluster_id; // 标记当前点
    std::list<size_t> queue;
    queue.push_back(i);
    Eigen::Vector3f aabb_max =
        Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                        std::numeric_limits<float>::lowest(),
                        std::numeric_limits<float>::lowest());
    Eigen::Vector3f aabb_min = Eigen::Vector3f(
        std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());

    while (!queue.empty()) {
      size_t current = queue.front();
      queue.pop_front();
      // 执行基于半径的搜索找到当前点的邻居
      if (getNbrs(i, current, indices) >= 1) {
        Eigen::Vector3f norm = frts_norm[current];
        for (size_t j = 0; j < indices.size(); ++j) {
          int neighbor_index = indices[j];
          if (labels[neighbor_index] == -1) {
            // 如果邻居未被访问，则将其添加到聚类中
            labels[neighbor_index] = cluster_id;
            queue.push_back(neighbor_index);
          } else if (labels[neighbor_index] == 0) {
            // 如果邻居是噪声点，则将其重新标记为当前聚类的一部分
            labels[neighbor_index] = cluster_id;
          }
          aabb_min =
              aabb_min.cwiseMin(frts2cluster[neighbor_index].getVector3fMap());
          aabb_max =
              aabb_max.cwiseMax(frts2cluster[neighbor_index].getVector3fMap());
          if ((aabb_max - aabb_min).maxCoeff() > frtp_.cluster_radius_)
            break;
        }
      } else {
        // 如果邻域内的点数不足以形成一个聚类，则将其标记为噪声
        labels[i] = 0;
      }
    }
  }
  // ROS_INFO("[DEBUG cluster_frts] DBSCAN found %d clusters from %lu points", cluster_id, frts2cluster.size());

  for (int i = 1; i <= cluster_id; i++) {
    PointVector frt_cluster_pt;
    vector<Eigen::Vector3f> frt_cluster_norm;
    for (int j = 0; j < labels.size(); j++) {
      if (labels[j] == i) {
        frt_cluster_pt.push_back(frts2cluster[j]);
        frt_cluster_norm.push_back(frts_norm[j]);
      }
    }
    // if (frt_cluster_pt.size() < frtp_.cluster_minmum_point_num_)
    //   continue;
    ClusterInfo::Ptr cluster = make_shared<ClusterInfo>();
    compute_cluster_info(frt_cluster_pt, frt_cluster_norm, cluster);
    cluster_list_.push_back(cluster);
    new_clusters.push_back(cluster);
  }

  // ROS_INFO("[DEBUG cluster_frts] Created %lu clusters, cluster_list_ now has %lu total",
  //          new_clusters.size(), cluster_list_.size());
  // 将噪音删掉
  for (int i = 0; i < labels.size(); i++) {
    if (labels[i] == 0) {
      ByteArrayRaw bytes;
      pos2bytes(frts2cluster[i], bytes);
      frtd_.label_map_[bytes] = DENSE;
      frtd_.frt_map_.erase(bytes);
    }
  }
}

void FrontierManager::idx2pos(const Eigen::Vector3i &idx, PointType &pt) {
  Eigen::Vector3f pt_v3f =
      (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ +
      frtp_.map_min_;
  pt.x = pt_v3f.x();
  pt.y = pt_v3f.y();
  pt.z = pt_v3f.z();
}

void FrontierManager::computeNormal(const PointVector &local_pts,
                                    Eigen::Vector3f &normal) {
  if (local_pts.size() < 3) {
    normal.setZero();
    return;
  }
  Eigen::Vector3f center(0.0, 0.0, 0.0);
  for (int i = 0; i < local_pts.size(); i++) {
    center += local_pts[i].getVector3fMap();
  }
  center /= local_pts.size();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

  for (auto &pt : local_pts) {
    Eigen::Vector3f div = pt.getVector3fMap() - center;
    covariance += div * div.transpose();
  }
  covariance /= local_pts.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
  if (saes.info() != Eigen::Success) {
    normal.setZero();
    return;
  }
  normal = saes.eigenvectors().col(0);
  const double normal_norm = normal.norm();
  if (!normal.allFinite() ||
      normal_norm <= lidar_map_interface_->lp_->vector_norm_eps_) {
    normal.setZero();
    return;
  }
  normal /= normal_norm;
}

void FrontierManager::computeNormalCell(const PointVector &local_pts,
                                        Eigen::Vector3f &normal,
                                        Eigen::Vector3f &center) {
  if (local_pts.size() < 3) {
    center.setZero();
    normal.setZero();
    return;
  }
  center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < local_pts.size(); i++) {
    center += local_pts[i].getVector3fMap();
  }
  center /= local_pts.size();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

  for (auto &pt : local_pts) {
    Eigen::Vector3f div = pt.getVector3fMap() - center;
    covariance += div * div.transpose();
  }

  covariance /= local_pts.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
  if (saes.info() != Eigen::Success) {
    normal.setZero();
    return;
  }
  normal = saes.eigenvectors().col(0);
  const double normal_norm = normal.norm();
  if (!normal.allFinite() ||
      normal_norm <= lidar_map_interface_->lp_->vector_norm_eps_) {
    normal.setZero();
    return;
  }
  normal /= normal_norm;
  Eigen::Vector3f dir =
      center - lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  if (dir.dot(normal) < 0) {
    return;
  } else {
    normal = -normal;
  }

  Eigen::Vector3f move_pt_ =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>() - center;
  double dotProduct = move_pt_.dot(normal);
  double normVec1 = move_pt_.norm();
  double normVec2 = normal.norm();
  if (!move_pt_.allFinite() ||
      normVec1 <= lidar_map_interface_->lp_->vector_norm_eps_ ||
      normVec2 <= lidar_map_interface_->lp_->vector_norm_eps_)
    return;
  const double angle_den =
      std::max(normVec1 * normVec2,
               lidar_map_interface_->lp_->vector_norm_eps_);
  double cosAngle = std::max(-1.0, std::min(1.0, dotProduct / angle_den));
  double angle = std::acos(cosAngle);
  angle = angle * 180.0 / M_PI;
  if (angle > 180)
    angle = 360 - angle;
  if (angle > 90)
    normal = -normal;
}

void FrontierManager::update_lidar_fov_edge(const vector<float> &depth) {
  auto viz_img = [&](cv::Mat img, string name) {
    cv::Mat image_8u, upsampled;
    cv::normalize(img, image_8u, 0, 255, cv::NORM_MINMAX);
    image_8u.convertTo(image_8u, CV_8UC1);
    cv::resize(image_8u, upsampled, cv::Size(400, 800));
    cv::imshow(name, upsampled);
    cv::waitKey(1);
  };
  frtd_.is_fov_edge_ = vector<bool>(20000, false);

  for (int j = 0; j < 200; j++) {
    for (int i = 0; i < 100; i++) {
      if (depth[i * 200 + j] <= 0.1)
        frtd_.is_fov_edge_[i * 200 + j] = true;
      else {
        frtd_.is_fov_edge_[i * 200 + j] = true;
        break;
      }
    }
    for (int i = 99; i >= 0; i--) {
      if (depth[i * 200 + j] <= 0.1)
        frtd_.is_fov_edge_[i * 200 + j] = true;
      else {
        frtd_.is_fov_edge_[i * 200 + j] = true;
        break;
      }
    }
  }
  // [feature: cone-clip] data-driven yaw (left/right) FOV edge for a cropped
  // LiDAR. The spherical image wraps in azimuth with forward at column 0, so a
  // naive left->right scan would mis-mark the forward center. Instead sweep
  // inward from the back (column 100 == 180 deg, guaranteed empty for a forward
  // crop) toward forward on both sides and mark the first valid pixel per row.
  // This is the azimuth analog of the pitch (top/bottom) scan above and needs
  // NO angular margin: it locates the actual outermost observed cell itself.
  if (!frtp_.is_360_lidar_ &&
      frtp_.yaw_fov_ < 2.0f * static_cast<float>(M_PI) - 1.0e-3f) {
    const int back_col = 100; // azimuth 180 deg, outside a forward crop
    for (int i = 0; i < 100; i++) {
      for (int s = 0; s < 200; s++) { // sweep back -> forward (one side)
        const int j = (back_col + s) % 200;
        if (depth[i * 200 + j] <= 0.1)
          frtd_.is_fov_edge_[i * 200 + j] = true;
        else {
          frtd_.is_fov_edge_[i * 200 + j] = true;
          break;
        }
      }
      for (int s = 0; s < 200; s++) { // sweep back -> forward (other side)
        const int j = (back_col - s + 200) % 200;
        if (depth[i * 200 + j] <= 0.1)
          frtd_.is_fov_edge_[i * 200 + j] = true;
        else {
          frtd_.is_fov_edge_[i * 200 + j] = true;
          break;
        }
      }
    }
  }
  cv::Mat img_origin(100, 200, CV_8UC1);
  for (int j = 0; j < 200; j++) {
    for (int i = 0; i < 100; i++) {
      img_origin.at<uchar>(i, j) = frtd_.is_fov_edge_[i * 200 + j];
    }
  }
  // viz_img(img_origin, "is_fov_edge");
}

bool FrontierManager::is_fov_edge(const PointType &pt) {
  const int idx = surface_pos2idx(pt);
  // yaw & pitch FOV edges are both marked data-driven in update_lidar_fov_edge()
  // (self-locating, no angular margin). The added yaw scan can mark more cells,
  // so bounds-check the projected index before indexing.
  return idx >= 0 && idx < static_cast<int>(frtd_.is_fov_edge_.size()) &&
         frtd_.is_fov_edge_[idx];
}

void FrontierManager::updateFrontierClusters(
    vector<ClusterInfo::Ptr> &cluster_updated, vector<int> &cluster_removed) {

  // ROS_INFO("[DEBUG updateFrontierClusters] Function called. Current cluster_list_ size: %lu", cluster_list_.size());

  PointVector frt_new;
  fov_edge_cells_.clear(); // [feature: cone-clip] rebuilt this frame
  auto has_dense_nbr = [&](const Eigen::Vector3i &idx) -> bool {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          if (get_state(idx + Eigen::Vector3i(i, j, k)) == DENSE) {
            return true;
          }
        }
      }
    }
    return false;
  };
  auto has_sparse_nbr = [&](const Eigen::Vector3i &idx) -> bool {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          if (get_state(idx + Eigen::Vector3i(i, j, k)) == SPARSE ||
              get_state(idx + Eigen::Vector3i(i, j, k)) == FRONTIER_DIR ||
              get_state(idx + Eigen::Vector3i(i, j, k)) == FRONTIER_DIS) {
            return true;
          }
        }
      }
    }
    return false;
  };

  ros::Time t1 = ros::Time::now();
  vector<Eigen::Vector3i> cells_2_update;
  update_lidar_pos();
  // Step1: 更新视角
  ros::Time t2 = ros::Time::now();
  static vector<PointVector> pts_vec;
  static int idx = 0;
  if (pts_vec.size() < 5) {
    pts_vec.push_back(lidar_map_interface_->ld_->lidar_cloud_.points);
    idx++;
  } else {
    idx = idx % 5;
    pts_vec[idx] = lidar_map_interface_->ld_->lidar_cloud_.points;
    idx++;
  }
  vector<float> depth = vector<float>(20000, -0.1);
  project_pts_2_depth_image(lidar_map_interface_->ld_->lidar_cloud_.points, depth);
  update_lidar_fov_edge(depth); // handle 雷达保护罩/旋翼/近点之类的东西
  for (int i = 0; i < pts_vec.size(); i++) {
    if (i == idx - 1)
      continue;
    project_pts_2_depth_image(pts_vec[i], depth);
  }
  update_lidar_pt_gap(depth);
  // 把gap-point可视化出来
  get_cells_2_update(lidar_map_interface_->ld_->lidar_cloud_.points,
                     cells_2_update);
  for (auto &cell : cells_2_update) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    frtd_.frt_map_.erase(bytes);
  }
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();

  PointVector bad_observation, good_observation;
  for (auto &cell : cells_2_update) {
    PointType pt;
    idx2pos(cell, pt);
    if (is_gap_point(pt) || is_fov_edge(pt) ||
        (pt.getVector3fMap() - lidar_position).norm() >
            frtp_.good_observation_trust_length_) {
      bad_observation.push_back(pt);
    } else
      good_observation.push_back(pt);
  }
  viz_point(bad_observation, "bad_obs");
  viz_point(good_observation, "good_obs");
  // cout << "update and vizgap: " << (ros::Time::now() - t2).toSec() * 1000
  // <<"----------------------------------------"<< endl;
  unordered_set<Eigen::Vector3i, Vector3i_Hash> old_frt_cells;
  old_frt_cells.reserve(cells_2_update.size());
  for (auto &cell : cells_2_update) {
    if (get_state(cell) == FRONTIER_DIS || get_state(cell) == FRONTIER_DIR)
      old_frt_cells.insert(cell);
  }
  // cout << "get_cells_2_update: " << (ros::Time::now() - t1).toSec() * 1000 <<
  // "----------------------------------------"<< endl;

  /*
  距离合适 && 视角合理->good
  距离过长 || 视角过大->bad
  距离特别短 ->good
  */

  ros::Time t3 = ros::Time::now();

  // Step2: 分类，距离特别短的->good, 距离合适 && 视角合理->good
  // ROS_INFO("[DEBUG] cells_2_update size: %lu", cells_2_update.size());

  vector<Eigen::Vector3i> cells_2_box_search;
  cells_2_box_search.reserve(cells_2_update.size());
  unordered_set<Eigen::Vector3i, Vector3i_Hash> bad_dis_set, bad_dir_set;
  int dense_count = 0, force_trust_count = 0, good_count = 0;
  for (int i = 0; i < cells_2_update.size(); i++) {
    ByteArrayRaw bytes;
    idx2bytes(cells_2_update[i], bytes);
    if (get_state(cells_2_update[i]) == DENSE) {
      dense_count++;
      continue;
    }
    PointType pt;
    idx2pos(cells_2_update[i], pt);
    float view_distance = (pt.getVector3fMap() - lidar_position).norm();
    // [feature: cone-clip] this cell reached classification and is a FOV-edge
    // cell => it becomes a FOV-edge frontier; record its world position for the
    // local planner's corridor cone clipping (rebuilt each frame).
    if (is_fov_edge(pt))
      fov_edge_cells_.push_back(pt);
    if (view_distance < frtp_.good_observation_force_trust_length_ &&
        !is_fov_edge(pt)) {
      // if (view_distance < frtp_.good_observation_force_trust_length_) {
      frtd_.label_map_[bytes] = DENSE;
      force_trust_count++;
      continue;
    }
    bool bad_dir = is_gap_point(pt) || is_fov_edge(pt);
    bool bad_dis = view_distance > frtp_.good_observation_trust_length_;
    if (!bad_dir && !bad_dis) {
      frtd_.label_map_[bytes] = DENSE;
      good_count++;
      continue;
    }
    // [feature: box-margin] 박스 경계 마진 내 bad 셀은 frontier 승격 경로에서
    // 제외하고 SPARSE 로만 남긴다: 경계 너머는 관측 자체가 불가라 영영 해소
    // 안 되는 frontier(탐사 미종료/재방문 루프)가 되기 때문. 여기서 끊으면
    // 아래 box search·normal 계산도 건너뛰어 반복 비용이 없다. good 관측으로
    // DENSE 가 되는 경로는 그대로 열려 있다 (위 분기 + force_trust).
    if (frtp_.box_boundary_margin_ > 0 && is_near_box_boundary(pt)) {
      frtd_.label_map_[bytes] = SPARSE;
      continue;
    }
    if (bad_dis) {
      bad_dis_set.insert(cells_2_update[i]);
    } else {
      bad_dir_set.insert(cells_2_update[i]);
    }
    cells_2_box_search.push_back(cells_2_update[i]);
  }

  // ROS_INFO("[DEBUG] Filtered cells: dense=%d, force_trust=%d, good=%d, bad_dis=%lu, bad_dir=%lu, cells_2_box_search=%lu",
  //          dense_count, force_trust_count, good_count, bad_dis_set.size(), bad_dir_set.size(), cells_2_box_search.size());
  // Step3: 分类，距离过长 || 视角过大->bad ,
  // 但要计算法向量判断一下是否是噪声点，如果是噪声点也设成good
  vector<PointVector> pts_inside;
  // cout << "prepare && split dense node: " << (ros::Time::now() - t3).toSec()
  // * 1000 << endl;
  ros::Time t4_ = ros::Time::now();
  get_pts_in_cells(cells_2_box_search, pts_inside);
  // cout << "get_pts_in_cells " << (ros::Time::now() - t4_).toSec() * 1000 <<
  // endl;
  ros::Time t4 = ros::Time::now();

  // 全都设置成sparse先
  unordered_set<Eigen::Vector3i, Vector3i_Hash> old_frt_set;
  for (auto &cell : cells_2_box_search) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    if (get_state(cell) == FRONTIER_DIR || get_state(cell) == FRONTIER_DIS) {
      old_frt_set.insert(cell);
    }
    frtd_.label_map_[bytes] = SPARSE;
    frtd_.frt_map_[bytes] = Eigen::Vector3f::Zero();
  }

  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < cells_2_box_search.size(); i++) {
    auto local_pts = pts_inside[i];
    if (local_pts.size() < 3)
      continue;
    else {
      ByteArrayRaw bytes;
      idx2bytes(cells_2_box_search[i], bytes);
      Eigen::Vector3f norm;
      Eigen::Vector3f t;
      // 使用view_directon(类似ray-cast)去噪
      computeNormalCell(local_pts, norm, t);
      Eigen::Vector3i norm_nbr_1, norm_nbr_2, norm_nbr_3;
      PointType pt;
      idx2pos(cells_2_box_search[i], pt);
      // norm = (lidar_position - pt.getVector3fMap()).normalized();
      pos2idx(pt.getVector3fMap() + norm * frtp_.cell_size_, norm_nbr_1);
      pos2idx(pt.getVector3fMap() - norm * frtp_.cell_size_, norm_nbr_2);
      pos2idx(pt.getVector3fMap() - 2 * norm * frtp_.cell_size_, norm_nbr_3);
      if (get_state(norm_nbr_1) == DENSE || get_state(norm_nbr_2) == DENSE ||
          get_state(norm_nbr_3) == DENSE) {
        frtd_.label_map_[bytes] = DENSE; // 说明这个点是噪声点
        // frt_map_[cells_2_box_search[i]] = norm;

        continue;
      } else {
        frtd_.frt_map_[bytes] = norm;
      }
    }
  }
  frt_new.clear();
  frt_new.reserve(cells_2_box_search.size());
  int sparse_count = 0, dense_nbr_count = 0, old_frt_count = 0;
  for (auto &cell : cells_2_box_search) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    if (get_state(cell) == SPARSE) {
      sparse_count++;
      if (has_dense_nbr(cell)) {
        dense_nbr_count++;
        if (bad_dis_set.count(cell)) {
          frtd_.label_map_[bytes] = FRONTIER_DIS;
        } else if (bad_dir_set.count(cell)) {
          frtd_.label_map_[bytes] = FRONTIER_DIR;
        } else {
          ROS_ERROR("wtf 885");
          exit(1);
        }
        if (old_frt_set.count(cell) == 0) {
          PointType pt;
          idx2pos(cell, pt);
          frt_new.push_back(pt);
        } else {
          old_frt_count++;
        }
      }
    } else {
      frtd_.frt_map_.erase(bytes);
    }
  }

  // ROS_INFO("[DEBUG] cells_2_box_search=%lu, sparse=%d, has_dense_nbr=%d, old_frt=%d, frt_new=%lu",
  //          cells_2_box_search.size(), sparse_count, dense_nbr_count, old_frt_count, frt_new.size());

  // cout << "set normal " << (ros::Time::now() - t4).toSec() * 1000 << endl;
  ros::Time t5 = ros::Time::now();
  PointVector updated_frt_pts;
  for (auto &cell : old_frt_cells) {
    if (get_state(cell) != FRONTIER_DIR && get_state(cell) != FRONTIER_DIS) {
      // updated_frt_pts.emplace_back
      PointType pt;
      idx2pos(cell, pt);
      updated_frt_pts.emplace_back(pt);
    }
  }
  PointVector updated_pts;
  updated_pts.insert(updated_pts.end(), updated_frt_pts.begin(),
                     updated_frt_pts.end());
  updated_pts.insert(updated_pts.end(), frt_new.begin(), frt_new.end());
  update_updating_aabb(updated_pts);

  // ROS_INFO("[DEBUG updateFrontierClusters] Before cluster_frts: frt_new=%lu, cluster_list_=%lu",
  //          frt_new.size(), cluster_list_.size());

  cluster_frts(frt_new, cluster_updated, cluster_removed);

  // ROS_INFO("[DEBUG updateFrontierClusters] After cluster_frts: cluster_list_=%lu, cluster_updated=%lu, cluster_removed=%lu",
  //          cluster_list_.size(), cluster_updated.size(), cluster_removed.size());

  std_msgs::Int32 count_msg;
  count_msg.data = static_cast<int32_t>(frtd_.label_map_.size());
  explored_cell_count_pub_.publish(count_msg);
}

int FrontierManager::surface_pos2idx(const PointType &pt) {
  Eigen::Vector3f pt_lidar_frame = transform_world2lidar * pt.getVector3fMap();
  Eigen::Vector2i surface_idx;
  Sphere_PosToIndex(Eigen::Vector3f::Zero(), pt_lidar_frame,
                                      surface_idx);
  return surface_idx.x() * 200 + surface_idx.y();
}

void FrontierManager::update_lidar_pos() {
  transform_world2lidar = Eigen::Isometry3f::Identity();
  transform_world2lidar.translate(
      lidar_map_interface_->ld_->lidar_pose_.cast<float>());
  transform_world2lidar.rotate(lidar_map_interface_->ld_->lidar_q_.cast<float>());
  transform_world2lidar = transform_world2lidar.inverse();
}

void FrontierManager::project_pts_2_depth_image(PointVector &pts_vec,
                                                vector<float> &depth_img) {
  PointVector pts_lidar_frame;
  // depth_img = vector<float>(20000, -0.1);
  auto project_pt = [&](PointType &pt) {
    Eigen::Vector3f pt_lidar_frame =
        transform_world2lidar * pt.getVector3fMap();
    float dis = pt_lidar_frame.norm();
    if (dis > frtp_.update_length_)
      return;
    Eigen::Vector2i surface_idx;
    Sphere_PosToIndex(Eigen::Vector3f::Zero(), pt_lidar_frame,
                                        surface_idx);
    if (depth_img[surface_idx.x() * 200 + surface_idx.y()] < 0 ||
        depth_img[surface_idx.x() * 200 + surface_idx.y()] > dis) {
      depth_img[surface_idx.x() * 200 + surface_idx.y()] = dis;
    }
  };
  for (auto &pt : pts_vec)
    project_pt(pt);
}

void FrontierManager::update_updating_aabb(const PointVector &new_frt_pts) {
  frtd_.updating_aabb_min = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max());
  frtd_.updating_aabb_max =
      Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest());
  for (auto &p : new_frt_pts) {
    frtd_.updating_aabb_min =
        frtd_.updating_aabb_min.cwiseMin(p.getVector3fMap());
    frtd_.updating_aabb_max =
        frtd_.updating_aabb_max.cwiseMax(p.getVector3fMap());
  }
  frtd_.updating_aabb_min -= Eigen::Vector3f::Ones() * 0.1;
  frtd_.updating_aabb_max += Eigen::Vector3f::Ones() * 0.1;
}

void FrontierManager::compute_cluster_info(
    const PointVector &frt_pts, const vector<Eigen::Vector3f> &frt_norms,
    ClusterInfo::Ptr cluster) {
  static int id = 0;
  cluster->id_ = id++;
  cluster->is_dormant_ = false;
  cluster->is_reachable_ = true;
  cluster->reason_ = CR_NOT_CONSIDERED;
  cluster->best_vp_score_ = 0;
  cluster->is_new_cluster_ = true;
  cluster->center_.setZero();
  cluster->normal_.setZero();
  cluster->cells_.resize(frt_pts.size());
  cluster->norms_.resize(frt_pts.size());
  cluster->box_max_ = Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest());
  cluster->box_min_ = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max());
  for (int i = 0; i < frt_pts.size(); i++) {
    Eigen::Vector3f pt = frt_pts[i].getVector3fMap();
    Eigen::Vector3f norm = frt_norms[i];
    cluster->center_ += pt;
    cluster->normal_ += norm;
    cluster->box_max_ = cluster->box_max_.cwiseMax(pt);
    cluster->box_min_ = cluster->box_min_.cwiseMin(pt);
    cluster->cells_[i] = PointType(pt.x(), pt.y(), pt.z());
    cluster->norms_[i] = norm;
  }
  cluster->box_max_ += Eigen::Vector3f::Ones() * 0.1;
  cluster->box_min_ -= Eigen::Vector3f::Ones() * 0.1;
  if (frt_pts.empty()) {
    cluster->center_.setZero();
    cluster->normal_.setZero();
    cluster->box_max_.setZero();
    cluster->box_min_.setZero();
    cluster->is_dormant_ = true;
    cluster->is_reachable_ = false;
    cluster->reason_ = CR_TOO_SMALL;
    return;
  }
  cluster->center_ /= static_cast<float>(frt_pts.size());
  const double cluster_normal_norm = cluster->normal_.norm();
  if (!cluster->normal_.allFinite() ||
      cluster_normal_norm <= lidar_map_interface_->lp_->vector_norm_eps_) {
    cluster->normal_.setZero();
  } else {
    cluster->normal_ /= cluster_normal_norm;
  }
  if ((cluster->box_max_ - cluster->box_min_).maxCoeff() <
      frtp_.cluster_min_size_) {
    cluster->is_dormant_ = true;
    cluster->reason_ = CR_TOO_SMALL;
  }
  if (cluster->cells_.size() < frtp_.cluster_min_size_) {
    cluster->is_dormant_ = true;
    cluster->reason_ = CR_TOO_FEW_CELLS;
  }
}

bool FrontierManager::has_overlap(const Eigen::Vector3f &box_max_,
                                  const Eigen::Vector3f &box_min_) {
  if (frtd_.updating_aabb_max.x() < box_min_.x() ||
      frtd_.updating_aabb_max.y() < box_min_.y() ||
      frtd_.updating_aabb_max.z() < box_min_.z() ||
      frtd_.updating_aabb_min.x() > box_max_.x() ||
      frtd_.updating_aabb_min.y() > box_max_.y() ||
      frtd_.updating_aabb_min.z() > box_max_.z()) {
    return false;
  }
  return true;
}

void FrontierManager::updateHalfSpaces(vector<ClusterInfo::Ptr> &clusters) {
  auto getNbrs = [&](Eigen::Vector3i &idx, vector<Eigen::Vector3i> &nbrs) {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          nbrs.emplace_back(idx[0] + i, idx[1] + j, idx[2] + k);
        }
      }
    }
  };
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &cluster : clusters) {
    unordered_set<Eigen::Vector3i, Vector3i_Hash> dense, sparse;
    for (auto &cell : cluster->cells_) {
      Eigen::Vector3i idx;
      pos2idx(cell, idx);
      vector<Eigen::Vector3i> nbrs;
      getNbrs(idx, nbrs);
      for (auto &nbr : nbrs) {
        if (get_state(nbr) == DENSE) {
          dense.insert(nbr);
        } else if (get_state(nbr) == UNKNOWN) {
          continue;
        } else {
          sparse.insert(nbr);
        }
      }
    }
    Eigen::Vector3f sparse_center = Eigen::Vector3f::Zero();
    for (auto &cell : sparse) {
      PointType pt;
      idx2pos(cell, pt);
      sparse_center += pt.getVector3fMap();
    }
    if (sparse.empty() || dense.empty())
      continue;
    sparse_center /= static_cast<float>(sparse.size());
    Eigen::Vector3f dense_center = Eigen::Vector3f::Zero();
    for (auto &cell : dense) {
      PointType pt;
      idx2pos(cell, pt);
      dense_center += pt.getVector3fMap();
    }
    dense_center /= static_cast<float>(dense.size());
    Eigen::Vector3f dir = sparse_center - dense_center;
    dir.z() = 0;
    const double dir_norm = dir.norm();
    if (!dir.allFinite() ||
        dir_norm <= lidar_map_interface_->lp_->vector_norm_eps_)
      continue;
    dir /= dir_norm;
    cluster->view_halfspace_ =
        Eigen::Vector4f(dir.x(), dir.y(), dir.z(), -cluster->center_.dot(dir));
  }
}

inline bool FrontierManager::isInBox(const PointType &pt) {
  return lidar_map_interface_->IsInBox(pt);
}

inline bool FrontierManager::isInBox(const Eigen::Vector3f &pt) {
  return lidar_map_interface_->IsInBox(pt);
}

// [feature: box-margin] 6면 프로브: pt 에서 각 축 ±margin*cell_size 지점 중
// 하나라도 박스 밖이면 경계 마진 안으로 판정. box 가 여러 개(union)여도 동작.
bool FrontierManager::is_near_box_boundary(const PointType &pt) {
  const float m = frtp_.box_boundary_margin_ * frtp_.cell_size_;
  const Eigen::Vector3f c = pt.getVector3fMap();
  for (int axis = 0; axis < 3; axis++) {
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      Eigen::Vector3f probe = c;
      probe(axis) += sgn * m;
      if (!lidar_map_interface_->IsInBox(probe))
        return true;
    }
  }
  return false;
}

void FrontierManager::selectBestViewpoint(ClusterInfo::Ptr &cluster) {
  if (cluster->vp_clusters_.empty()) {
    return;
  }
  PointVector vps;
  // [feature: vp-viz] vps 와 같은 순서로 후보 인덱스를 펼쳐 둔다 (판정 역추적용).
  vector<int> vps2cand;
  for (auto &vp_cluster : cluster->vp_clusters_) {
    vps.insert(vps.end(), vp_cluster.vps_.begin(), vp_cluster.vps_.end());
    vps2cand.insert(vps2cand.end(), vp_cluster.vp_cand_idx_.begin(),
                    vp_cluster.vp_cand_idx_.end());
  }
  vector<float> score(vps.size(), 0);
  vector<float> yaw(vps.size(), 0);
  vector<PointVector> occ_free_frts; // raycast成功，但没有考虑视角
  occ_free_frts.resize(vps.size(), PointVector());
  RayCaster ray_caster;
  ray_caster.setParams(double(frtp_.cell_size_), frtp_.map_min_.cast<double>());
  for (int i = 0; i < vps.size(); i++) {
    Eigen::Vector3f vp = vps[i].getVector3fMap();
    for (int j = 0; j < cluster->cells_.size(); j++) {
      Eigen::Vector3f frt = cluster->cells_[j].getVector3fMap();
      Eigen::Vector3f dir = (frt - vp).cast<float>();
      float distance = dir.norm();
      if (!dir.allFinite() || !std::isfinite(distance) ||
          distance <= lidar_map_interface_->lp_->vector_norm_eps_)
        continue;
      dir /= distance;
      if (distance > frtp_.good_observation_trust_length_)
        continue;
      CELL_STATE state = get_state(cluster->cells_[j]);
      // [feature: split-trust-length] 여기서 재는 distance 는 "후보 뷰포인트 -> 셀"
      // 이지 "라이다 -> 셀"이 아니다. 매핑용 force_trust_length 와 분리된 값을 쓴다.
      if (state == FRONTIER_DIR && distance > frtp_.viewpoint_dir_trust_length_)
        continue;
      Eigen::Vector3f norm = cluster->norms_[j];
      const double norm_length = norm.norm();
      if (!norm.allFinite() ||
          norm_length <= lidar_map_interface_->lp_->vector_norm_eps_)
        continue;
      norm /= norm_length;
      float sin_theta =
          std::max(-1.0f, std::min(1.0f, dir.dot(norm)));
      float cos_thera =
          std::sqrt(std::max(0.0f, 1.0f - sin_theta * sin_theta));
      float delta = M_PI / 100.0;
      const double raw_score_den = sin_theta + delta * cos_thera;
      const double score_den = std::copysign(
          std::max(std::abs(raw_score_den),
                   lidar_map_interface_->lp_->trig_gradient_eps_),
          raw_score_den);
      float score = sin_theta / score_den;
      if (score < frtp_.good_observation_direction_score_)
        continue;
      if (!ray_caster.input(frt.cast<double>(), vp.cast<double>()))
        continue;
      bool visib = true;
      Eigen::Vector3i idx;
      while (ray_caster.nextId(idx)) {
        // 必须在box里
        CELL_STATE state = get_state(idx);
        PointType pt;
        idx2pos(idx, pt);
        if (!lidar_map_interface_->IsInBox(pt) || state == DENSE ||
            state == SPARSE) {
          visib = false;
          break;
        }
      }
      if (visib) {
        occ_free_frts[i].push_back(cluster->cells_[j]);
      }
    }
  }
  for (int i = 0; i < vps.size(); i++) {
    // [feature: split-trust-length] 문턱을 파라미터화 (원래 3 하드코딩).
    if ((int)occ_free_frts[i].size() < frtp_.viewpoint_min_visible_cells_) {
      continue;
    }
    Eigen::Vector3f vp = vps[i].getVector3fMap();
    vector<int> yaw_score = vector<int>(8, 0);
    for (int j = -4; j < 4; j++) {
      float yaw = (45.0 * j + 22.5) * M_PI / 180.0;
      if (yaw > M_PI)
        yaw -= 2 * M_PI;
      if (yaw < -M_PI)
        yaw += 2 * M_PI;
      // world -> sensor: R = RotZ(yaw) * RotZ(lidar_yaw) * RotY(lidar_pitch)
      // 의 역변환. lidar_yaw/pitch 는 장착 회전(odom 프레임 대비, deg).
      Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
      transform.rotate(Eigen::AngleAxisf(-vpp_.lidar_pitch_ * M_PI / 180.0,
                                         Eigen::Vector3f::UnitY()));
      transform.rotate(Eigen::AngleAxisf(
          -(yaw + vpp_.lidar_yaw_ * M_PI / 180.0), Eigen::Vector3f::UnitZ()));
      for (auto &pt : occ_free_frts[i]) {
        Eigen::Vector3f pt2see = transform * (pt.getVector3fMap() - vp);
        // 수평 FOV 검사: 센서 x축(전방) 기준 방위각이 반각을 넘으면 이 yaw
        // 에선 안 보임. fov_h_half_=180deg(전방위)면 항상 통과 = 기존 동작.
        float azimuth = atan2(pt2see.y(), pt2see.x());
        if (fabs(azimuth) > vpp_.fov_h_half_)
          continue;
        float pitch = atan2(pt2see.z(), sqrt(pt2see.x() * pt2see.x() +
                                             pt2see.y() * pt2see.y()));
        if (pitch > vpp_.fov_up_ || pitch < vpp_.fov_down_)
          continue;
        yaw_score[j + 4]++;
      }
    }
    int max_yaw_idx = distance(yaw_score.begin(),
                               max_element(yaw_score.begin(), yaw_score.end()));
    // if (yaw_score[max_yaw_idx] == 0)
    //   continue;
    score[i] = yaw_score[max_yaw_idx];
    // Eigen::Vector4f hs = cluster->view_halfspace_;
    // Eigen::Vector4f vp_h(vp.x(), vp.y(), vp.z(), 1.0);
    // if (vp_h.dot(hs) < -0.1) {
    //   score[i] *= (1.5 - vp_h.dot(hs));
    // }
    yaw[i] = (45.0 * (max_yaw_idx - 4) + 22.5) / 180.0 * M_PI;
  }
  // [feature: vp-viz] 점수가 매겨진 후보를 사유와 함께 되짚어 기록한다.
  // score[i] > 0 이면 그 후보 자체가 "보이는" 유효 뷰포인트이고, yaw[i] 가 그
  // 후보의 최적 yaw 다 (best 하나만이 아니라 후보별로 각자 최적 yaw 를 가진다).
  for (int i = 0; i < (int)vps.size() && i < (int)vps2cand.size(); i++) {
    auto &cand = cluster->vp_candidates_[vps2cand[i]];
    cand.score_ = (int)score[i];
    if (score[i] > 0) {
      cand.status_ = VP_VALID;
      cand.yaw_ = yaw[i];
    } // else: VP_INVALID_NO_VIS 유지
  }

  int best_vp_idx = std::distance(score.begin(),
                                  std::max_element(score.begin(), score.end()));
  if (score[best_vp_idx] == 0) {
    cluster->is_reachable_ = false;
    cluster->reason_ = CR_NO_VISIBILITY;
    cluster->best_vp_score_ = 0;
    cluster->vp_clusters_.clear();
  } else {
    cluster->is_reachable_ = true;
    cluster->best_vp_yaw_ = yaw[best_vp_idx];
    cluster->best_vp_ = vps[best_vp_idx].getVector3fMap();
    cluster->best_vp_score_ = (int)score[best_vp_idx];
    cluster->reason_ = CR_OK;
    // [feature: vp-reached-clear] 뷰포인트에 도달했는데도 프론티어가 안 없어지는
    // 클러스터를 강제 해소한다. 원 저자 주석: "飞到但看不到，说明odom漂了".
    // 원래 임계값은 1e-2 (위치 1cm / yaw 0.57deg) 라 실질적으로 발동한 적이 없다 —
    // best_vp_ 는 origin_viewpoints_ 의 이산 샘플이고 best_vp_yaw_ 는 45deg 격자라
    // 실제 자세와 그 정도로 일치할 일이 없다. 이제 실도달 허용치를 쓰되, 지나가다
    // 우연히 겹친 클러스터를 지우지 않도록 hold_time 만큼 연속 유지를 요구한다.
    if (vpp_.vp_reached_clear_enable_) {
      const float dpos =
          (cluster->best_vp_ - graph_->odom_node_->center_).norm();
      float dyaw = cluster->best_vp_yaw_ - graph_->odom_node_->yaw_;
      // 원래 코드에 없던 wrap. +179deg 와 -179deg 는 2deg 차이지 358deg 가 아니다.
      while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      if (dpos < vpp_.vp_reached_pos_tol_ &&
          fabs(dyaw) < vpp_.vp_reached_yaw_tol_) {
        cluster->is_reachable_ = false;
        cluster->is_dormant_ = true;
        cluster->reason_ = CR_ODOM_DRIFT;
        cluster->vp_clusters_.clear();
      }
    }
    int tmp_idx = best_vp_idx;
    for (auto &vpc : cluster->vp_clusters_) {
      if (tmp_idx < vpc.vps_.size()) {
        cluster->distance_ = vpc.distance_;
        break;
      } else {
        tmp_idx -= vpc.vps_.size();
      }
    }
  }
}

void FrontierManager::initClusterViewpoints(ClusterInfo::Ptr &cluster) {
  cluster->vp_clusters_.clear();
  // [feature: vp-viz] 샘플 후보를 버리는 대신 사유와 함께 전부 기록한다.
  // vps_init2cand[i] = vps_init[i] 에 대응하는 vp_candidates_ 인덱스.
  cluster->vp_candidates_.clear();
  cluster->vp_candidates_.reserve(origin_viewpoints_.size());
  vector<int> vps_init2cand;
  PointVector vps_init;
  vps_init.reserve(origin_viewpoints_.size());
  vps_init2cand.reserve(origin_viewpoints_.size());
  for (auto &ovp : origin_viewpoints_) {
    Eigen::Vector3f vp = ovp + cluster->center_;
    VpCandidate cand;
    cand.pos_ = vp;
    if (lidar_map_interface_->getDisToOcc(vp) < vpp_.min_obstacle_clearance_) {
      cand.status_ = VP_INVALID_CLEARANCE;
      cluster->vp_candidates_.push_back(cand);
      continue;
    }
    if (!isInBox(vp)) {
      cand.status_ = VP_INVALID_OUT_OF_BOX;
      cluster->vp_candidates_.push_back(cand);
      continue;
    }
    Eigen::Vector3i idx;
    graph_->getIndex(vp, idx);
    if (graph_->getRegionNode(idx) == nullptr) {
      cand.status_ = VP_INVALID_NO_REGION;
      cluster->vp_candidates_.push_back(cand);
      continue;
    }
    // 기하 필터 통과. DB-SCAN 이 군집에 넣어주지 못하면 ISOLATED 로 남는다.
    cand.status_ = VP_INVALID_ISOLATED;
    cluster->vp_candidates_.push_back(cand);
    vps_init2cand.push_back((int)cluster->vp_candidates_.size() - 1);
    vps_init.emplace_back(vp.x(), vp.y(), vp.z());
  }
  if (vps_init.empty()) {
    cluster->is_reachable_ = false;
    cluster->reason_ = CR_NO_CANDIDATE;
    return;
  }
  pcl::PointCloud<PointType>::Ptr vp_cloud(new pcl::PointCloud<PointType>);
  vp_cloud->points = vps_init;
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(vp_cloud);
  vector<float> radius_vec;
  radius_vec.resize(vps_init.size(), 0.0);
  for (int i = 0; i < vps_init.size(); i++) {
    radius_vec[i] = lidar_map_interface_->getDisToOcc(vps_init[i]);
  }
  // DB-SCAN 基于连通性将初始viewpoint聚成几类
  std::vector<int> labels;
  labels.resize(vps_init.size(), -1); // 初始化标签，-1 表示未访问
  auto getNbrs = [&](int idx, vector<int> &nbr_idx) -> int {
    vector<float> sqr_distances;
    PointType p = vps_init[idx];
    vector<int> nbrs_tmp;
    kdtree.radiusSearch(p, radius_vec[idx], nbrs_tmp, sqr_distances);
    nbr_idx.clear();
    for (int i = 0; i < nbrs_tmp.size(); i++) {
      if (labels[nbrs_tmp[i]] == -1)
        nbr_idx.push_back(nbrs_tmp[i]);
    }
    return nbr_idx.size();
  };

  int cluster_id = 0; // 聚类ID
  for (int i = 0; i < vps_init.size(); i++) {
    if (labels[i] != -1)
      continue;
    vector<int> nbr_idx;
    if (getNbrs(i, nbr_idx) == 0)
      continue;
    cluster_id++;
    labels[i] = cluster_id;
    std::list<size_t> queue;
    queue.push_back(i);
    while (!queue.empty()) {
      size_t current = queue.front();
      queue.pop_front();
      if (getNbrs(current, nbr_idx) == 0)
        continue;
      for (int j = 0; j < nbr_idx.size(); j++) {
        auto nbr = nbr_idx[j];
        if (labels[nbr] == -1) {
          labels[nbr] = cluster_id;
          queue.push_back(nbr);
        }
      }
    }
  }
  for (int i = 0; i < cluster_id; i++) {
    ViewpointCluster vp_cluster;
    vp_cluster.vps_.clear();
    vector<float> cls_radius_vec;
    vector<int> cls_idx_vec;
    for (int j = 0; j < labels.size(); j++) {
      if (labels[j] != i + 1)
        continue;
      vp_cluster.vps_.push_back(vps_init[j]);
      // 군집에 편입 -> 이제 도달성 판정 대기 상태로 승격 (ISOLATED 해제)
      vp_cluster.vp_cand_idx_.push_back(vps_init2cand[j]);
      cluster->vp_candidates_[vps_init2cand[j]].status_ = VP_INVALID_UNREACHABLE;
      cls_radius_vec.push_back(radius_vec[j]);
    }
    for (int j = 0; j < cls_radius_vec.size(); j++) {
      cls_idx_vec.push_back(j);
    }
    sort(cls_idx_vec.begin(), cls_idx_vec.end(),
         [&](int a, int b) { return cls_radius_vec[a] > cls_radius_vec[b]; });
    vp_cluster.center_ = vp_cluster.vps_[cls_idx_vec[0]].getVector3fMap();
    for (int j = 0; j < cls_radius_vec.size(); j++) {
      Eigen::Vector3f pt(vp_cluster.vps_[cls_idx_vec[j]].x,
                         vp_cluster.vps_[cls_idx_vec[j]].y,
                         vp_cluster.vps_[cls_idx_vec[j]].z);
      Eigen::Vector3i idx;
      graph_->getIndex(pt, idx);
      auto region = graph_->getRegionNode(idx);
      if (region && !region->topo_nodes_.empty()) {
        vp_cluster.center_ = vp_cluster.vps_[cls_idx_vec[j]].getVector3fMap();
        break;
      }
    }
    cluster->vp_clusters_.push_back(vp_cluster);
  }
  std::sort(cluster->vp_clusters_.begin(), cluster->vp_clusters_.end(),
            [](const ViewpointCluster &a, const ViewpointCluster &b) {
              return a.vps_.size() > b.vps_.size();
            });
  // cluster->vp_clusters_.resize(min(16, int(cluster->vp_clusters_.size())));
}

void FrontierManager::removeUnreachableViewpoints(
    vector<ClusterInfo::Ptr> &clusters) {
  if (graph_->odom_node_->neighbors_.empty())
    return;
  // 建立一张映射表，可以通过topo-node映射到要删除的vp_cluster
  vector<int> nodeidx2clusteridx;
  vector<int> nodeidx2vpclusteridx;
  vector<TopoNode::Ptr> nodes2insert;
  for (int i = 0; i < clusters.size(); i++) {
    for (int j = 0; j < clusters[i]->vp_clusters_.size(); j++) {
      nodeidx2clusteridx.push_back(i);
      nodeidx2vpclusteridx.push_back(j);
      TopoNode::Ptr vp_node = make_shared<TopoNode>();
      vp_node->center_ = clusters[i]->vp_clusters_[j].center_;
      nodes2insert.push_back(vp_node);
    }
  }

  ros::Time t1 = ros::Time::now();
  graph_->insertNodes(nodes2insert, true); // only_raycast=true 可以显著加速
  ros::Time t2 = ros::Time::now();
  vector<bool> vp_cluster_kept;
  vp_cluster_kept.resize(nodes2insert.size(), true);
  // 可以并行
  for (int i = 0; i < nodes2insert.size(); i++) {
    if (nodes2insert[i]->neighbors_.empty()) {
      vp_cluster_kept[i] = false;
      vp_stats_.reach_noedge++;
      continue;
    }
    vector<TopoNode::Ptr> topo_path;
    auto closest_node = graph_->odom_node_;
    float closest_dis =
        (closest_node->center_ - nodes2insert[i]->center_).squaredNorm();
    for (auto &hodom : graph_->history_odom_nodes_) {
      if ((hodom->center_ - nodes2insert[i]->center_).squaredNorm() <
          closest_dis) {
        closest_dis = (hodom->center_ - nodes2insert[i]->center_).squaredNorm();
        closest_node = hodom;
      }
    }
    // [feature: topo-timeout] graphSearch 는 시간 초과와 경로 없음을 똑같이 false 로
    // 돌려준다. result_code 를 받아 둘을 갈라 기록해야 "도달 불가"가 기하 때문인지
    // 예산 때문인지 사후에 판정할 수 있다 (rviz 에서는 둘 다 주황으로만 보인다).
    int search_result = ParallelBubbleAstar::NO_PATH;
    if (!graph_->graphSearch(closest_node, nodes2insert[i], topo_path,
                             vpp_.reachability_search_timeout_, false, {},
                             &search_result)) {
      vp_cluster_kept[i] = false;
      if (search_result == ParallelBubbleAstar::TIME_OUT)
        vp_stats_.reach_timeout++;
      else
        vp_stats_.reach_nopath++;
    } else {
      vp_stats_.reach_ok++;
      clusters[nodeidx2clusteridx[i]]
          ->vp_clusters_[nodeidx2vpclusteridx[i]]
          .distance_ = graph_->getPathLength(topo_path);
    }
  }
  graph_->removeNodes(nodes2insert);
  vector<unordered_set<int>> kept_vp_cluster;
  kept_vp_cluster.resize(clusters.size(), unordered_set<int>());
  for (int i = 0; i < vp_cluster_kept.size(); i++) {
    if (!vp_cluster_kept[i])
      continue;
    kept_vp_cluster[nodeidx2clusteridx[i]].insert(nodeidx2vpclusteridx[i]);
  }
  for (int i = 0; i < clusters.size(); i++) {
    vector<ViewpointCluster> tmp;
    tmp.swap(clusters[i]->vp_clusters_);
    for (int j = 0; j < tmp.size(); j++) {
      // 탈락한 vp_cluster 의 후보는 VP_INVALID_UNREACHABLE 인 채로 남는다.
      if (kept_vp_cluster[i].find(j) == kept_vp_cluster[i].end())
        continue;
      // 도달은 확인됨. 아래 거리 상위 컷에서 살아남지 못하면 PRUNED 로 끝난다.
      for (int ci : tmp[j].vp_cand_idx_)
        clusters[i]->vp_candidates_[ci].status_ = VP_INVALID_PRUNED;
      clusters[i]->vp_clusters_.push_back(tmp[j]);
    }
    if (clusters[i]->vp_clusters_.empty()) {
      clusters[i]->is_reachable_ = false;
      clusters[i]->reason_ = CR_TOPO_UNREACHABLE;
    } else {
      clusters[i]->is_reachable_ = true;
      sort(clusters[i]->vp_clusters_.begin(), clusters[i]->vp_clusters_.end(),
           [](const ViewpointCluster &a, const ViewpointCluster &b) {
             return a.distance_ < b.distance_;
           });
      float min_distance = clusters[i]->vp_clusters_[0].distance_;
      vector<ViewpointCluster> tmp2;
      for (int j = 0; j < min(8, int(clusters[i]->vp_clusters_.size())); j++) {
        if (clusters[i]->vp_clusters_[j].distance_ <= min_distance * 1.35 ||
            clusters[i]->vp_clusters_[j].distance_ <=
                min_distance + vpp_.sample_pillar_max_radius_)
          tmp2.push_back(clusters[i]->vp_clusters_[j]);
      }
      clusters[i]->vp_clusters_.swap(tmp2);
      // 거리 컷까지 통과 -> selectBestViewpoint 의 가시성 판정 대기 상태로.
      for (auto &vpc : clusters[i]->vp_clusters_)
        for (int ci : vpc.vp_cand_idx_)
          clusters[i]->vp_candidates_[ci].status_ = VP_INVALID_NO_VIS;
    }
  }
}

void FrontierManager::printMemoryCost() {
  int label_map_size = frtd_.label_map_.size();
  int frt_map_size = frtd_.frt_map_.size();
  cout << "label_map_size: "
       << "(" << to_string(frtp_.idx_byte_size_) << " + 2) * " << label_map_size
       << " = " << (float(label_map_size * (frtp_.idx_byte_size_ + 2)) / 1024.0)
       << "KB" << endl;
  static ros::Publisher mem_pub =
      nh_.advertise<std_msgs::Float32>("/mem_cost", 1);
  static ros::Publisher mem_pub_2 =
      nh_.advertise<std_msgs::Float32>("/mem_cost_2", 1);
  static ros::Publisher mem_pub_3 =
      nh_.advertise<std_msgs::Float32>("/mem_cost_3", 1);
  std_msgs::Float32 msg, msg_2, msg_3;
  msg.data = (float(label_map_size * (frtp_.idx_byte_size_ + 2)) / 1024.0);
  mem_pub.publish(msg);
  // 4+2 -> 4+1+8
  msg_2.data =
      (float(label_map_size * (frtp_.idx_byte_size_ + 1 + 8)) / 1024.0);
  //
  msg_3.data = (float(label_map_size * (frtp_.idx_byte_size_ + 1 + 8) +
                      frtd_.label_map_.bucket_count() * sizeof(void *)) /
                1024.0);

  cout << "frt_map_size2 = " << msg_2.data << endl;
  cout << "frt_map_size3 = " << msg_3.data << endl;
  mem_pub_2.publish(msg_2);
  mem_pub_3.publish(msg_3);
}
inline void
FrontierManager::Sphere_PosToIndex(const Eigen::Vector3f &lidar_center,
                                   const Eigen::Vector3f &pos,
                                   Eigen::Vector2i &id) {
  // double dis = sqrt(pow((pos(0)-lidar_center(0)),2) +
  // pow((pos(1)-lidar_center(1)),2) + pow((pos(2)-lidar_center(2)),2));
  double dis = (pos - lidar_center).norm(); // pos是被按回去的，卡了最大范围。
  if (!std::isfinite(dis) ||
      dis <= lidar_map_interface_->lp_->vector_norm_eps_) {
    id.setZero();
    return;
  }
  double phi_x = atan2((pos(1) - lidar_center(1)),
                       (pos(0) - lidar_center(0))); // 水平面，以x为极轴的转角
  if (phi_x < 0)
    phi_x = 2 * M_PI + phi_x;                              // 范围是0-2pi
  const double cos_theta =
      std::max(-1.0, std::min(1.0, static_cast<double>(
                                      (pos(2) - lidar_center(2)) / dis)));
  double theta_z = std::acos(cos_theta); // 以z为极轴的转角
  if (theta_z < 0)
    theta_z = 2 * M_PI + theta_z;
  double sphere_r = 1 / M_PI;
  Eigen::Vector2d Arc_l;
  Arc_l(0) = sphere_r * theta_z;
  Arc_l(1) = sphere_r * phi_x;
  for (int i = 0; i < 2; ++i) {
    id(i) = floor(Arc_l(i) * 100);
  }
}
