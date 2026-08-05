/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-12 12:22:06
 * @LastEditTime: 2024-08-05 20:08:38
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#pragma once
#include "lidar_map/ikd_Tree.h"
#include <bits/stdc++.h>
#include <frontier_manager/raycast.h>
#include <lidar_map/lidar_map.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_topo/graph.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
using namespace fast_planner;

enum CELL_STATE { DENSE, SPARSE, UNKNOWN, FRONTIER_DIS, FRONTIER_DIR };

struct Vector3i_Hash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct FrontierParam {
  float cluster_radius_ = 0.0f;
  float cluster_min_radius_ = 0.0f;
  float cluster_direction_radius_ = 0.0f;
  int cluster_minmum_point_num_ = 0;
  float cell_size_ = 0.0f;
  float inv_cell_size_ = 0.0f;
  // [feature: cone-clip] limited-FOV LiDAR boundary model. is_360_lidar_ false
  // => the data-driven yaw FOV-edge scan runs; yaw_fov_ stored in radians.
  bool is_360_lidar_ = true;
  float yaw_fov_ = 2.0f * M_PI;
  float occ_min_dis_ = 0.0f;
  float good_observation_direction_score_ = 0.0f;
  float good_observation_trust_length_ = 0.0f;
  float update_length_ = 0.0f;
  float good_observation_force_trust_length_ = 0.0f;
  // [feature: split-trust-length] 뷰포인트 평가에서 FRONTIER_DIR 셀(gap/FOV-edge
  // 로 표시된 셀)을 "보인다"고 인정할 최대 거리 [m]. 기준점은 후보 뷰포인트다.
  //
  // 원래는 good_observation_force_trust_length_ 하나를 두 곳에서 같이 썼다:
  //   (a) 매핑: 라이다에서 이 거리 안이면 DENSE 로 확정 (관측 완료)
  //   (b) 뷰포인트 평가: 후보에서 이 거리 밖의 FRONTIER_DIR 셀은 안 보이는 것으로 처리
  // 기준점(라이다 vs 후보)도, 의미도 다른데 값이 묶여 있어서 (a)를 보수적으로
  // 낮추면 (b)가 조용히 망가졌다 — real.yaml 의 0.5 가 sample_pillar_min_radius(1.0)
  // 보다 작아 FRONTIER_DIR 셀이 어떤 후보에서도 카운트되지 못한 사례.
  // 이제 (b)는 이 값을 쓴다. yaml 에 키가 없으면 기존 값으로 폴백해 동작이 바뀌지 않는다.
  float viewpoint_dir_trust_length_ = -1.0f;
  // [feature: split-trust-length] 한 뷰포인트가 "유효"로 인정받기 위해 raycast 로
  // 실제로 보여야 하는 최소 프론티어 셀 수. 원래 코드에 3 으로 박혀 있었다.
  // 이 문턱을 못 넘으면 yaw 스캔조차 하지 않고 score=0 -> NO_VIS 로 탈락한다.
  int viewpoint_min_visible_cells_ = 3;
  int dense_cell_cloud_num_ = 0;
  int sparse_cell_cloud_num_ = 0;
  int noise_cell_range_ = 0;
  // [feature: box-margin] 탐사 박스 경계로부터 이 셀 수 이내의 bad 관측 셀은
  // frontier 로 승격하지 않는다 (경계 너머 관측 불가 → 해소 안 되는 frontier 방지).
  // 기본 0 = 비활성. yaml 의 FrontierManager/box_boundary_margin 으로만 켠다.
  int box_boundary_margin_ = 0;
  // [feature: obs-breakdown] bad_obs 를 far/fov_edge/gap 세 토픽으로도 발행할지.
  // 진단 전용이며 판정 로직에는 영향이 없다. bad_obs 와 같은 양의 점군이 한 벌
  // 더 나가므로(약 2배) 대역폭·bag 크기가 늘어난다. 원인 규명이 끝나면 끌 것.
  bool viz_obs_breakdown_ = true;
  float cluster_min_size_ = 0.0f;
  Eigen::Vector3f map_min_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f map_max_ = Eigen::Vector3f::Zero();
  Eigen::Vector3i cell_max_cnt_ = Eigen::Vector3i::Zero();
  Eigen::Vector3i bits_need_ = Eigen::Vector3i::Zero();
  uint8_t idx_byte_size_ = 0;
  bool view_cluster_ = false, view_frt_ = false;
};

struct ViewpointParam {
  int sample_pillar_circle_sample_num_ = 0, sample_pillar_radius_layer_num_ = 0,
      sample_pillar_height_layer_num_ = 0;
  float sample_pillar_min_height_ = 0.0f, sample_pillar_max_height_ = 0.0f,
      sample_pillar_min_radius_ = 0.0f, sample_pillar_max_radius_ = 0.0f;
  float fov_up_ = 0.0f, fov_down_ = 0.0f, lidar_pitch_ = 0.0f;
  // 수평 FOV 절반각 [rad] (fov_viewpoint_horizontal/2, 미설정 시 180deg=전방위)
  // + 센서 장착 yaw [deg] (lidar_pitch_ 와 같은 의미: odom 프레임 대비 장착 회전)
  float fov_h_half_ = 0.0f, lidar_yaw_ = 0.0f;
  // 뷰포인트가 장애물(벽)에서 최소 떨어져야 하는 거리[m].
  // 이보다 가까운 후보 뷰포인트는 탈락 -> 폭이 2*이 값보다 좁은 복도는
  // 유효 뷰포인트가 없어 "도달 불가"로 낙인되어 탐사 제외됨.
  // 낮추면 좁은 복도까지 탐사(단 벽에 근접 비행), 높이면 보수적.
  float min_obstacle_clearance_ = 0.0f;
  // ViewpointManager/local_tsp_size: 실비행 config(real.yaml/real1.yaml/real2.yaml)가
  // 공통으로 쓰는 값 8을 기본값으로 삼는다 (nh.param 기본값과 동일하게 맞춤).
  int local_tsp_size_ = 8;
  // [feature: vp-viz] 샘플링된 viewpoint 후보 전체를 탈락 사유별 색으로 발행할지.
  // 후보는 클러스터당 radius_layer*circle_sample*height_layer 개라 수천 개가 될 수
  // 있어 기본 on 이되 화살표 수만 상한을 둔다 (구는 SPHERE_LIST 라 비용이 낮다).
  bool viz_candidates_ = true;
  int viz_max_yaw_arrows_ = 300;
  // 탈락 후보를 상태별로 이 개수까지만 그린다 (균등 간격 서브샘플).
  // circle_sample_num 을 올리면 후보가 클러스터당 수백 개가 되어 메시지당 수천
  // 점이 나오고, 그대로 그리면 rviz 가 버거워진다. VALID 는 이 상한을 받지 않는다.
  int viz_max_points_per_status_ = 4000;
  // [feature: topo-timeout] removeUnreachableViewpoints 의 topo A* 예산 [s].
  // 원래 3e-4(0.3ms) 로 박혀 있었는데, 같은 graphSearch 를 쓰는 다른 호출부는
  // 10ms(getPathCost) / 100ms(planGoalPath) 를 준다 — 33~333 배 차이다.
  // graphSearch 는 시간 초과와 "경로 없음"을 모두 false 로 돌려주므로, 예산이
  // 모자라면 그래프가 멀쩡히 연결돼 있어도 VP_INVALID_UNREACHABLE(주황) 이 된다.
  // 실측(5회차 09-08-30): topo A* 소요 median 0.119ms / p90 0.290ms / max 0.958ms.
  // 즉 0.3ms 는 p90 에 정확히 걸쳐 있어 먼 목표부터 시간만으로 탈락했다.
  double reachability_search_timeout_ = 3e-4;
  // [feature: vp-reached-clear] "뷰포인트까지 갔는데 프론티어가 그대로"인 클러스터를
  // 강제 해소한다. 원 저자가 CR_ODOM_DRIFT 로 의도했던 경로인데 세 가지가 깨져 있어
  // 한 번도 발동한 적이 없었다: 임계값 1cm/0.57deg, is_reachable_ 체크가 먼저 오는
  // continue, 그리고 index 를 id 로 비교하는 제거 조건.
  // 실측(4회차 09-23-37): 드론이 뷰포인트 0.07m 앞에 33초를 떠 있었고 클러스터 #519
  // (frt 25, see 17) 는 끝까지 살아남아 탐사가 비행 내내 한 목표에 묶였다.
  bool vp_reached_clear_enable_ = true;
  float vp_reached_pos_tol_ = 0.3f;   // [m]   best_vp_ 와의 거리 허용치
  float vp_reached_yaw_tol_ = 0.35f;  // [rad] best_vp_yaw_ 와의 차이 허용치 (~20deg)
  // [feature: vp-reached-clear-radius] 위 조건이 걸렸을 때, 목표 클러스터 하나만
  // 지우는 대신 드론 반경 이 거리 안의 프론티어를 전부 해소한다 [m]. 0 = 비활성.
  // 목표만 지우면 바로 옆 클러스터가 다음 목표가 되어 같은 자리에 다시 묶이는
  // 일이 있어서, "여기서는 더 볼 게 없다"를 한 번에 정리하는 강한 선택지다.
  // SPARSE 가 아니라 DENSE 로 찍는다 — 분류 루프에서 건너뛰는 상태는 DENSE 뿐이라
  // SPARSE 로 두면 다음 프레임에 곧바로 프론티어로 되돌아온다.
  float vp_reached_clear_radius_ = 0.0f;
};

// [feature: vp-viz] viewpoint 후보 하나의 최종 판정. 파이프라인이 후보를 버리는
// 지점마다 사유를 남겨서, rviz 에서 "왜 이 클러스터에 뷰포인트가 없는가"를
// 클러스터 단위가 아니라 후보 단위로 볼 수 있게 한다.
enum VP_STATUS {
  VP_VALID = 0,           // 가시 프론티어 확보(score>0) -> best yaw 유효
  VP_INVALID_CLEARANCE,   // 장애물 최소 이격(min_obstacle_clearance_) 미달
  VP_INVALID_OUT_OF_BOX,  // 탐사 박스 밖
  VP_INVALID_NO_REGION,   // 토포 그래프 region 없음
  VP_INVALID_ISOLATED,    // DB-SCAN 에서 어느 군집에도 못 붙음(고립 후보)
  VP_INVALID_UNREACHABLE, // 토포 그래프로 도달 불가
  VP_INVALID_PRUNED,      // 도달은 되나 거리 상위 후보에서 탈락
  VP_INVALID_NO_VIS,      // 가시 프론티어 셀 부족 (score == 0)
  VP_STATUS_COUNT
};

inline const char *vpStatusStr(uint8_t s) {
  switch (s) {
  case VP_VALID: return "VALID";
  case VP_INVALID_CLEARANCE: return "CLEARANCE";
  case VP_INVALID_OUT_OF_BOX: return "OUT_OF_BOX";
  case VP_INVALID_NO_REGION: return "NO_REGION";
  case VP_INVALID_ISOLATED: return "ISOLATED";
  case VP_INVALID_UNREACHABLE: return "UNREACHABLE";
  case VP_INVALID_PRUNED: return "PRUNED";
  case VP_INVALID_NO_VIS: return "NO_VISIBILITY";
  default: return "?";
  }
}

struct VpCandidate {
  Eigen::Vector3f pos_;
  float yaw_ = 0.0f;   // VP_VALID 일 때만 의미 있음
  int score_ = 0;      // 해당 yaw 에서 보이는 프론티어 셀 수
  uint8_t status_ = VP_INVALID_ISOLATED;
};

// [feature: vp-viz] 클러스터가 이번 라운드에 왜 쓰이지 못했는지. 기존에는
// is_reachable_/is_dormant_ 두 bool 로 뭉뚱그려져 rviz 에서 검정 박스의 원인을
// 구분할 수 없었다 (토포 도달 불가인지, 너무 작아서인지, 안 보여서인지).
enum CLUSTER_REASON {
  CR_OK = 0,           // 유효 뷰포인트 확보
  CR_NOT_CONSIDERED,   // 이번 라운드 top-K(local_tsp_size) 밖 -> 평가 안 함
  CR_TOO_SMALL,        // 박스 크기 < cluster_min_size_
  CR_TOO_FEW_CELLS,    // 프론티어 셀 수 < cluster_min_size_
  CR_NO_CANDIDATE,     // 기하 필터(이격/박스/region) 통과 후보 0
  CR_TOPO_UNREACHABLE, // 토포 그래프로 도달 불가
  CR_NO_VISIBILITY,    // 가시 프론티어 셀 < 3 (score == 0)
  CR_ODOM_DRIFT,       // 뷰포인트에 이미 도달했는데도 안 보임 -> odom 드리프트 의심
  CR_REASON_COUNT
};

inline const char *clusterReasonStr(uint8_t r) {
  switch (r) {
  case CR_OK: return "OK";
  case CR_NOT_CONSIDERED: return "NOT_CONSIDERED";
  case CR_TOO_SMALL: return "TOO_SMALL";
  case CR_TOO_FEW_CELLS: return "TOO_FEW_CELLS";
  case CR_NO_CANDIDATE: return "NO_CANDIDATE";
  case CR_TOPO_UNREACHABLE: return "TOPO_UNREACHABLE";
  case CR_NO_VISIBILITY: return "NO_VISIBILITY";
  case CR_ODOM_DRIFT: return "ODOM_DRIFT";
  default: return "?";
  }
}
struct ByteArrayRaw {
  uint8_t *data;
  static size_t size;

  ByteArrayRaw() { data = new uint8_t[size]; }

  ByteArrayRaw(const ByteArrayRaw &other) {
    data = new uint8_t[size];
    memcpy(data, other.data, size);
  }

  ByteArrayRaw &operator=(const ByteArrayRaw &other) {
    if (this != &other) {
      memcpy(data, other.data, size);
    }
    return *this;
  }

  ~ByteArrayRaw() { delete[] data; }

  bool operator==(const ByteArrayRaw &other) const {
    return memcmp(data, other.data, size) == 0;
  }
};

struct ByteArrayRawHasher {
  size_t operator()(const ByteArrayRaw &arr) const {
    const uint64_t seed = 0xc3a5c85c97cb3127ULL;
    const uint64_t m = 0xc6a4a7935bd1e995ULL;
    const int r = 47;

    size_t h = seed ^ (ByteArrayRaw::size * m);

    // 处理8字节对齐的数据
    const uint64_t *data = reinterpret_cast<const uint64_t *>(arr.data);
    const uint64_t *end = data + (ByteArrayRaw::size / 8);

    while (data != end) {
      uint64_t k = *data++;

      k *= m;
      k ^= k >> r;
      k *= m;

      h ^= k;
      h *= m;
    }

    // 处理剩余字节
    const unsigned char *tail = reinterpret_cast<const unsigned char *>(data);

    switch (ByteArrayRaw::size & 7) {
    case 7:
      h ^= uint64_t(tail[6]) << 48;
    case 6:
      h ^= uint64_t(tail[5]) << 40;
    case 5:
      h ^= uint64_t(tail[4]) << 32;
    case 4:
      h ^= uint64_t(tail[3]) << 24;
    case 3:
      h ^= uint64_t(tail[2]) << 16;
    case 2:
      h ^= uint64_t(tail[1]) << 8;
    case 1:
      h ^= uint64_t(tail[0]);
      h *= m;
    }

    h ^= h >> r;
    h *= m;
    h ^= h >> r;

    return h;
  }
};
struct FrontierData {
  FrontierData() { ByteArrayRaw::size = 8; }
  FrontierData(size_t idx_bytes_size) {
    ByteArrayRaw::size = idx_bytes_size; // 设置全局size
  }
  unordered_map<ByteArrayRaw, uint8_t, ByteArrayRawHasher> label_map_;
  unordered_map<ByteArrayRaw, Eigen::Vector3f, ByteArrayRawHasher> frt_map_;
  vector<bool> is_gap_;
  vector<float> direction_score_;
  vector<bool> is_fov_edge_;
  vector<float> sphere_distance_;
  Eigen::Vector3f updating_aabb_min;
  Eigen::Vector3f updating_aabb_max;
};

struct ViewpointCluster {
  ViewpointCluster() {
    center_ = Eigen::Vector3f::Zero();
    vps_.clear();
  }

  Eigen::Vector3f center_;
  PointVector vps_;
  // [feature: vp-viz] vps_ 와 1:1 대응하는 ClusterInfo::vp_candidates_ 인덱스.
  // 이후 단계(도달성/가시성)의 판정을 원래 샘플 후보로 되짚기 위한 것.
  vector<int> vp_cand_idx_;
  float distance_;
};

struct ClusterInfo {
  typedef shared_ptr<ClusterInfo> Ptr;
  // index of the frt
  PointVector cells_;
  vector<Eigen::Vector3f> norms_;
  Eigen::Vector3f center_;
  Eigen::Vector3f normal_;
  Eigen::Vector4f view_halfspace_;

  int id_;
  int odom_id_;
  vector<ViewpointCluster> vp_clusters_;
  // [feature: vp-viz] 이번 라운드에 샘플링된 viewpoint 후보 전체 + 탈락 사유.
  // vp_clusters_ 와 달리 파이프라인 중간에 비워지지 않으므로 rviz 에서 항상
  // "이 클러스터가 어느 단계에서 후보를 잃었는지" 볼 수 있다.
  vector<VpCandidate> vp_candidates_;
  Eigen::Vector3f best_vp_;
  float best_vp_yaw_;
  float distance_;
  // [feature: vp-viz] rviz 박스 라벨용 진단.
  // best_vp_score_: best_vp_/best_vp_yaw_ 에서 실제로 보이는 프론티어 셀 수.
  //   cells_.size() (박스가 담고 있는 셀 수) 와는 별개다 — 셀이 많아도 한
  //   뷰포인트에서 다 보이지는 않으므로 둘을 따로 표시한다.
  int best_vp_score_ = 0;
  uint8_t reason_ = CR_NOT_CONSIDERED;

  bool is_dormant_;
  bool is_reachable_;
  bool is_new_cluster_;

  // bbox
  Eigen::Vector3f box_max_;
  Eigen::Vector3f box_min_;
};

struct SuperClusterInfo {
  typedef shared_ptr<SuperClusterInfo> Ptr;
  vector<ClusterInfo::Ptr> sons;
  Eigen::Vector3f waypoint;
};

// generateTSPViewpoints() 파이프라인 단계별 탈락 카운트.
// "뷰포인트가 왜 0개인가"를 이벤트 로그/HUD에 그대로 표기하기 위한 계측.
struct VpPipelineStats {
  int total = 0;            // cluster_list_ 전체
  int dormant = 0;          // 휴면(is_dormant_) 스킵
  int unreachable_pre = 0;  // 이전 라운드에서 unreachable 판정 스킵
  int considered = 0;       // 이번 라운드 뷰포인트 생성 대상(top-K + 신규)
  int no_candidates = 0;    // initClusterViewpoints: 후보 0 (clearance/box/topo-region)
  int topo_unreachable = 0; // removeUnreachableViewpoints: 토포그래프 도달 불가
  int no_visibility = 0;    // selectBestViewpoint: 가시 프론티어 셀 부족(score 0)
  int ok = 0;               // 최종 살아남은 클러스터(=뷰포인트 수)

  // [feature: topo-timeout] 위의 topo_unreachable 은 "클러스터가 통째로 죽은 수"라
  // 원인을 못 가린다 — 그래프가 정말 끊겨서인지, A* 가 예산 안에 못 끝내서인지.
  // 아래는 그 판정을 내리는 vp_cluster 단위 탐색의 결과 내역이다. reach_timeout 이
  // 0 이 아니면 "도달 불가"의 일부는 기하가 아니라 시간 때문이라는 뜻이다.
  int reach_ok = 0;      // graphSearch 성공 (경로 확인됨)
  int reach_timeout = 0; // graphSearch 가 TIME_OUT 으로 실패 (예산 부족)
  int reach_nopath = 0;  // graphSearch 가 경로 없음으로 실패 (진짜 단절)
  int reach_noedge = 0;  // 삽입한 vp 노드가 이웃을 하나도 못 얻음 (raycast 실패)

  std::string str() const {
    char b[384];
    snprintf(b, sizeof(b),
             "pipeline[total=%d dormant=%d prev_unreachable=%d evaluated=%d "
             "no_candidate=%d topo_unreachable=%d no_visibility=%d survived=%d] "
             "reach[ok=%d timeout=%d nopath=%d noedge=%d]",
             total, dormant, unreachable_pre, considered, no_candidates,
             topo_unreachable, no_visibility, ok, reach_ok, reach_timeout,
             reach_nopath, reach_noedge);
    return b;
  }
};

class FrontierManager {
private:
  FrontierParam frtp_;
  FrontierData frtd_;
  ViewpointParam vpp_;
  int logging_detail_ = 1;
  unordered_set<int> force_recluster_;
  LIOInterface::Ptr
      lidar_map_interface_; // TODO: 改成
                            // sim_lio,去掉不必要的接口,ikd-treeh和odom存储在里面
  TopoGraph::Ptr graph_;
  ros::NodeHandle nh_;
  vector<Eigen::Vector3f> origin_viewpoints_;
  
  // Publishers for timing data
  ros::Publisher vp_cluster_cost_pub_, remove_unreachable_cost_pub_, select_vp_cost_pub_;
  ros::Publisher explored_cell_count_pub_;
  // frontier update functions:
  void update_lidar_pos();
  void update_updating_aabb(const PointVector &new_frt_pts);
  void cluster_frts(const PointVector &frt_new,
                    vector<ClusterInfo::Ptr> &new_clusters,
                    vector<int> &cluster_removed);
  void compute_cluster_info(const PointVector &frt_pts,
                            const vector<Eigen::Vector3f> &frt_norms,
                            ClusterInfo::Ptr cluster);
  bool has_overlap(const Eigen::Vector3f &box_max_,
                   const Eigen::Vector3f &box_min_);
  void project_pts_2_depth_image(PointVector &pts_vec,
                                 vector<float> &depth_img);
  int surface_pos2idx(const PointType &pt); // 2w个点
  Eigen::Vector3f map_min_bd_, map_max_bd_;
  void pos2idx(const PointType &pt, Eigen::Vector3i &idx);
  void pos2idx(const Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  void pos2bytes(const PointType &pt, ByteArrayRaw &bytes);
  void bytes2pos(const ByteArrayRaw &bytes, PointType &pt);

  void idx2bytes(const Eigen::Vector3i &idx, ByteArrayRaw &bytes);
  void idx2pos(const Eigen::Vector3i &idx, PointType &pt);

  void update_lidar_pt_gap(const vector<float> &depth);
  void update_lidar_fov_edge(const vector<float> &depth);
  void computeNormal(const PointVector &local_pts, Eigen::Vector3f &normal);
  void computeNormalCell(const PointVector &local_pts, Eigen::Vector3f &normal,
                         Eigen::Vector3f &center);
  void Sphere_PosToIndex(const Eigen::Vector3f &lidar_center,
                         const Eigen::Vector3f &pos, Eigen::Vector2i &id);
  CELL_STATE get_state(const PointType &pt);
  CELL_STATE get_state(const Eigen::Vector3i &idx);
  bool is_gap_point(const PointType &pt);
  bool is_fov_edge(const PointType &pt);
  void get_cells_2_update(const PointVector &points,
                          vector<Eigen::Vector3i> &cells_2_update);
  void get_pts_in_cells(const vector<Eigen::Vector3i> &cells_2_update,
                        vector<PointVector> &pts_inside);
  void updateHalfSpaces(vector<ClusterInfo::Ptr> &clusters);
  //                                const vector<float> &bubble_radius);
  void selectBestViewpoint(ClusterInfo::Ptr &cluster);
  void initClusterViewpoints(ClusterInfo::Ptr &cluster);
  void removeUnreachableViewpoints(vector<ClusterInfo::Ptr> &clusters);
  bool isInBox(const PointType &pt);
  bool isInBox(const Eigen::Vector3f &pt);
  // [feature: box-margin] pt 가 탐사 박스 경계에서 margin*cell_size 이내인가 (6면 프로브)
  bool is_near_box_boundary(const PointType &pt);
  // [feature: perception-config-check] lidar_perception/* 조합의 모순을 시작 시
  // 한 번 검사해 경고만 남긴다 (동작 변경 없음). 자세한 배경은 .cpp 주석 참고.
  void checkPerceptionConfig(ros::NodeHandle &nh);
  bool computeSuperClusterInfo(SuperClusterInfo::Ptr &super_cluster);
  void reclusterSuperCluster(SuperClusterInfo::Ptr &super_cluster);

public:
  typedef std::shared_ptr<FrontierManager> Ptr;
  Eigen::Isometry3f transform_world2lidar;

  void init(ros::NodeHandle &nh, LIOInterface::Ptr &lio_interface,
            TopoGraph::Ptr graph);
  // unordered_map<int, ClusterInfo::Ptr> new_clusters_;
  std::list<ClusterInfo::Ptr> cluster_list_;
  VpPipelineStats vp_stats_; // 마지막 generateTSPViewpoints 단계별 통계
  // [feature: cone-clip] this frame's FOV-edge frontier cells (rebuilt every
  // updateFrontierClusters); consumed by the local planner to clip its SFC
  // corridor to the observed FOV cone.
  PointVector fov_edge_cells_;
  // Observation/occupancy state at a world point and the frontier grid size;
  // used by the local planner's cone clipping.
  CELL_STATE getCellState(const Eigen::Vector3f &p);
  float getCellSize() const { return frtp_.cell_size_; }
  // 현재 프론티어 셀 개수 (epic.log 진단용 — frt_map_ 은 private 라 getter 제공)
  size_t frontierCellCount() const { return frtd_.frt_map_.size(); }
  void printMemoryCost();

  void viz_pocc();
  void viz_point(PointVector &pts2viz, string topic_name);
  void viz_point(vector<Eigen::Vector3f> &pts2viz, string topic_name);
  void visfrtnorm(const std::vector<Eigen::Vector3f> &centers,
                  const std::vector<Eigen::Vector3f> &normals);
  void visfrtcluster();
  void vizBestViewpoint();

  // [EARLY_FINISH] EFP(early-finish point) 마커용 경계 인터페이스.
  // EFP 는 FSM 쪽 상태라 FrontierManager 는 그 존재를 모른다. FSM 이 매 계획
  // 주기마다 이 setter 로 최신 값을 밀어넣고, 그리는 쪽(vizBestViewpoint)은
  // 아래 세 멤버만 읽는다. 마커를 vizBestViewpoint 안에서 그려야 하는 이유는
  // 그 함수가 viewpoint_candidates 토픽에 매 주기 DELETEALL 을 먼저 쏘기
  // 때문이다 — 밖에서 따로 발행하면 0.2s 마다 지워진다.
  // EFP 가 소멸하면 active=false 로 꺼야 유령 마커가 남지 않는다.
  void setEarlyFinishMarker(bool active, const Eigen::Vector3f &pos, float yaw);
  bool efp_marker_active_ = false;
  Eigen::Vector3f efp_marker_pos_ = Eigen::Vector3f::Zero();
  float efp_marker_yaw_ = 0.0f;

  void updateFrontierClusters(vector<ClusterInfo::Ptr> &cluster_updated,
                              vector<int> &cluster_removed);
  void generateTSPViewpoints(Eigen::Vector3f &center_pose,
                             vector<TopoNode::Ptr> &viewpoints);
};
