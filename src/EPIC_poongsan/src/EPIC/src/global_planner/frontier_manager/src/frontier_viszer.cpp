/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-08 15:24:56
 * @LastEditTime: 2024-08-05 20:55:22
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <frontier_manager/frontier_manager.h>
#include <frontier_manager/global_log.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

enum VizColor { RED = 0, ORANGE = 1, BLACK = 2, YELLOW = 3, BLUE = 4, GREEN = 5, EMERALD = 6, WHITE = 7, MAGNA = 8, PURPLE = 9 };

void inline static SetColor(const VizColor &color, const float &alpha, Marker &scan_marker) {
  std_msgs::ColorRGBA c;
  c.a = alpha;
  if (color == VizColor::RED) {
    c.r = 1.0f, c.g = c.b = 0.f;
  } else if (color == VizColor::ORANGE) {
    c.r = 1.0f, c.g = 0.45f, c.b = 0.1f;
  } else if (color == VizColor::BLACK) {
    c.r = c.g = c.b = 0.1f;
  } else if (color == VizColor::YELLOW) {
    c.r = c.g = 0.9f, c.b = 0.1;
  } else if (color == VizColor::BLUE) {
    c.b = 1.0f, c.r = 0.1f, c.g = 0.1f;
  } else if (color == VizColor::GREEN) {
    c.g = 0.9f, c.r = c.b = 0.f;
  } else if (color == VizColor::EMERALD) {
    c.g = c.b = 0.9f, c.r = 0.f;
  } else if (color == VizColor::WHITE) {
    c.r = c.g = c.b = 0.9f;
  } else if (color == VizColor::MAGNA) {
    c.r = c.b = 0.9f, c.g = 0.f;
  } else if (color == VizColor::PURPLE) {
    c.r = c.b = 0.5f, c.g = 0.f;
  }
  scan_marker.color = c;
}

void inline SetMarker(const VizColor &color, const std::string &ns, const float &scale, const float &alpha, Marker &scan_marker,
                      const float &scale_ratio) {
  scan_marker.header.frame_id = "odom";
  scan_marker.header.stamp = ros::Time::now();
  scan_marker.ns = ns;
  scan_marker.action = Marker::ADD;
  scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * scale_ratio;
  scan_marker.pose.orientation.x = 0.0;
  scan_marker.pose.orientation.y = 0.0;
  scan_marker.pose.orientation.z = 0.0;
  scan_marker.pose.orientation.w = 1.0;
  scan_marker.pose.position.x = 0.0;
  scan_marker.pose.position.y = 0.0;
  scan_marker.pose.position.z = 0.0;
  SetColor(color, alpha, scan_marker);
}

void FrontierManager::visfrtcluster() {
  if (!frtp_.view_cluster_)
    return;
  static ros::Publisher sf_cluster_pub = nh_.advertise<visualization_msgs::MarkerArray>("sf_cluster_marker", 5);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(marker);

  for (auto &sf_cluster : cluster_list_) {

    visualization_msgs::Marker aabb_marker, viewpoint_number;
    visualization_msgs::Marker best_viewpoint, vp_frt_connecter;
    if (!sf_cluster->is_reachable_ || sf_cluster->is_dormant_) {
      SetMarker(VizColor::BLACK, "aabb", 1.0, 0.5, aabb_marker, 1.0);
    } else if (!sf_cluster->is_new_cluster_) {
      SetMarker(VizColor::GREEN, "aabb", 1.0, 0.5, aabb_marker, 1.0);
    } else {
      SetMarker(VizColor::RED, "aabb", 1.0, 0.5, aabb_marker, 1.0);
    }
    SetMarker(VizColor::WHITE, "cluster_label", 0.35, 1.0, viewpoint_number, 1.0);
    SetMarker(VizColor::RED, "best_viewpoint", 0.5, 1.0, best_viewpoint, 1.0);
    SetMarker(VizColor::WHITE, "vp_frt_connecter", 0.05, 0.7, vp_frt_connecter, 1.0);

    aabb_marker.id = sf_cluster->id_;
    viewpoint_number.id = sf_cluster->id_;
    best_viewpoint.id = sf_cluster->id_;
    vp_frt_connecter.id = sf_cluster->id_;

    aabb_marker.type = visualization_msgs::Marker::CUBE;
    viewpoint_number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    best_viewpoint.type = visualization_msgs::Marker::ARROW;
    vp_frt_connecter.type = visualization_msgs::Marker::LINE_STRIP;

    aabb_marker.pose.position.x = (sf_cluster->box_min_.x() + sf_cluster->box_max_.x()) / 2.0;
    aabb_marker.pose.position.y = (sf_cluster->box_min_.y() + sf_cluster->box_max_.y()) / 2.0;
    aabb_marker.pose.position.z = (sf_cluster->box_min_.z() + sf_cluster->box_max_.z()) / 2.0;
    aabb_marker.scale.x = sf_cluster->box_min_.x() - sf_cluster->box_max_.x();
    aabb_marker.scale.y = sf_cluster->box_min_.y() - sf_cluster->box_max_.y();
    aabb_marker.scale.z = sf_cluster->box_min_.z() - sf_cluster->box_max_.z();
    viewpoint_number.pose = aabb_marker.pose;
    // [feature: vp-viz] 라벨을 "숫자 하나"에서 진단 블록으로 확장.
    // 기존 숫자는 vp_clusters_ 안의 샘플 수였는데, vp_clusters_ 는 파이프라인
    // 중간에 비워지므로(도달불가/비가시/top-K 밖) 대부분 0 으로만 보였다.
    // 이제 각 수치를 분리해서 보여준다:
    //   frt  = 이 박스가 담고 있는 프론티어 셀 수 (cells_.size())
    //   vp   = 유효 후보 / 샘플링된 전체 후보  (vp_candidates_ 기준)
    //   see  = best 뷰포인트에서 실제로 보이는 프론티어 셀 수 (best_vp_score_)
    // frt 와 see 는 별개다 — 셀이 많아도 한 뷰포인트에서 다 보이지는 않는다.
    int vp_sampled = (int)sf_cluster->vp_candidates_.size();
    int vp_valid = 0;
    for (auto &c : sf_cluster->vp_candidates_)
      if (c.status_ == VP_VALID)
        vp_valid++;

    std::ostringstream lbl;
    lbl << "#" << sf_cluster->id_ << " "
        << clusterReasonStr(sf_cluster->reason_) << "\n"
        << "frt " << sf_cluster->cells_.size();
    if (vp_sampled > 0)
      lbl << "  vp " << vp_valid << "/" << vp_sampled;
    if (sf_cluster->reason_ == CR_OK)
      lbl << "\nsee " << sf_cluster->best_vp_score_;
    viewpoint_number.text = lbl.str();

    if (sf_cluster->is_reachable_ && !sf_cluster->is_dormant_ && !sf_cluster->is_new_cluster_) {
      best_viewpoint.scale.x = 0.2;
      best_viewpoint.scale.y = 0.5;
      best_viewpoint.scale.z = 0.5;
      Eigen::Vector3f best_vp = sf_cluster->best_vp_;
      float yaw = sf_cluster->best_vp_yaw_;
      Eigen::Vector3f diff(cos(yaw), sin(yaw), 0);
      geometry_msgs::Point pt;
      pt.x = best_vp.x();
      pt.y = best_vp.y();
      pt.z = best_vp.z();
      best_viewpoint.points.push_back(pt);
      vp_frt_connecter.points.push_back(pt);

      pt.x = best_vp.x() + diff.x();
      pt.y = best_vp.y() + diff.y();
      pt.z = best_vp.z() + diff.z();
      best_viewpoint.points.push_back(pt);
      marker_array.markers.push_back(best_viewpoint);
      Eigen::Vector3f center = (sf_cluster->box_min_ + sf_cluster->box_max_) / 2.0;
      pt.x = center.x();
      pt.y = center.y();
      pt.z = center.z();
      vp_frt_connecter.points.push_back(pt);
      marker_array.markers.push_back(vp_frt_connecter);
    }

    marker_array.markers.push_back(viewpoint_number);
    marker_array.markers.push_back(aabb_marker);
  }

  sf_cluster_pub.publish(marker_array);
}

void FrontierManager::visfrtnorm(const std::vector<Eigen::Vector3f> &centers, const std::vector<Eigen::Vector3f> &normals) {
  static ros::Publisher norm_pub = nh_.advertise<visualization_msgs::MarkerArray>("norm_directions", 1);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  for (size_t i = 0; i < centers.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom"; // 你的坐标系名称
    marker.header.stamp = ros::Time::now();
    marker.ns = "normal_directions";
    marker.id = i; // unique id for each marker
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    // 设置箭头方向
    geometry_msgs::Point end_point;
    geometry_msgs::Point start_point;
    start_point.x = centers[i].x();
    start_point.y = centers[i].y();
    start_point.z = centers[i].z();
    end_point.x = centers[i].x() + normals[i].x();
    end_point.y = centers[i].y() + normals[i].y();
    end_point.z = centers[i].z() + normals[i].z();
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // 设置颜色和尺寸
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.scale.x = 0.05; // 箭头宽度
    marker.scale.y = 0.08; // 箭头长度
    marker.scale.z = 0.1;

    marker_array.markers.push_back(marker);
  }

  norm_pub.publish(marker_array);
}

void FrontierManager::viz_point(PointVector &pts2viz, string topic_name) {
  static unordered_map<string, ros::Publisher> pub_map;
  ros::Publisher occ_pub;
  if (pub_map.count(topic_name))
    occ_pub = pub_map[topic_name];
  else {
    occ_pub = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 5);
    pub_map[topic_name] = occ_pub;
  }
  pcl::PointCloud<pcl::PointXYZ> occ_cloud;
  occ_cloud.width = pts2viz.size();
  occ_cloud.height = 1;
  occ_cloud.points = pts2viz;
  sensor_msgs::PointCloud2 occ_msg;
  pcl::toROSMsg(occ_cloud, occ_msg);
  occ_msg.header.stamp = ros::Time::now();
  occ_msg.header.frame_id = "odom";
  occ_pub.publish(occ_msg);
}

void FrontierManager::viz_point(vector<Eigen::Vector3f> &pts2viz, string topic_name) {

  PointVector pts;
  pts.reserve(pts2viz.size());
  for (auto &pt : pts2viz) {
    pts.emplace_back(pt.x(), pt.y(), pt.z());
  }
  viz_point(pts, topic_name);
}

void FrontierManager::viz_pocc() {
  if (!frtp_.view_frt_)
    return;
  // cout << "bucket_count: " << frtd_.label_map_.bucket_count() << endl;
  // cout << "load factor: " << frtd_.label_map_.load_factor() << endl;
  static ros::Publisher occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("occ", 5);
  static ros::Publisher pocc_pub = nh_.advertise<sensor_msgs::PointCloud2>("pocc", 5);
  static ros::Publisher frt_pub = nh_.advertise<sensor_msgs::PointCloud2>("frt", 5);
  PointVector occ_pts, pocc_pts, frt_pts;
  for (auto &[bytes, label] : frtd_.label_map_) {
    // Eigen::Vector3i idx = pt_label.first;
    // Eigen::Vector3f pt =
    // (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ + frtp_.map_min_;
    PointType pt;
    bytes2pos(bytes, pt);
    if (label == SPARSE) {
      pocc_pts.emplace_back(pt);
    } else if (label == DENSE) {
      occ_pts.emplace_back(pt);
    } else {
      frt_pts.emplace_back(pt);
    }
  }
  pcl::PointCloud<pcl::PointXYZ> occ_cloud;
  pcl::PointCloud<pcl::PointXYZ> pocc_cloud;
  pcl::PointCloud<pcl::PointXYZ> frt_cloud;
  occ_cloud.width = occ_pts.size();
  occ_cloud.height = 1;
  occ_cloud.points = occ_pts;
  pocc_cloud.width = pocc_pts.size();
  pocc_cloud.height = 1;
  pocc_cloud.points = pocc_pts;
  frt_cloud.width = frt_pts.size();
  frt_cloud.height = 1;
  frt_cloud.points = frt_pts;

  sensor_msgs::PointCloud2 occ_msg, pocc_msg, frt_msg;

  pcl::toROSMsg(occ_cloud, occ_msg);
  pcl::toROSMsg(pocc_cloud, pocc_msg);
  pcl::toROSMsg(frt_cloud, frt_msg);
  occ_msg.header.stamp = ros::Time::now();
  pocc_msg.header.stamp = ros::Time::now();
  frt_msg.header.stamp = ros::Time::now();
  occ_msg.header.frame_id = "odom";
  pocc_msg.header.frame_id = "odom";
  frt_msg.header.frame_id = "odom";

  occ_pub.publish(occ_msg);
  pocc_pub.publish(pocc_msg);
  frt_pub.publish(frt_msg);
}

// [EARLY_FINISH] FSM -> viszer 경계. 값만 보관하고 그리기는 vizBestViewpoint 가
// 한다 (그 함수가 viewpoint_candidates 토픽을 DELETEALL 로 초기화하므로).
void FrontierManager::setEarlyFinishMarker(bool active,
                                           const Eigen::Vector3f &pos,
                                           float yaw) {
  efp_marker_active_ = active;
  efp_marker_pos_ = pos;
  efp_marker_yaw_ = yaw;
}

// [feature: vp-viz] 샘플링된 viewpoint 후보 전체를 탈락 사유별 색으로,
// 유효(가시) 후보는 각자의 최적 yaw 화살표까지 발행한다.
//
// 토픽: viewpoint_candidates (visualization_msgs/MarkerArray)
//   ns=vp/<STATUS>  : 상태별 SPHERE_LIST (상태당 마커 1개 -> 후보가 수천 개여도 저렴)
//   ns=vp_yaw       : 유효 후보별 ARROW (최적 yaw). viz_max_yaw_arrows_ 로 상한
//   ns=vp_best      : 클러스터별 best viewpoint 의 굵은 화살표
//   ns=vp/EFP       : EARLY_FINISH 목표점(EFP) 의 마젠타 화살표. vp_best 와 같은
//                     모양/크기이고 viz_candidates_ 가 꺼져 있어도 발행된다
//
// 클러스터 박스(sf_cluster_marker)가 "이 클러스터가 왜 죽었나"를 보여준다면,
// 이쪽은 "후보 하나하나가 어느 단계에서 걸러졌나"를 보여준다.
void FrontierManager::vizBestViewpoint() {
  static ros::Publisher vp_pub =
      nh_.advertise<visualization_msgs::MarkerArray>("viewpoint_candidates", 5);

  // [EARLY_FINISH] EFP(early-finish point) 화살표. best-viewpoint 화살표(:vp_best)
  // 와 같은 모양/크기(ARROW, scale 0.10/0.22/0.25, 길이 1.2m)에 색만 마젠타로
  // 바꾼다. 조기 리턴 경로/통상 경로 양쪽에서 그대로 재사용한다.
  auto makeEfpMarker = [&]() -> visualization_msgs::Marker {
    visualization_msgs::Marker m;
    SetMarker(VizColor::MAGNA, "vp/EFP", 1.0, 1.0, m, 1.0);
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.10;
    m.scale.y = 0.22;
    m.scale.z = 0.25;
    // 공용 VizColor 팔레트는 두지 않고 RGB 를 직접 지정한다 (아래 상태별 색과
    // 같은 이유 — MAGNA 팔레트 값이 다른 팔레트 색과 헷갈릴 수 있어 확실한
    // 마젠타로 덮어쓴다).
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;
    geometry_msgs::Point p;
    p.x = efp_marker_pos_.x();
    p.y = efp_marker_pos_.y();
    p.z = efp_marker_pos_.z();
    m.points.push_back(p);
    geometry_msgs::Point tip = p;
    tip.x += 1.2 * cos(efp_marker_yaw_);
    tip.y += 1.2 * sin(efp_marker_yaw_);
    m.points.push_back(tip);
    return m;
  };

  if (!vpp_.viz_candidates_) {
    // viz_candidates_ 는 후보 디버그 그림 전용 스위치다. EFP 는 "탐사가 곧
    // 끝난다"는 별개의 신호라 이 플래그와 무관하게 항상 보여야 한다. EFP 도
    // 없으면 원래 동작(무발행) 그대로 리턴한다.
    if (!efp_marker_active_)
      return;
    visualization_msgs::MarkerArray efp_only;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    efp_only.markers.push_back(del);
    efp_only.markers.push_back(makeEfpMarker());
    vp_pub.publish(efp_only);
    return;
  }

  // 상태별 색. 공용 VizColor 팔레트를 쓰지 않고 RGB 를 직접 지정한다 —
  // 그쪽 ORANGE(1,0.45,0.1)/YELLOW(0.9,0.9,0.1) 와 PURPLE/MAGNA 는 서로 너무
  // 가까워서, 실제로 가장 많이 뜨는 UNREACHABLE 과 NO_VIS 를 눈으로 구분할 수
  // 없었다. 실전에서 동시에 많이 보이는 네 가지
  // (CLEARANCE / UNREACHABLE / OUT_OF_BOX / NO_VIS) 를 최대한 벌려 놓는다.
  struct Rgb { float r, g, b; };
  static const Rgb kStatusColor[VP_STATUS_COUNT] = {
      {0.10f, 0.95f, 0.20f},  // VP_VALID                초록
      {0.95f, 0.10f, 0.10f},  // VP_INVALID_CLEARANCE    빨강   장애물에 너무 붙음
      {0.45f, 0.45f, 0.45f},  // VP_INVALID_OUT_OF_BOX   회색   탐사 박스 밖
      {0.65f, 0.20f, 0.95f},  // VP_INVALID_NO_REGION    보라   토포 region 없음
      {1.00f, 0.40f, 0.75f},  // VP_INVALID_ISOLATED     분홍   DB-SCAN 고립
      {1.00f, 0.50f, 0.00f},  // VP_INVALID_UNREACHABLE  주황   토포 도달 불가
      {0.20f, 0.35f, 1.00f},  // VP_INVALID_PRUNED       파랑   거리 컷 탈락
      {0.00f, 0.90f, 0.90f},  // VP_INVALID_NO_VIS       청록   가시 셀 부족
  };
  static const char *kStatusNs[VP_STATUS_COUNT] = {
      "vp/VALID",       "vp/CLEARANCE", "vp/OUT_OF_BOX", "vp/NO_REGION",
      "vp/ISOLATED",    "vp/UNREACHABLE", "vp/PRUNED",   "vp/NO_VIS",
  };

  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker del;
  del.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(del);

  // 상태별 SPHERE_LIST 하나씩 준비.
  std::vector<visualization_msgs::Marker> spheres(VP_STATUS_COUNT);
  for (int s = 0; s < VP_STATUS_COUNT; s++) {
    // 탈락 후보는 클러스터당 수백 개 x 클러스터 수 = 메시지당 수천 점이 된다.
    // SPHERE_LIST 는 점마다 구 메시를 인스턴스화해서 rviz 가 그 물량을 감당하지
    // 못하고 표시가 지연되거나 아예 안 그려진다. 대량은 POINTS(빌보드)로 그린다.
    // 수가 적고 중요한 VALID 만 구로 남겨 눈에 띄게 한다.
    const bool valid = (s == VP_VALID);
    const float scale = valid ? 0.20f : 0.12f;
    const float alpha = valid ? 1.0f : 0.80f;
    SetMarker(VizColor::WHITE, kStatusNs[s], scale, alpha, spheres[s], 1.0);
    spheres[s].color.r = kStatusColor[s].r;
    spheres[s].color.g = kStatusColor[s].g;
    spheres[s].color.b = kStatusColor[s].b;
    spheres[s].color.a = alpha;
    spheres[s].id = s;
    spheres[s].type = valid ? visualization_msgs::Marker::SPHERE_LIST
                            : visualization_msgs::Marker::POINTS;
    if (!valid) {
      // POINTS 는 scale.x/y 만 쓴다 (화면상 점 크기 [m]). z 는 무시된다.
      spheres[s].scale.x = spheres[s].scale.y = scale;
      spheres[s].scale.z = 0.0;
    }
  }

  std::vector<visualization_msgs::Marker> arrows;
  std::vector<visualization_msgs::Marker> bests;
  int valid_total = 0;
  // 상태별 후보 총수를 먼저 세어 균등 간격 서브샘플 stride 를 정한다.
  // (앞쪽만 잘라내면 공간적으로 편향된 그림이 되므로 stride 로 고르게 솎는다.)
  std::vector<size_t> status_total(VP_STATUS_COUNT, 0), status_seen(VP_STATUS_COUNT, 0);
  for (auto &cluster : cluster_list_)
    for (auto &cand : cluster->vp_candidates_)
      if (cand.status_ < VP_STATUS_COUNT)
        status_total[cand.status_]++;
  std::vector<size_t> stride(VP_STATUS_COUNT, 1);
  const size_t cap = (size_t)std::max(1, vpp_.viz_max_points_per_status_);
  for (int s = 0; s < VP_STATUS_COUNT; s++)
    if (s != VP_VALID && status_total[s] > cap)
      stride[s] = (status_total[s] + cap - 1) / cap;

  for (auto &cluster : cluster_list_) {
    for (auto &cand : cluster->vp_candidates_) {
      if (cand.status_ >= VP_STATUS_COUNT)
        continue;
      const size_t idx = status_seen[cand.status_]++;
      if (idx % stride[cand.status_] != 0)
        continue;
      geometry_msgs::Point p;
      p.x = cand.pos_.x();
      p.y = cand.pos_.y();
      p.z = cand.pos_.z();
      spheres[cand.status_].points.push_back(p);
      if (cand.status_ != VP_VALID)
        continue;
      valid_total++;
      if ((int)arrows.size() >= vpp_.viz_max_yaw_arrows_)
        continue;
      // 유효 후보의 최적 yaw 화살표. 길이는 고정 0.6m (방향만 보이면 됨).
      visualization_msgs::Marker a;
      SetMarker(VizColor::EMERALD, "vp_yaw", 1.0, 0.9, a, 1.0);
      a.id = (int)arrows.size();
      a.type = visualization_msgs::Marker::ARROW;
      a.scale.x = 0.04;  // shaft diameter
      a.scale.y = 0.09;  // head diameter
      a.scale.z = 0.10;  // head length
      a.points.push_back(p);
      geometry_msgs::Point tip = p;
      tip.x += 0.6 * cos(cand.yaw_);
      tip.y += 0.6 * sin(cand.yaw_);
      a.points.push_back(tip);
      arrows.push_back(a);
    }

    // best viewpoint: 굵고 길게, 위 후보 화살표와 구분되는 색.
    if (cluster->reason_ != CR_OK)
      continue;
    visualization_msgs::Marker b;
    SetMarker(VizColor::RED, "vp_best", 1.0, 1.0, b, 1.0);
    b.id = cluster->id_;
    b.type = visualization_msgs::Marker::ARROW;
    b.scale.x = 0.10;
    b.scale.y = 0.22;
    b.scale.z = 0.25;
    geometry_msgs::Point p;
    p.x = cluster->best_vp_.x();
    p.y = cluster->best_vp_.y();
    p.z = cluster->best_vp_.z();
    b.points.push_back(p);
    geometry_msgs::Point tip = p;
    tip.x += 1.2 * cos(cluster->best_vp_yaw_);
    tip.y += 1.2 * sin(cluster->best_vp_yaw_);
    b.points.push_back(tip);
    bests.push_back(b);
  }

  for (int s = 0; s < VP_STATUS_COUNT; s++)
    if (!spheres[s].points.empty())
      arr.markers.push_back(spheres[s]);
  arr.markers.insert(arr.markers.end(), arrows.begin(), arrows.end());
  arr.markers.insert(arr.markers.end(), bests.begin(), bests.end());

  // 상한에 걸려 잘라냈으면 조용히 넘어가지 않고 알린다 (그림이 전부인 줄 알면
  // 오판하게 된다).
  if (valid_total > (int)arrows.size())
    ROS_LOG_ONCE(::ros::console::levels::Warn, "global.viewpoint",
                 "[global.viewpoint] yaw arrows capped at %d of %d valid viewpoints "
                 "(raise ViewpointManager/viz_max_yaw_arrows)",
                 (int)arrows.size(), valid_total);
  for (int s = 0; s < VP_STATUS_COUNT; s++)
    if (stride[s] > 1)
      ROS_LOG_ONCE(::ros::console::levels::Warn, "global.viewpoint",
                   "[global.viewpoint] %s subsampled 1/%zu (%zu candidates; raise "
                   "ViewpointManager/viz_max_points_per_status)",
                   kStatusNs[s], stride[s], status_total[s]);

  // [EARLY_FINISH] EFP 마커도 같은 arr 에 담아 발행한다 — 별도 퍼블리셔로 쏘면
  // 이 함수가 매 주기 앞에서 하는 DELETEALL 에 곧바로 지워진다. active 가
  // false 가 되면(=EFP 소멸) 다음 주기부터 이 push_back 이 실행되지 않고,
  // 위 DELETEALL 이 이전 주기의 EFP 마커까지 지우므로 유령 마커 없이 사라진다.
  if (efp_marker_active_)
    arr.markers.push_back(makeEfpMarker());

  vp_pub.publish(arr);
}
