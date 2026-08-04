#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "gcopter/geo_utils.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/trajectory.hpp"

// Visualizer for the planner
class Visualizer {
private:
  // config contains the scale for some markers
  ros::NodeHandle nh;

  // These are publishers for path, waypoints on the trajectory,
  // the entire trajectory, the mesh of free-space polytopes,
  // the edge of free-space polytopes, and spheres for safety radius
  ros::Publisher routePub;
  ros::Publisher routeIdPub;
  ros::Publisher wayPointsPub;
  ros::Publisher trajectoryPub;
  ros::Publisher meshPub;
  ros::Publisher edgePub;
  ros::Publisher meshPubOrig;
  ros::Publisher edgePubOrig;
  ros::Publisher spherePub;
  ros::Publisher PolysGenerate_timecostPub;
  ros::Publisher trajOptimize_timecostPub;
  ros::Publisher pointCloudProcess_timecostPub;
  ros::Publisher totoalOptimize_timecostPub;
  
  // Local planning detailed timing publishers
  ros::Publisher fast_searcher_search_cost_pub_;
  ros::Publisher bubble_astar_search_cost_pub_;
  ros::Publisher topo_graph_search_cost_pub_;
  ros::Publisher trajectory_generation_cost_pub_;
  ros::Publisher yaw_trajectory_optimization_cost_pub_;
  ros::Publisher lbfgs_optimization_cost_pub_;
  ros::Publisher path_collision_check_cost_pub_;
  ros::Publisher collision_check_cost_pub_;
  ros::Publisher velocity_check_cost_pub_;

  // ---- /visualizer/trajectory : LA-Planner-style red path + green FOV
  // frustums that vanish as the *planned* time advances.
  // Port of la_planner PlanningVisualization::setFOVmarker / drawYawFOVTraj
  // (the same viz the JAX local planner publishes on
  // /local_planner/optimal_trajectory). The fade is driven purely by the
  // trajectory's own time parameterization -- where the drone is *supposed*
  // to be at t = now - traj_start_time -- never by odometry.
  struct FovSample {
    Eigen::Vector3d pos;
    double yaw;
    double t;
  };
  std::vector<geometry_msgs::Point> traj_line_pts_;
  std::vector<FovSample> traj_fov_samples_;
  ros::Time traj_start_time_;
  bool traj_valid_ = false;
  int last_skip_count_ = -1;
  ros::Timer traj_viz_timer_;

  double fov_depth_ = 0.3;
  double fov_h_rad_ = 79.1396 * M_PI / 180.0;
  double fov_v_rad_ = 63.5803 * M_PI / 180.0;
  double fov_sample_dt_ = 0.3;
  int fov_max_slots_ = 50;
  double traj_line_sample_dt_ = 0.05;

public:
  ros::Publisher speedPub;
  ros::Publisher thrPub;
  ros::Publisher tiltPub;
  ros::Publisher bdrPub;
  ros::Publisher cloud_inputPub;

public:
  Visualizer() {};

  void init(ros::NodeHandle &nh_) {
    nh = nh_;
    routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
    routeIdPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/routeid", 10);
    wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
    trajectoryPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/trajectory", 10);
    meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000, true);
    edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000, true);
    meshPubOrig = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh_origin", 1000, true);
    edgePubOrig = nh.advertise<visualization_msgs::Marker>("/visualizer/edge_origin", 1000, true);
    spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
    speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
    thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
    tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
    bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);
    PolysGenerate_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/PolysGenerate_timecost", 1000);
    trajOptimize_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/trajOptimize_timecost", 1000);
    pointCloudProcess_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/pointCloudProcess_timecost", 1000);
    totoalOptimize_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/totoalOptimize_timecost", 1000);
    cloud_inputPub = nh.advertise<sensor_msgs::PointCloud2>("/visualizer/cloud_input", 10);
    fast_searcher_search_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/fast_searcher_search_cost", 1000);
    bubble_astar_search_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/bubble_astar_search_cost", 1000);
    topo_graph_search_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/topo_graph_search_cost", 1000);
    trajectory_generation_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/trajectory_generation_cost", 1000);
    yaw_trajectory_optimization_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/yaw_trajectory_optimization_cost", 1000);
    lbfgs_optimization_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/lbfgs_optimization_cost", 1000);
    path_collision_check_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/path_collision_check_cost", 1000);
    collision_check_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/collision_check_cost", 1000);
    velocity_check_cost_pub_ = nh.advertise<std_msgs::Float64>("/visualizer/velocity_check_cost", 1000);

    double fov_h_deg = 79.1396, fov_v_deg = 63.5803, traj_viz_rate = 20.0;
    nh.param("visualizer/fov_depth", fov_depth_, 0.3);
    nh.param("visualizer/fov_horizontal_deg", fov_h_deg, 79.1396);
    nh.param("visualizer/fov_vertical_deg", fov_v_deg, 63.5803);
    nh.param("visualizer/fov_sample_dt", fov_sample_dt_, 0.3);
    nh.param("visualizer/fov_max_slots", fov_max_slots_, 50);
    nh.param("visualizer/traj_line_sample_dt", traj_line_sample_dt_, 0.05);
    nh.param("visualizer/traj_viz_rate", traj_viz_rate, 20.0);
    fov_h_rad_ = fov_h_deg * M_PI / 180.0;
    fov_v_rad_ = fov_v_deg * M_PI / 180.0;
    fov_max_slots_ = std::max(1, fov_max_slots_);

    traj_viz_timer_ = nh.createTimer(ros::Duration(1.0 / traj_viz_rate),
                                     &Visualizer::trajVizTimerCallback, this);
  }

  inline Eigen::Vector3d jetColorMap(double value) {
    double r, g, b;
    if (value < 0.0)
      value = 0.0;
    else if (value > 1.0)
      value = 1.0;

    if (value < 0.25) {
      r = 0.0;
      g = 4.0 * value;
      b = 1.0;
    } else if (value < 0.5) {
      r = 0.0;
      g = 1.0;
      b = 1.0 - 4.0 * (value - 0.25);
    } else if (value < 0.75) {
      r = 4.0 * (value - 0.5);
      g = 1.0;
      b = 0.0;
    } else {
      r = 1.0;
      g = 1.0 - 4.0 * pow((value - 0.75), 1);
      b = 0.0;
    }

    return Eigen::Vector3d(r, g, b);
  }

  // Visualize the trajectory and its front-end path
  inline void visualizeRoute(const std::vector<Eigen::Vector3f> &route) {
    visualization_msgs::Marker routeMarker;
    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = ros::Time::now();
    routeMarker.header.frame_id = "odom";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.0f;
    routeMarker.color.g = 0.9f;
    routeMarker.color.b = 1.0f;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.1;
    if (route.size() > 0) {
      bool first = true;
      Eigen::Vector3f last;
      for (auto it : route) {
        if (first) {
          first = false;
          last = it;
          continue;
        }
        geometry_msgs::Point point;

        point.x = last(0);
        point.y = last(1);
        point.z = last(2);
        routeMarker.points.push_back(point);
        point.x = it(0);
        point.y = it(1);
        point.z = it(2);
        routeMarker.points.push_back(point);
        last = it;
      }
      // ROS_INFO("route num = %d", routeMarker.points.size());
      routePub.publish(routeMarker);
    }

  }

  // Build one green LINE_LIST frustum with its apex at `pos`, +X_body along
  // `yaw`. Yaw-only rotation (the camera stays level), matching the JAX port.
  inline visualization_msgs::Marker buildFovMarker(const Eigen::Vector3d &pos,
                                                   const double yaw,
                                                   const int id,
                                                   const ros::Time &stamp) const {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "odom";
    mk.header.stamp = stamp;
    mk.ns = "optimal_trajectory_fov";
    mk.id = id;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.00;
    mk.scale.x = fov_depth_ * 0.1;
    mk.color.r = 0.00;
    mk.color.g = 1.00;
    mk.color.b = 0.00;
    mk.color.a = 1.00;

    const double half_w = std::tan(fov_h_rad_ / 2.0) * fov_depth_;
    const double half_h = std::tan(fov_v_rad_ / 2.0) * fov_depth_;
    const double c = std::cos(yaw), s = std::sin(yaw);
    // camera frame (+X forward): the 4 far-plane corners
    const double cam_corners[4][3] = {
        {fov_depth_, -half_w, -half_h},
        {fov_depth_, half_w, -half_h},
        {fov_depth_, half_w, half_h},
        {fov_depth_, -half_w, half_h},
    };
    geometry_msgs::Point world[4];
    for (int i = 0; i < 4; i++) {
      world[i].x = c * cam_corners[i][0] - s * cam_corners[i][1] + pos.x();
      world[i].y = s * cam_corners[i][0] + c * cam_corners[i][1] + pos.y();
      world[i].z = cam_corners[i][2] + pos.z();
    }
    geometry_msgs::Point apex;
    apex.x = pos.x();
    apex.y = pos.y();
    apex.z = pos.z();

    for (int i = 0; i < 4; i++) {  // far-plane rectangle
      mk.points.push_back(world[i]);
      mk.points.push_back(world[(i + 1) % 4]);
    }
    for (int i = 0; i < 4; i++) {  // apex -> corner rays
      mk.points.push_back(apex);
      mk.points.push_back(world[i]);
    }
    return mk;
  }

  inline visualization_msgs::Marker staleFovMarker(const int id,
                                                   const ros::Time &stamp) const {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "odom";
    mk.header.stamp = stamp;
    mk.ns = "optimal_trajectory_fov";
    mk.id = id;
    mk.action = visualization_msgs::Marker::DELETE;
    return mk;
  }

  // Republish the cached trajectory. Frustums whose planned timestamp has
  // already elapsed are DELETEd, so they vanish one-by-one ahead of where the
  // drone is scheduled to be. The red path line always stays full-length
  // (LA parity: only the frustums shrink). Publishes only when the set of
  // surviving frustums actually changes, unless `force`.
  inline void publishTrajViz(const bool force) {
    if (!traj_valid_) return;

    const ros::Time stamp = ros::Time::now();
    const double elapsed = (stamp - traj_start_time_).toSec();
    int skip = 0;
    while (skip < static_cast<int>(traj_fov_samples_.size()) &&
           traj_fov_samples_[skip].t < elapsed) {
      skip++;
    }
    if (!force && skip == last_skip_count_) return;
    last_skip_count_ = skip;

    visualization_msgs::MarkerArray arr;

    visualization_msgs::Marker lineMarker;
    lineMarker.header.frame_id = "odom";
    lineMarker.header.stamp = stamp;
    lineMarker.ns = "optimal_trajectory";
    lineMarker.id = 0;
    lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
    lineMarker.action = visualization_msgs::Marker::ADD;
    lineMarker.pose.orientation.w = 1.00;
    lineMarker.scale.x = 0.08;
    lineMarker.color.r = 1.00;
    lineMarker.color.g = 0.00;
    lineMarker.color.b = 0.00;
    lineMarker.color.a = 1.00;
    lineMarker.points = traj_line_pts_;
    arr.markers.push_back(lineMarker);

    const int n = static_cast<int>(traj_fov_samples_.size());
    for (int i = skip; i < n; i++) {
      arr.markers.push_back(buildFovMarker(traj_fov_samples_[i].pos,
                                           traj_fov_samples_[i].yaw, i + 1,
                                           stamp));
    }
    for (int i = 0; i < skip; i++) {  // already flown past
      arr.markers.push_back(staleFovMarker(i + 1, stamp));
    }
    for (int i = n; i < fov_max_slots_; i++) {  // horizon shrank since last plan
      arr.markers.push_back(staleFovMarker(i + 1, stamp));
    }

    trajectoryPub.publish(arr);
  }

  void trajVizTimerCallback(const ros::TimerEvent &) { publishTrajViz(false); }

  // `traj_start_time` must be the same stamp the trajectory is executed
  // against (local_data_.start_time_) -- that is what makes the frustum fade
  // track planned time rather than wall-clock.
  template <int D, int DY>
  inline void visualize(const Trajectory<D> &traj,
                        const Trajectory<DY> &yaw_traj,
                        const ros::Time &traj_start_time, double max_vel) {
    visualization_msgs::Marker wayPointsMarker;
    visualization_msgs::Marker routeMarker;
    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = ros::Time::now();
    routeMarker.header.frame_id = "odom";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.0f;
    routeMarker.color.g = 0.9f;
    routeMarker.color.b = 1.0f;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.1;
    wayPointsMarker = routeMarker;
    wayPointsMarker.id = 0;
    wayPointsMarker.header.frame_id = "odom";
    wayPointsMarker.pose.orientation.w = 1.00;
    wayPointsMarker.action = visualization_msgs::Marker::ADD;

    wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    wayPointsMarker.ns = "waypoints";
    wayPointsMarker.color.r = 1.0f;
    wayPointsMarker.color.g = 0.0f;
    wayPointsMarker.color.b = 0.0f;
    wayPointsMarker.scale.x = 0.35;
    wayPointsMarker.scale.y = 0.35;
    wayPointsMarker.scale.z = 0.35;

    if (traj.getPieceNum() > 0) {
      Eigen::MatrixXd wps = traj.getPositions();
      for (int i = 0; i < wps.cols(); i++) {
        geometry_msgs::Point point;
        point.x = wps.col(i)(0);
        point.y = wps.col(i)(1);
        point.z = wps.col(i)(2);
        wayPointsMarker.points.push_back(point);
      }

      wayPointsPub.publish(wayPointsMarker);
    }

    (void)max_vel;  // velocity jet-colormap dropped in favour of the FOV view

    const double dur = traj.getTotalDuration();
    if (traj.getPieceNum() <= 0 || !std::isfinite(dur) || dur <= 0.0) {
      traj_valid_ = false;
      return;
    }

    // Yaw along the trajectory. The escape path carries the previous yaw traj
    // over, which can be shorter than the new position traj -- clamp to its
    // own duration. Only if there is no yaw traj at all (very first plan) fall
    // back to the velocity heading.
    const double yaw_dur =
        yaw_traj.getPieceNum() > 0 ? yaw_traj.getTotalDuration() : 0.0;
    const bool yaw_ok = std::isfinite(yaw_dur) && yaw_dur > 0.0;
    auto yawAt = [&](double t) -> double {
      if (yaw_ok) return yaw_traj.getPos(std::min(t, yaw_dur)).x();
      const Eigen::Vector3d v = traj.getVel(t);
      return v.head<2>().norm() > 1e-6 ? std::atan2(v.y(), v.x()) : 0.0;
    };

    traj_line_pts_.clear();
    traj_fov_samples_.clear();

    geometry_msgs::Point point;
    for (double t = 0.0; t < dur; t += traj_line_sample_dt_) {
      const Eigen::Vector3d X = traj.getPos(t);
      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      traj_line_pts_.push_back(point);
    }
    const Eigen::Vector3d Xend = traj.getPos(dur);
    point.x = Xend(0);
    point.y = Xend(1);
    point.z = Xend(2);
    traj_line_pts_.push_back(point);

    // Sparse frustums, never more than fov_max_slots_ (the DELETE-sweep range).
    //
    // A rotate-in-place recovery has a constant position polynomial and is
    // commonly only 0.30--0.40 s long. With the ordinary 0.30 s sampling it
    // produced only the t=0 frustum; by the time the ~40 ms planning callback
    // published it, publishTrajViz() quite correctly considered that sole
    // sample stale and deleted it. The command uses the normal position+yaw
    // pipeline, so visualize it through that same pipeline as a stationary
    // sequence of FOVs spanning the yaw polynomial (including its endpoint).
    const Eigen::Vector3d Xstart = traj.getPos(0.0);
    const bool stationary = (Xend - Xstart).norm() <= 1.0e-5;
    if (stationary && yaw_ok) {
      const double yaw_delta = std::abs(yawAt(dur) - yawAt(0.0));
      const int desired_samples = yaw_delta > 1.0e-5 ? 9 : 1;
      const int sample_count = std::min(fov_max_slots_, desired_samples);
      for (int i = 0; i < sample_count; ++i) {
        const double t = sample_count == 1
                             ? dur
                             : dur * static_cast<double>(i) /
                                   static_cast<double>(sample_count - 1);
        FovSample s;
        s.pos = traj.getPos(t);
        s.yaw = yawAt(t);
        s.t = t;
        traj_fov_samples_.push_back(s);
      }
    } else {
      const double fov_dt = std::max(fov_sample_dt_, dur / fov_max_slots_);
      for (double t = 0.0;
           t < dur &&
           static_cast<int>(traj_fov_samples_.size()) < fov_max_slots_;
           t += fov_dt) {
        FovSample s;
        s.pos = traj.getPos(t);
        s.yaw = yawAt(t);
        s.t = t;
        traj_fov_samples_.push_back(s);
      }
    }

    traj_start_time_ = traj_start_time;
    traj_valid_ = true;
    last_skip_count_ = -1;
    publishTrajViz(true);
  }

  inline void renderPolytope(const std::vector<Eigen::MatrixX4d> &hPolys,
                             ros::Publisher &mesh_pub, ros::Publisher &edge_pub,
                             double edge_r, double edge_g, double edge_b,
                             double mesh_r, double mesh_g, double mesh_b) {
    // Due to the fact that H-representation cannot be directly visualized
    // We first conduct vertex enumeration of them, then apply quickhull
    // to obtain triangle meshs of polyhedra
    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    for (size_t id = 0; id < hPolys.size(); id++) {
      oldTris = mesh;
      Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
      geo_utils::enumerateVs(hPolys[id], vPoly);

      quickhull::QuickHull<double> tinyQH;
      const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
      const auto &idxBuffer = polyHull.getIndexBuffer();
      int hNum = idxBuffer.size() / 3;

      curTris.resize(3, hNum * 3);
      for (int i = 0; i < hNum * 3; i++) {
        curTris.col(i) = vPoly.col(idxBuffer[i]);
      }
      mesh.resize(3, oldTris.cols() + curTris.cols());
      mesh.leftCols(oldTris.cols()) = oldTris;
      mesh.rightCols(curTris.cols()) = curTris;
    }

    // RVIZ support tris for visualization
    visualization_msgs::Marker meshMarker, edgeMarker;

    meshMarker.id = 0;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "odom";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = mesh_r;
    meshMarker.color.g = mesh_g;
    meshMarker.color.b = mesh_b;
    meshMarker.color.a = 0.07;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = edge_r;
    edgeMarker.color.g = edge_g;
    edgeMarker.color.b = edge_b;
    edgeMarker.color.a = 0.20;
    edgeMarker.scale.x = 0.02;

    geometry_msgs::Point point;

    int ptnum = mesh.cols();

    for (int i = 0; i < ptnum; i++) {
      point.x = mesh(0, i);
      point.y = mesh(1, i);
      point.z = mesh(2, i);
      meshMarker.points.push_back(point);
    }

    for (int i = 0; i < ptnum / 3; i++) {
      for (int j = 0; j < 3; j++) {
        point.x = mesh(0, 3 * i + j);
        point.y = mesh(1, 3 * i + j);
        point.z = mesh(2, 3 * i + j);
        edgeMarker.points.push_back(point);
        point.x = mesh(0, 3 * i + (j + 1) % 3);
        point.y = mesh(1, 3 * i + (j + 1) % 3);
        point.z = mesh(2, 3 * i + (j + 1) % 3);
        edgeMarker.points.push_back(point);
      }
    }

    mesh_pub.publish(meshMarker);
    edge_pub.publish(edgeMarker);

    return;
  }

  inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys,
                                bool red_edge = false) {
    const double edge_r = red_edge ? 1.00 : 0.00;
    const double edge_g = red_edge ? 0.00 : 1.00;
    renderPolytope(hPolys, meshPub, edgePub, edge_r, edge_g, 1.00,
                   0.00, 0.00, 1.00);
  }

  inline void visualizePolytopeOrigin(
      const std::vector<Eigen::MatrixX4d> &hPolys) {
    renderPolytope(hPolys, meshPubOrig, edgePubOrig, 1.00, 0.55, 0.00,
                   1.00, 0.55, 0.00);
  }

  // Visualize all spheres with centers sphs and the same radius
  inline void visualizeSphere(const Eigen::Vector3d &center1, const Eigen::Vector3d &center2, const double &radius) {
    visualization_msgs::Marker sphereMarkers, sphereDeleter;

    sphereMarkers.id = 0;
    sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp = ros::Time::now();
    sphereMarkers.header.frame_id = "odom";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::Marker::ADD;
    sphereMarkers.ns = "spheres";
    sphereMarkers.color.r = 0.00;
    sphereMarkers.color.g = 0.00;
    sphereMarkers.color.b = 1.00;
    sphereMarkers.color.a = 1.00;
    sphereMarkers.scale.x = radius * 2.0;
    sphereMarkers.scale.y = radius * 2.0;
    sphereMarkers.scale.z = radius * 2.0;

    sphereDeleter = sphereMarkers;
    sphereDeleter.action = visualization_msgs::Marker::DELETE;

    geometry_msgs::Point point;
    point.x = center1(0);
    point.y = center1(1);
    point.z = center1(2);
    sphereMarkers.points.push_back(point);
    point.x = center2(0);
    point.y = center2(1);
    point.z = center2(2);
    sphereMarkers.points.push_back(point);

    // spherePub.publish(sphereDeleter);
    spherePub.publish(sphereMarkers);
  }

  inline void visualizeStartGoal(const Eigen::Vector3d &center, const double &radius, const int sg) {
    visualization_msgs::Marker sphereMarkers, sphereDeleter;

    sphereMarkers.id = sg;
    sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp = ros::Time::now();
    sphereMarkers.header.frame_id = "odom";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::Marker::ADD;
    sphereMarkers.ns = "StartGoal";
    sphereMarkers.color.r = 1.00;
    sphereMarkers.color.g = 0.00;
    sphereMarkers.color.b = 0.00;
    sphereMarkers.color.a = 1.00;
    sphereMarkers.scale.x = radius * 2.0;
    sphereMarkers.scale.y = radius * 2.0;
    sphereMarkers.scale.z = radius * 2.0;

    sphereDeleter = sphereMarkers;
    sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

    geometry_msgs::Point point;
    point.x = center(0);
    point.y = center(1);
    point.z = center(2);
    sphereMarkers.points.push_back(point);

    if (sg == 0) {
      spherePub.publish(sphereDeleter);
      ros::Duration(1.0e-9).sleep();
      sphereMarkers.header.stamp = ros::Time::now();
    }
    spherePub.publish(sphereMarkers);
  }

  inline void visualizeTimeCost(const double PolysGenerate_time, const double trajOptimize_time, const double pointCloudProcess_time) {
    std_msgs::Float64 time_cost;
    time_cost.data = PolysGenerate_time;
    PolysGenerate_timecostPub.publish(time_cost);
    time_cost.data = trajOptimize_time;
    trajOptimize_timecostPub.publish(time_cost);
    time_cost.data = pointCloudProcess_time;
    pointCloudProcess_timecostPub.publish(time_cost);
    time_cost.data = PolysGenerate_time + pointCloudProcess_time + trajOptimize_time;
    totoalOptimize_timecostPub.publish(time_cost);
  }

  // Local planning detailed timing helper functions
  inline void publishFastSearcherSearchCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    fast_searcher_search_cost_pub_.publish(msg);
  }

  inline void publishBubbleAstarSearchCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    bubble_astar_search_cost_pub_.publish(msg);
  }

  inline void publishTopoGraphSearchCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    topo_graph_search_cost_pub_.publish(msg);
  }

  inline void publishTrajectoryGenerationCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    trajectory_generation_cost_pub_.publish(msg);
  }

  inline void publishYawTrajectoryOptimizationCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    yaw_trajectory_optimization_cost_pub_.publish(msg);
  }

  inline void publishLbfgsOptimizationCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    lbfgs_optimization_cost_pub_.publish(msg);
  }

  inline void publishPathCollisionCheckCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    path_collision_check_cost_pub_.publish(msg);
  }

  inline void publishCollisionCheckCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    collision_check_cost_pub_.publish(msg);
  }

  inline void publishVelocityCheckCost(double time_ms) {
    std_msgs::Float64 msg;
    msg.data = time_ms;
    velocity_check_cost_pub_.publish(msg);
  }

  // inline void visualizeCloud(
  //     const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  //     sensor_msgs::PointCloud2 cloud_msg;
  //     pcl::toROSMsg(*cloud, cloud_msg);
  //     cloud_msg.header.stamp = ros::Time::now();
  //     cloud_msg.header.frame_id = "world";
  //     cloud_inputPub.publish(cloud_msg);
  // }
};

#endif
