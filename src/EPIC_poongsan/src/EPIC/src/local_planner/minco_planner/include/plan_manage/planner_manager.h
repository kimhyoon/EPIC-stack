#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <path_searching/bubble_astar.h>

#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/PolyTraj.h>
#include <lidar_map/lidar_map.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include "misc/visualizer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pointcloud_topo/graph.h>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <pointcloud_topo/parallel_bubble_astar.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

// Defined in the frontier_manager package (global namespace). Forward-declared
// here so this header does not force every includer to pull in the frontier
// headers; the full definition is included only in planner_manager.cpp, which
// queries observed-frontier cells to clip the local SFC corridor.
class FrontierManager;

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called
struct GcopterConfig {
  std::string mapTopic;
  std::string targetTopic;
  double dilateRadiusSoft, dilateRadiusHard;
  double timeoutRRT;
  double maxVelMag;
  double maxAccMag;
  double maxBdrMag;
  double maxTiltAngle;
  double minThrust;
  double maxThrust;
  double vehicleMass;
  double gravAcc;
  double horizDrag;
  double vertDrag;
  double parasDrag;
  double speedEps;
  double weightT;
  double WeightSafeT;
  std::vector<double> chiVec;
  std::vector<double> pvaRunningWeights;
  std::vector<double> pvaInitialWeights;
  std::vector<double> pvaTerminalWeights;
  double smoothingEps;
  int integralIntervs;
  double relCostTol;
  double corridor_size;
  double yaw_max_vel;
  double yaw_rho_vis;
  double yaw_time_fwd;
  double yawDiffEps;
  double trigGradientEps;
  double vectorNormEps;
  double minSegmentTime;
  double flatnessNormEps;
  double minPathSegmentLength;
  double linearSolvePivotEps;
  double corridorObstacleVoxelSize;
  double corridorSearchMarginXY;
  double corridorSearchMarginZ;
  double corridorProgressLength;
  int corridorMaxObstacleSamples;
  double firiObstacleDistanceLimit;
  int firiMaxPlaneCount;
  int corridorGapViolationPlaneThreshold;
  double corridorOverlapTolerance;
  double corridorFallbackTargetMargin;
  double corridorFallbackMinProgress;

  void init(const ros::NodeHandle &nh_priv) {
    auto missingParam = [](const char *name) {
      ROS_FATAL("[NumericalGuard] required parameter %s is missing", name);
      throw std::runtime_error(std::string("missing parameter: ") + name);
    };
    auto requireDouble = [&nh_priv, &missingParam](const char *name,
                                                   double &value) {
      if (!nh_priv.getParam(name, value))
        missingParam(name);
    };
    auto requireInt = [&nh_priv, &missingParam](const char *name, int &value) {
      if (!nh_priv.getParam(name, value))
        missingParam(name);
    };
    auto requireVector = [&nh_priv, &missingParam](
                             const char *name, std::vector<double> &value) {
      if (!nh_priv.getParam(name, value))
        missingParam(name);
    };

    requireDouble("DilateRadiusSoft", dilateRadiusSoft);
    requireDouble("DilateRadiusHard", dilateRadiusHard);
    requireDouble("MaxVelMag", maxVelMag);
    requireDouble("MaxAccMag", maxAccMag);
    requireDouble("maxBdrMag", maxBdrMag);
    requireDouble("MaxTiltAngle", maxTiltAngle);
    requireDouble("MinThrust", minThrust);
    requireDouble("MaxThrust", maxThrust);
    requireDouble("VehicleMass", vehicleMass);
    requireDouble("GravAcc", gravAcc);
    requireDouble("HorizDrag", horizDrag);
    requireDouble("VertDrag", vertDrag);
    requireDouble("ParasDrag", parasDrag);
    requireDouble("SpeedEps", speedEps);
    requireDouble("WeightT", weightT);
    requireDouble("WeightSafeT", WeightSafeT);
    requireVector("ChiVec", chiVec);
    requireVector("PvaRunningWeights", pvaRunningWeights);
    requireVector("PvaInitialWeights", pvaInitialWeights);
    requireVector("PvaTerminalWeights", pvaTerminalWeights);
    requireDouble("SmoothingEps", smoothingEps);
    requireInt("IntegralIntervs", integralIntervs);
    requireDouble("RelCostTol", relCostTol);
    requireDouble("MaxCorridorSize", corridor_size);
    requireDouble("yaw_rho_vis", yaw_rho_vis);
    requireDouble("yaw_max_vel", yaw_max_vel);
    requireDouble("yaw_time_fwd", yaw_time_fwd);
    requireDouble("numerical/yaw_diff_eps", yawDiffEps);
    requireDouble("numerical/trig_gradient_eps", trigGradientEps);
    requireDouble("numerical/vector_norm_eps", vectorNormEps);
    requireDouble("numerical/min_segment_time", minSegmentTime);
    requireDouble("numerical/flatness_norm_eps", flatnessNormEps);
    requireDouble("numerical/min_path_segment_length", minPathSegmentLength);
    requireDouble("numerical/linear_solve_pivot_eps", linearSolvePivotEps);
    requireDouble("local_planning/corridor/obstacle_voxel_size",
                  corridorObstacleVoxelSize);
    requireDouble("local_planning/corridor/search_margin_xy",
                  corridorSearchMarginXY);
    requireDouble("local_planning/corridor/search_margin_z",
                  corridorSearchMarginZ);
    requireDouble("local_planning/corridor/progress_length",
                  corridorProgressLength);
    requireInt("local_planning/corridor/max_obstacle_samples",
               corridorMaxObstacleSamples);
    requireDouble("local_planning/corridor/firi_obstacle_distance_limit",
                  firiObstacleDistanceLimit);
    requireInt("local_planning/corridor/firi_max_plane_count",
               firiMaxPlaneCount);
    requireInt("local_planning/corridor/gap_violation_plane_threshold",
               corridorGapViolationPlaneThreshold);
    requireDouble("local_planning/corridor/overlap_tolerance",
                  corridorOverlapTolerance);
    requireDouble("local_planning/corridor/fallback_target_margin",
                  corridorFallbackTargetMargin);
    requireDouble("local_planning/corridor/fallback_min_progress",
                  corridorFallbackMinProgress);

    auto validatePositive = [](const char *name, double value) {
      if (!std::isfinite(value) || value <= 0.0) {
        ROS_FATAL("[NumericalGuard] %s must be positive and finite", name);
        throw std::runtime_error(std::string("invalid parameter: ") + name);
      }
    };
    auto validateFinite = [](const char *name, double value) {
      if (!std::isfinite(value)) {
        ROS_FATAL("[NumericalGuard] %s must be finite", name);
        throw std::runtime_error(std::string("invalid parameter: ") + name);
      }
    };
    validatePositive("DilateRadiusSoft", dilateRadiusSoft);
    validatePositive("DilateRadiusHard", dilateRadiusHard);
    validatePositive("MaxVelMag", maxVelMag);
    validatePositive("MaxAccMag", maxAccMag);
    validatePositive("maxBdrMag", maxBdrMag);
    validatePositive("MaxTiltAngle", maxTiltAngle);
    validatePositive("MinThrust", minThrust);
    validatePositive("MaxThrust", maxThrust);
    validatePositive("VehicleMass", vehicleMass);
    validatePositive("GravAcc", gravAcc);
    validatePositive("SpeedEps", speedEps);
    validatePositive("WeightT", weightT);
    validatePositive("WeightSafeT", WeightSafeT);
    validatePositive("SmoothingEps", smoothingEps);
    validatePositive("RelCostTol", relCostTol);
    validatePositive("MaxCorridorSize", corridor_size);
    validatePositive("yaw_max_vel", yaw_max_vel);
    validatePositive("numerical/yaw_diff_eps", yawDiffEps);
    validatePositive("numerical/trig_gradient_eps", trigGradientEps);
    validatePositive("numerical/vector_norm_eps", vectorNormEps);
    validatePositive("numerical/min_segment_time", minSegmentTime);
    validatePositive("numerical/flatness_norm_eps", flatnessNormEps);
    validatePositive("numerical/min_path_segment_length",
                     minPathSegmentLength);
    validatePositive("numerical/linear_solve_pivot_eps",
                     linearSolvePivotEps);
    validatePositive("local_planning/corridor/obstacle_voxel_size",
                     corridorObstacleVoxelSize);
    validatePositive("local_planning/corridor/search_margin_xy",
                     corridorSearchMarginXY);
    validatePositive("local_planning/corridor/search_margin_z",
                     corridorSearchMarginZ);
    validatePositive("local_planning/corridor/progress_length",
                     corridorProgressLength);
    validatePositive("local_planning/corridor/firi_obstacle_distance_limit",
                     firiObstacleDistanceLimit);
    validatePositive("local_planning/corridor/overlap_tolerance",
                     corridorOverlapTolerance);
    validatePositive("local_planning/corridor/fallback_min_progress",
                     corridorFallbackMinProgress);
    validateFinite("local_planning/corridor/fallback_target_margin",
                   corridorFallbackTargetMargin);
    validateFinite("HorizDrag", horizDrag);
    validateFinite("VertDrag", vertDrag);
    validateFinite("ParasDrag", parasDrag);
    validateFinite("yaw_rho_vis", yaw_rho_vis);
    validateFinite("yaw_time_fwd", yaw_time_fwd);
    if (maxThrust <= minThrust || yaw_rho_vis < 0.0 || yaw_time_fwd < 0.0 ||
        chiVec.size() != 5 || pvaRunningWeights.size() != 3 ||
        pvaInitialWeights.size() != 2 || pvaTerminalWeights.size() != 3 ||
        !std::all_of(chiVec.begin(), chiVec.end(),
                     [](double value) {
                       return std::isfinite(value) && value >= 0.0;
                     }) ||
        !std::all_of(pvaRunningWeights.begin(), pvaRunningWeights.end(),
                     [](double value) {
                       return std::isfinite(value) && value >= 0.0;
                     }) ||
        !std::all_of(pvaInitialWeights.begin(), pvaInitialWeights.end(),
                     [](double value) {
                       return std::isfinite(value) && value >= 0.0;
                     }) ||
        !std::all_of(pvaTerminalWeights.begin(), pvaTerminalWeights.end(),
                     [](double value) {
                       return std::isfinite(value) && value >= 0.0;
                     })) {
      ROS_FATAL("[NumericalGuard] invalid MINCO/Yaw parameter relationship");
      throw std::runtime_error("invalid MINCO/Yaw parameter relationship");
    }
    if (integralIntervs <= 0) {
      ROS_FATAL("[NumericalGuard] IntegralIntervs must be greater than zero");
      throw std::runtime_error("invalid parameter: IntegralIntervs");
    }
    if (corridorMaxObstacleSamples <= 0 || firiMaxPlaneCount < 6 ||
        corridorGapViolationPlaneThreshold <= 0) {
      ROS_FATAL("[CorridorConfig] sample count and gap threshold must be "
                "positive, and firi_max_plane_count must be at least 6");
      throw std::runtime_error("invalid Corridor/FIRI integer parameter");
    }
    if (corridorFallbackTargetMargin < 0.0) {
      ROS_FATAL("[CorridorConfig] fallback_target_margin must be non-negative");
      throw std::runtime_error(
          "invalid parameter: local_planning/corridor/fallback_target_margin");
    }
  }
};

class FastPlannerManager {
  // SECTION stable
public:
  typedef shared_ptr<FastPlannerManager> Ptr;
  FastPlannerManager();
  ~FastPlannerManager();
  void printTimeCost(double time_threhold, double time_cost, string printInfo);

  bool planExploreTraj(const vector<Eigen::Vector3f> &path, bool is_static);
  bool flyToSafeRegion(bool is_static);
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time);
  void polyYawTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time);

  void initPlanModules(ros::NodeHandle &nh, ParallelBubbleAstar::Ptr &parallel_path_finder,
                       TopoGraph::Ptr &graph);

  bool checkTrajCollision(double &collision_time);
  bool checkTrajVelocity();

  bool YawTrajOpt(const Eigen::Vector3d &initial_yaw_state, double &end_yaw,
                  bool use_shorten_path);
  bool YawTrajwithoutOpt(double &start_yaw, double &end_yaw, bool is_static, bool use_shorten_path);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void posCallback(const nav_msgs::OdometryConstPtr &msg);
  bool YawInterpolationwithoutOpt(double &start, double &end, vector<double> &newYaw,
                                  vector<double> &newDur, double &CompT);
  void YawLookforward(const Trajectory<5> &pos_traj, double &start, double &end,
                      vector<double> &newYaw, vector<double> &newDur, double &CompT);
  void YawLookforwardwithoutOpt(double &start, double &end, vector<double> &newYaw,
                                vector<double> &newDur, double &CompT, bool use_short_path);
  void angleLimite(double &angle);

  double start_yaw, end_yaw;
  double is_static_yaw = false;

  ros::Subscriber goal_sub;
  ros::Subscriber pos_sub;
  ros::Publisher yaw_state_pub;

  // Simulation-only planner evidence. The publishers remain disabled unless
  // planner_debug/enable is explicitly set by sim_bringup.
  bool planner_debug_enabled_ = false;
  uint64_t planner_debug_seq_ = 0;
  uint64_t current_debug_plan_seq_ = 0;
  ros::Publisher debug_obstacle_points_pub_;
  ros::Publisher debug_guide_path_pub_;
  ros::Publisher debug_raw_hpolys_pub_;
  ros::Publisher debug_clipped_hpolys_pub_;
  ros::Publisher debug_fov_faces_pub_;
  ros::Publisher debug_traj_clearance_pub_;

  minco::MINCO_S3NU yaw_traj_opt_;
  LocalTrajData local_data_;
  // 마지막 로컬 궤적 계획 실패 사유 (성공 시 clear). FSM 이벤트 로거가 읽어
  // "경로가 왜 안 나왔는지"를 이벤트/HUD에 표기하는 데 쓴다.
  std::string last_plan_fail_reason_;
  // 직전 "성공"이 요청 경로가 아니라 flyToSafeRegion 탈출 궤적이었는지
  bool last_plan_was_escape_ = false;
  double max_traj_len_;
  LIOInterface::Ptr lidar_map_interface_;
  unique_ptr<Visualizer> gcopter_viz_;
  unique_ptr<GcopterConfig> gcopter_config_;
  BubbleAstar::Ptr bubble_path_finder_;
  ParallelBubbleAstar::Ptr parallel_path_finder_;
  TopoGraph::Ptr topo_graph_;
  GraphVisualizer::Ptr graph_visualizer_;
  FastSearcher::Ptr fast_searcher_;
  bool use_mid360;
  double max_ray_length;
  double fov_up, fov_down;
  double lidar_pitch;

  // ---- Local SFC corridor clipping to the observed FOV cone (forward-FOV) ----
  // Handle to the frontier manager (wired in FastExplorationManager::initialize).
  // When enabled, the local corridor is clipped so it cannot extend past what a
  // limited-FOV sensor has actually observed.
  shared_ptr<FrontierManager> frontier_manager_;
  bool clip_corridor_to_observed_ = false; // master switch (default off = legacy)
  bool clip_cone_faces_ = false;           // Method B: FOV cone half-plane clipping
  double p0_len_x_ = 0.6; // robot free box P0: body-x (fwd/back) full length
  double p0_len_y_ = 0.6; // P0: body-y (left/right) full length
  double p0_up_ = 0.2;    // P0: extent above the flight controller
  double p0_down_ = 0.2;  // P0: extent below the flight controller
  bool viz_origin_corridor_ = false; // publish raw FIRI corridor for comparison
  double yaw_fov_ = 2.0 * M_PI; // horizontal FOV [rad] (lidar_perception/yaw_fov)
  // Immutable snapshot of the PR #1 observed-boundary decision for one planning
  // cycle. A candidate face is active only when an FOV-edge frontier lies on it
  // and no DENSE cell continues outside it.
  struct ObservedBoundaryFace {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double offset = 0.0;
    bool has_frontier = false;
    bool observed_outside = false;
    bool active = false;
  };

  struct ObservedBoundary {
    Eigen::Vector3d apex = Eigen::Vector3d::Zero();
    double max_range = 0.0;
    std::vector<ObservedBoundaryFace> faces;
  };

  // Build the PR #1 boundary once so FIRI generation and post-clipping consume
  // the exact same active half-spaces.
  ObservedBoundary buildObservedBoundary() const;
  Eigen::MatrixX4d activeObservedHalfspaces(
      const ObservedBoundary &boundary) const;
  void publishObservedBoundaryDebug(const ObservedBoundary &boundary);
  void clipCorridorToObservedBoundary(
      std::vector<Eigen::MatrixX4d> &hPolys,
      const ObservedBoundary &boundary);

  void publishDebugGuidePath(const vector<Eigen::Vector3d> &path);
  void publishDebugObstaclePoints(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
  void publishDebugHPolys(const std::vector<Eigen::MatrixX4d> &hPolys,
                          ros::Publisher &publisher);
  void publishDebugTrajectoryClearance(bool safe, double collision_time,
                                       double min_distance,
                                       const Eigen::Vector3d &sample_position);

private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> sdf_map_;

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d> &colli_start, vector<Eigen::Vector3d> &colli_end,
                          vector<Eigen::Vector3d> &start_pts, vector<Eigen::Vector3d> &end_pts);

  Eigen::MatrixXd paramLocalTraj(double start_t, double &dt, double &duration);
  Eigen::MatrixXd reparamLocalTraj(const double &start_t, const double &duration, const double &dt);

public:
  void planYawActMap(const Eigen::Vector3d &start_yaw);
  void test();
  void searchFrontier(const Eigen::Vector3d &p);

private:
  // Benchmark method, local exploration
public:
  bool localExplore(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                    Eigen::Vector3d end_pt);

  // !SECTION
};
} // namespace fast_planner

#endif
