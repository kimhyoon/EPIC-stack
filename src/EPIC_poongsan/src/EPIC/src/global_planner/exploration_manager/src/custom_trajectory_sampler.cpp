#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <traj_utils/PolyTraj.h>
#include <gcopter/trajectory.hpp>

class TrajectoryDiscreteSampler {
private:
    ros::NodeHandle nh_;
    ros::Subscriber traj_sub_;
    
    // Publishers for different sampling rates
    ros::Publisher discrete_path_pub_;        // nav_msgs/Path
    ros::Publisher discrete_points_pub_;      // visualization_msgs/MarkerArray
    ros::Publisher multidof_traj_pub_;        // trajectory_msgs/MultiDOFJointTrajectory
    
    std::shared_ptr<Trajectory<7>> current_traj_;
    std::shared_ptr<Trajectory<5>> current_yaw_traj_;
    
    // Sampling parameters (configurable via ROS parameters)
    double sample_interval_;    // T seconds (sampling interval)
    double total_duration_;     // N seconds (total sampling duration)
    bool auto_duration_;        // Use trajectory's full duration if true
    
public:
    TrajectoryDiscreteSampler() : nh_("~") {
        // Load parameters
        nh_.param("sample_interval", sample_interval_, 0.1);   // Default: 0.1s (10Hz)
        nh_.param("total_duration", total_duration_, 10.0);    // Default: 10s
        nh_.param("auto_duration", auto_duration_, true);      // Use full traj duration
        
        // Publishers
        discrete_path_pub_ = nh_.advertise<nav_msgs::Path>("/planning/trajectory_discrete", 10);
        discrete_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planning/trajectory_waypoints", 10);
        multidof_traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/planning/trajectory_multidof", 10);

        
        // Subscribers
        traj_sub_ = nh_.subscribe("/planning/trajectory", 10, &TrajectoryDiscreteSampler::trajectoryCallback, this);
        
        ROS_INFO("[Trajectory Sampler] Initialized with:");
        ROS_INFO("  - Sample interval: %.3f seconds", sample_interval_);
        ROS_INFO("  - Total duration: %.3f seconds", total_duration_);
        ROS_INFO("  - Auto duration: %s", auto_duration_ ? "true" : "false");
        ROS_INFO("  - Publishing to 3 topics:");
        ROS_INFO("    * /planning/trajectory_discrete (nav_msgs/Path)");
        ROS_INFO("    * /planning/trajectory_waypoints (visualization_msgs/MarkerArray)");
        ROS_INFO("    * /planning/trajectory_multidof (trajectory_msgs/MultiDOFJointTrajectory)");
    }
    
    void trajectoryCallback(const traj_utils::PolyTrajPtr& msg) {
        if (msg->order != 7) {
            ROS_WARN("[Trajectory Sampler] Only support 7th order trajectory!");
            return;
        }
        
        // Parse trajectory
        int piece_nums = msg->duration.size();
        std::vector<double> dura(piece_nums);
        std::vector<Piece<7>::CoefficientMat> cMats(piece_nums);
        
        for (int i = 0; i < piece_nums; ++i) {
            int i8 = i * 8;
            cMats[i].row(0) << msg->coef_x[i8+0], msg->coef_x[i8+1], msg->coef_x[i8+2], 
                               msg->coef_x[i8+3], msg->coef_x[i8+4], msg->coef_x[i8+5], 
                               msg->coef_x[i8+6], msg->coef_x[i8+7];
            cMats[i].row(1) << msg->coef_y[i8+0], msg->coef_y[i8+1], msg->coef_y[i8+2], 
                               msg->coef_y[i8+3], msg->coef_y[i8+4], msg->coef_y[i8+5], 
                               msg->coef_y[i8+6], msg->coef_y[i8+7];
            cMats[i].row(2) << msg->coef_z[i8+0], msg->coef_z[i8+1], msg->coef_z[i8+2], 
                               msg->coef_z[i8+3], msg->coef_z[i8+4], msg->coef_z[i8+5], 
                               msg->coef_z[i8+6], msg->coef_z[i8+7];
            dura[i] = msg->duration[i];
        }
        
        current_traj_.reset(new Trajectory<7>(dura, cMats));
        
        // Sample and publish
        sampleAndPublish(msg->start_time);
    }
    
    void sampleAndPublish(const ros::Time& start_time) {
        if (!current_traj_) return;
        
        double traj_duration = current_traj_->getTotalDuration();
        double sampling_duration = auto_duration_ ? traj_duration : std::min(total_duration_, traj_duration);
        
        // Prepare messages
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "odom";
        
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker points_marker;
        points_marker.header = path_msg.header;
        points_marker.ns = "discrete_trajectory";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.color.r = 0.0; points_marker.color.g = 1.0; points_marker.color.b = 0.0; points_marker.color.a = 1.0;
        points_marker.scale.x = 0.1; points_marker.scale.y = 0.1; points_marker.scale.z = 0.1;
        
        // Prepare MultiDOFJointTrajectory message
        trajectory_msgs::MultiDOFJointTrajectory multidof_msg;
        multidof_msg.header = path_msg.header;
        multidof_msg.joint_names.push_back("base_link"); // Joint name for the UAV base
        
        int sample_count = 0;
        
        // Sample at T-second intervals for N seconds
        for (double t = 0.0; t <= sampling_duration; t += sample_interval_) {
            if (t > traj_duration) break;
            
            // Get position, velocity, acceleration
            Eigen::Vector3d pos = current_traj_->getPos(t);
            Eigen::Vector3d vel = current_traj_->getVel(t);
            Eigen::Vector3d acc = current_traj_->getAcc(t);
            
            // Create PoseStamped for path
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.header.stamp = start_time + ros::Duration(t);
            pose_stamped.pose.position.x = pos(0);
            pose_stamped.pose.position.y = pos(1);
            pose_stamped.pose.position.z = pos(2);
            
            // Calculate yaw from velocity (optional)
            double yaw = 0.0;
            if (vel.head<2>().norm() > 0.1) {
                yaw = atan2(vel(1), vel(0));
                pose_stamped.pose.orientation.z = sin(yaw * 0.5);
                pose_stamped.pose.orientation.w = cos(yaw * 0.5);
            } else {
                pose_stamped.pose.orientation.w = 1.0;
            }
            
            path_msg.poses.push_back(pose_stamped);
            
            // Create point for visualization
            geometry_msgs::Point point;
            point.x = pos(0);
            point.y = pos(1);
            point.z = pos(2);
            points_marker.points.push_back(point);
            
            // Create MultiDOFJointTrajectoryPoint
            trajectory_msgs::MultiDOFJointTrajectoryPoint multidof_point;
            multidof_point.time_from_start = ros::Duration(t);
            
            // Transform (position + orientation)
            geometry_msgs::Transform transform;
            transform.translation.x = pos(0);
            transform.translation.y = pos(1);
            transform.translation.z = pos(2);
            transform.rotation.x = 0.0;
            transform.rotation.y = 0.0;
            transform.rotation.z = sin(yaw * 0.5);
            transform.rotation.w = cos(yaw * 0.5);
            multidof_point.transforms.push_back(transform);
            
            // Velocity (linear + angular)
            geometry_msgs::Twist velocity;
            velocity.linear.x = vel(0);
            velocity.linear.y = vel(1);
            velocity.linear.z = vel(2);
            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0; // Can be computed from velocity direction change if needed
            multidof_point.velocities.push_back(velocity);
            
            // Acceleration (linear + angular)
            geometry_msgs::Twist acceleration;
            acceleration.linear.x = acc(0);
            acceleration.linear.y = acc(1);
            acceleration.linear.z = acc(2);
            acceleration.angular.x = 0.0;
            acceleration.angular.y = 0.0;
            acceleration.angular.z = 0.0; // Can be computed from acceleration direction change if needed
            multidof_point.accelerations.push_back(acceleration);
            
            multidof_msg.points.push_back(multidof_point);
            
            sample_count++;
        }
        
        marker_array.markers.push_back(points_marker);
        
        // Publish all messages
        discrete_path_pub_.publish(path_msg);
        discrete_points_pub_.publish(marker_array);
        multidof_traj_pub_.publish(multidof_msg);
        
        // ROS_INFO("[Trajectory Sampler] Sampled %d points at %.3fs intervals over %.3fs duration", 
        //          sample_count, sample_interval_, sampling_duration);
        // ROS_INFO("[Trajectory Sampler] Published to 3 topics: trajectory_discrete, trajectory_waypoints, trajectory_multidof");
    }
    
    // Dynamic reconfigure function (can be called via service)
    void updateSamplingParams(double interval, double duration, bool auto_dur) {
        sample_interval_ = interval;
        total_duration_ = duration;
        auto_duration_ = auto_dur;
        
        // ROS_INFO("[Trajectory Sampler] Updated parameters:");
        // ROS_INFO("  - Sample interval: %.3f seconds", sample_interval_);
        // ROS_INFO("  - Total duration: %.3f seconds", total_duration_);
        // ROS_INFO("  - Auto duration: %s", auto_duration_ ? "true" : "false");
        
        // Re-sample current trajectory if available
        if (current_traj_) {
            sampleAndPublish(ros::Time::now());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_discrete_sampler");
    
    TrajectoryDiscreteSampler sampler;
    
    ROS_INFO("[Trajectory Sampler] Node started. Waiting for trajectory...");
    
    ros::spin();
    
    return 0;
} 