#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// ── 라이다 입력 타입 (CMakeLists 의 USE_LIVOX_LIDAR 옵션으로 선택) ──────────
//  정의됨   : livox_ros_driver2/CustomMsg 구독 (Mid360 등 livox 드라이버 직결)
//  정의 안됨: sensor_msgs/PointCloud2 구독 (일반 라이다; livox 의존성 없이 빌드)
//  ※ 어느 쪽이든 포인트는 "센서/바디 프레임" 이어야 한다 (밴드 게이팅이 바디 z/r 기준).
#ifdef USE_LIVOX_LIDAR
#include <livox_ros_driver2/CustomMsg.h>
#endif
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>

#include <math.h>
#include <string>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


double current_yaw;
double avoidance_enable =0;
mavros_msgs::PositionTarget pose_target_;
geometry_msgs::Pose current_pose;
tf::Matrix3x3 R_body_to_map;                       // body -> map (local) full rotation
pcl::PointCloud<pcl::PointXYZ> cloud_data;          // latest 3D point cloud (sensor/body frame)
bool odom_received = false;
geometry_msgs::PoseStamped visualization_pose;
std::string odom_frame_id = "map";                 // frame the odom (and thus the target) lives in
std::string viz_frame_ = "world";                  // RViz fixed frame for the avoidance-direction arrow
std_msgs::Int16 FSM_flag;
ros::Publisher debug_pub;                           // /local_avoidance/debug_points (force-producing points)
pcl::PointCloud<pcl::PointXYZI> debug_cloud;        // intensity: 1=xy-ring 2=z-pillar (+2 if within emergency r)
double repulsive_m, repulsive_gain, lidar_min_threshold, avoidance_trigger_m, emergency_avoidance_m, avoidance_moving_m;
double band_z_thr, band_r_thr;   // two-band gating (small-tilt assumption, raw sensor frame)



void odom_cb(const nav_msgs::OdometryConstPtr& msg){

    current_pose = msg->pose.pose;
    odom_frame_id = msg->header.frame_id;           // target is expressed in the odom's frame
    tf::Quaternion q(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    R_body_to_map = m;                              // keep full rotation for body->map transform
    odom_received = true;
}



void local_avoidance(double min_distance){
    ros::param::get("/local_avoidance/repulsive_m", repulsive_m);
    ros::param::get("/local_avoidance/repulsive_gain", repulsive_gain);
    ros::param::get("/local_avoidance/emergency_avoidance_m", emergency_avoidance_m);
    ros::param::get("/local_avoidance/avoidance_moving_m", avoidance_moving_m);
    ros::param::get("/local_avoidance/band_z_thr", band_z_thr);
    ros::param::get("/local_avoidance/band_r_thr", band_r_thr);

	float avoidance_vector_x = 0;   // horizontal (from slab)
	float avoidance_vector_y = 0;
	float avoidance_vector_z = 0;   // vertical   (from column)
	bool avoid = true;
    bool final_avoidance_activate = (min_distance <= emergency_avoidance_m);
    double emergency_boost_range = emergency_avoidance_m*sqrt(2)+0.05;

    for(size_t i=0; i<cloud_data.points.size(); i++)
	{
        const pcl::PointXYZ& p = cloud_data.points[i];
        if(!pcl::isFinite(p)) continue;

        double x = p.x, y = p.y, z = p.z;
        double dist = sqrt(x*x + y*y + z*z);
        if(dist <= lidar_min_threshold) continue;   // noise cut / self
        double rh = sqrt(x*x + y*y);                 // horizontal distance (sensor frame)

        // ---- horizontal slab ( |z| < band_z_thr )  ->  x,y avoidance ----
        if(fabs(z) < band_z_thr && rh > 1e-3 && rh < repulsive_m){
            float U = -0.5*repulsive_gain*pow(((1/rh) - (1/repulsive_m)), 2);
            if(final_avoidance_activate && rh <= emergency_boost_range){
                U = 5*U;
            }
            avoidance_vector_x = avoidance_vector_x + (x/rh)*U;
            avoidance_vector_y = avoidance_vector_y + (y/rh)*U;
        }

        // ---- vertical column ( hypot(x,y) < band_r_thr )  ->  z avoidance ----
        if(rh < band_r_thr){
            double dv = fabs(z);                     // vertical distance to ceiling/floor
            if(dv > lidar_min_threshold && dv < repulsive_m){
                float U = -0.5*repulsive_gain*pow(((1/dv) - (1/repulsive_m)), 2);
                if(final_avoidance_activate && dv <= emergency_boost_range){
                    U = 5*U;
                }
                avoidance_vector_z = avoidance_vector_z + (z/dv)*U;   // z/dv = +-1
            }
        }
	}

    // ---- clamp total (horizontal + vertical) move distance (before rotation) ----
    double cal_move = sqrt(avoidance_vector_x*avoidance_vector_x + avoidance_vector_y*avoidance_vector_y + avoidance_vector_z*avoidance_vector_z);
    if(cal_move > avoidance_moving_m){
        avoidance_vector_x = avoidance_moving_m * (avoidance_vector_x/cal_move);
        avoidance_vector_y = avoidance_moving_m * (avoidance_vector_y/cal_move);
        avoidance_vector_z = avoidance_moving_m * (avoidance_vector_z/cal_move);
    }

    // Transform from Body frame to Local(map) frame using the full rotation matrix
    tf::Vector3 v_body(avoidance_vector_x, avoidance_vector_y, avoidance_vector_z);
    tf::Vector3 v_map = R_body_to_map * v_body;

	if(avoid)
	{
        pose_target_.header.stamp = ros::Time::now();
        pose_target_.header.frame_id = odom_frame_id;
        pose_target_.coordinate_frame = 1;
        pose_target_.position.x = v_map.x() + current_pose.position.x;
        pose_target_.position.y = v_map.y() + current_pose.position.y;
	    pose_target_.position.z = v_map.z() + current_pose.position.z;   // z avoidance from column band only
	    pose_target_.yaw=current_yaw;
        pose_target_.type_mask = 3064;
        avoidance_enable = true;
	}
}


void visualization_avoidance(mavros_msgs::PositionTarget& msg)
{
    // Anchor coords come from /mavros/local_position/odom (numerically the odom
    // frame), but RViz's fixed frame is the EPIC world frame. Since EV feeds
    // FAST-LIO into the PX4 EKF, odom and world share the same origin/axes, so
    // we just label the arrow with the world frame to make it render in RViz.
    visualization_pose.header.frame_id = viz_frame_;
    visualization_pose.header.stamp=msg.header.stamp;
    // anchor the arrow at the robot and point it along the actual escape direction
    // (target - current), so it shows the avoidance direction (away from obstacle),
    // not the drone heading.
    visualization_pose.pose.position = current_pose.position;
    double dx = msg.position.x - current_pose.position.x;
    double dy = msg.position.y - current_pose.position.y;
    double target_yaw = atan2(dy, dx);
    tf::Quaternion quat;
    quat.setRPY(0,0,target_yaw);
    quat.normalize();
    visualization_pose.pose.orientation.x = quat[0];
    visualization_pose.pose.orientation.y = quat[1];
    visualization_pose.pose.orientation.z = quat[2];
    visualization_pose.pose.orientation.w = quat[3];
}

// 메시지 타입 무관 공통 처리: cloud_data(센서 프레임) 채워진 뒤 호출된다.
// sensor_header 는 디버그 클라우드 발행용 (frame_id/stamp).
void processLidarCloud(const std_msgs::Header &sensor_header){
    ros::param::get("/local_avoidance/avoidance_trigger_m", avoidance_trigger_m);
    ros::param::get("/local_avoidance/lidar_min_threshold", lidar_min_threshold);
    ros::param::get("/local_avoidance/band_z_thr", band_z_thr);
    ros::param::get("/local_avoidance/band_r_thr", band_r_thr);
    ros::param::get("/local_avoidance/repulsive_m", repulsive_m);
    ros::param::get("/local_avoidance/emergency_avoidance_m", emergency_avoidance_m);
    debug_cloud.clear();

    if (cloud_data.points.size() < 1){
        return;
    }
    int minIndex = 0;
    double minval = 999;
    for(size_t i = 0; i < cloud_data.points.size(); i++){
        const pcl::PointXYZ& p = cloud_data.points[i];
        if(!pcl::isFinite(p)) continue;

        double dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (dist <= lidar_min_threshold){
            continue;
        }
        double rh = sqrt(p.x*p.x + p.y*p.y);
        bool in_slab = (fabs(p.z) < band_z_thr);     // horizontal band
        bool in_col  = (rh < band_r_thr);            // vertical band

        // ---- debug: collect the points that actually generate a repulsive force ----
        //   intensity 1 = xy-ring (slab) , 2 = z-pillar (column) , +2 if inside emergency radius
        bool ring   = (in_slab && rh > 1e-3 && rh < repulsive_m);
        bool pillar = (in_col  && fabs(p.z) > lidar_min_threshold && fabs(p.z) < repulsive_m);
        if(ring || pillar){
            pcl::PointXYZI dp;
            dp.x = p.x; dp.y = p.y; dp.z = p.z;
            double pd = ring ? rh : fabs(p.z);        // effective distance for this point
            dp.intensity = (ring ? 1.0f : 2.0f) + (pd <= emergency_avoidance_m ? 2.0f : 0.0f);
            debug_cloud.points.push_back(dp);
        }

        if(!in_slab && !in_col) continue;            // outside both bands -> ignore

        double d_eff = 1e9;                           // effective distance for trigger
        if(in_slab) d_eff = rh;
        if(in_col && fabs(p.z) < d_eff) d_eff = fabs(p.z);

        if (d_eff < minval){
           minval = d_eff;
           minIndex = i;
        }
    }
//    ROS_INFO("LIDAR_MIN_THRES: %f",lidar_min_threshold);

    // Only log on avoidance state changes (and throttle the active distance),
    // so the terminal isn't spammed at the 40 Hz lidar rate when nothing's near.
    static bool was_avoiding = false;
    if(minval < avoidance_trigger_m){
        avoidance_enable = true;
        if(!was_avoiding)
            ROS_WARN("[local_avoidance] ACTIVATED  (min_dist=%.2f m, idx=%d)", minval, minIndex);
        else
            ROS_INFO_THROTTLE(1.0, "[local_avoidance] avoiding...  min_dist=%.2f m", minval);
    }else{
        avoidance_enable = false;
        if(was_avoiding)
            ROS_INFO("[local_avoidance] DEACTIVATED");
    }
    was_avoiding = (avoidance_enable != 0);

    if (avoidance_enable){
	    local_avoidance(minval);
        visualization_avoidance(pose_target_);
    }

    // publish the force-producing points (sensor frame) for RViz debugging
    sensor_msgs::PointCloud2 debug_msg;
    debug_cloud.width  = debug_cloud.points.size();
    debug_cloud.height = 1;
    debug_cloud.is_dense = false;
    pcl::toROSMsg(debug_cloud, debug_msg);
    debug_msg.header.frame_id = sensor_header.frame_id;  // 센서 프레임
    debug_msg.header.stamp    = sensor_header.stamp;
    debug_pub.publish(debug_msg);
}

#ifdef USE_LIVOX_LIDAR
// livox 드라이버 직결: CustomMsg(xfer_format=1, FAST-LIO 와 같은 스트림)에서
// CustomPoint 리스트를 직접 PCL 클라우드로 옮긴다.
void lidarCallback(const livox_ros_driver2::CustomMsg::ConstPtr &msg){
    cloud_data.clear();
    cloud_data.points.reserve(msg->point_num);
    for(uint32_t i = 0; i < msg->point_num; i++){
        pcl::PointXYZ p;
        p.x = msg->points[i].x;
        p.y = msg->points[i].y;
        p.z = msg->points[i].z;
        cloud_data.points.push_back(p);
    }
    processLidarCloud(msg->header);
}
#else
// 일반 라이다: sensor_msgs/PointCloud2 (센서/바디 프레임이어야 함!
// world 프레임 클라우드(/cloud_registered 등)를 물리면 밴드 게이팅이 틀어진다).
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    cloud_data.clear();
    pcl::fromROSMsg(*msg, cloud_data);
    processLidarCloud(msg->header);
}
#endif




int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_avoidance");
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);
    avoidance_enable=0;
    R_body_to_map.setIdentity();

    // Only set a default if the param is not already provided (e.g. by launch/yaml),
    // so launch-configured values are not clobbered on startup.
    auto set_default = [](const std::string& key, double val){
        if(!ros::param::has(key)) ros::param::set(key, val);
    };
    set_default("/local_avoidance/repulsive_m",          2.0);
    set_default("/local_avoidance/repulsive_gain",       0.15);
    set_default("/local_avoidance/avoidance_moving_m",   1.45);
    set_default("/local_avoidance/emergency_avoidance_m",0.95);
    set_default("/local_avoidance/avoidance_trigger_m",  1.45);
    set_default("/local_avoidance/lidar_min_threshold",  0.4);
    set_default("/local_avoidance/band_z_thr",           0.3);   // horizontal slab half-thickness
    set_default("/local_avoidance/band_r_thr",           0.3);   // vertical column radius

    // RViz fixed frame for the avoidance-direction arrow (defaults to the EPIC world frame).
    ros::param::param<std::string>("/local_avoidance/viz_frame", viz_frame_, std::string("world"));


#ifdef USE_LIVOX_LIDAR
    ros::Subscriber lidar_sub = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar",1,lidarCallback);
    ROS_INFO("[local_avoidance] lidar input: livox_ros_driver2/CustomMsg on /livox/lidar (remap-able)");
#else
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar",1,lidarCallback);
    ROS_INFO("[local_avoidance] lidar input: sensor_msgs/PointCloud2 on /livox/lidar (remap-able; "
             "cloud must be in SENSOR/BODY frame)");
#endif
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",30, odom_cb);
    ros::Publisher position_target_pub= nh.advertise<mavros_msgs::PositionTarget>("/target_avoidance", 30);
    ros::Publisher FMS_flg_pub= nh.advertise<std_msgs::Int16>("/FSM_flag_avoidance", 30);
    ros::Publisher visualization_pub= nh.advertise<geometry_msgs::PoseStamped>("/local_avoidance_visualization", 30);
    debug_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_avoidance/debug_points", 1);

    int param_update_inteval = 50;

    // 마스터 스위치: real.yaml 의 local_avoidance/enable (exploration_node ns 에 로드됨).
    // false 면 회피 목표/시각화를 내보내지 않고 flag=0 만 발행 (완전 비활성).
    // 단독 실행(EPIC 없이) 시엔 /local_avoidance/enable (자체 yaml) -> 기본 true.
    auto read_master_enable = []() {
        bool en = true;
        if (!ros::param::get("/exploration_node/local_avoidance/enable", en))
            ros::param::param("/local_avoidance/enable", en, true);
        return en;
    };
    bool master_enable = read_master_enable();
    ROS_INFO("[local_avoidance] master enable = %s (from real.yaml local_avoidance/enable)",
             master_enable ? "true" : "false");
    int loop_cnt = 0;

    while (ros::ok()){
        // 파라미터는 비행 중에도 바꿀 수 있게 ~2s 마다 재확인
        if (++loop_cnt % 80 == 0) {
            bool prev = master_enable;
            master_enable = read_master_enable();
            if (prev != master_enable)
                ROS_WARN("[local_avoidance] master enable changed -> %s",
                         master_enable ? "true" : "false");
        }

        if (master_enable && avoidance_enable){
            position_target_pub.publish(pose_target_);
            FSM_flag.data=1;
            visualization_pub.publish(visualization_pose);
        }
        else{
            FSM_flag.data=0;
            //std::cout << std::stod(d0_in_code) << std::endl;
        }
        FMS_flg_pub.publish(FSM_flag);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
