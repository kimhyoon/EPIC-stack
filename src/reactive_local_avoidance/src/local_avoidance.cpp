#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// ── 라이다 입력 타입 (CMakeLists 의 USE_LIVOX_LIDAR 옵션으로 선택) ──────────
//  정의됨   : livox_ros_driver2/CustomMsg 구독 (Mid360 등 livox 드라이버 직결)
//  정의 안됨: sensor_msgs/PointCloud2 구독 (일반 라이다; livox 의존성 없이 빌드)
//  ※ 어느 쪽이든 입력은 "센서 프레임" 이어야 하며, 수신 즉시 장착각(sensor_pitch_deg)
//    회전을 적용해 바디 프레임으로 만든 뒤 사용한다 (밴드 게이팅이 바디 z/r 기준).
#ifdef USE_LIVOX_LIDAR
#include <livox_ros_driver2/CustomMsg.h>
#endif
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>

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
pcl::PointCloud<pcl::PointXYZ> cloud_data;          // latest 3D point cloud (BODY frame; 수신 시 장착각 회전 적용)
bool odom_received = false;
visualization_msgs::Marker visualization_marker;    // 타겟 명령 화살표 (미발동 시 DELETE/lifetime 로 제거)
std::string odom_frame_id = "map";                 // frame the odom (and thus the target) lives in
std::string viz_frame_ = "world";                  // RViz fixed frame for the avoidance-direction arrow
std::string body_frame_id = "";                    // 바디 프레임 (odom child_frame_id 에서 latch)
std_msgs::Int16 FSM_flag;
ros::Publisher debug_pub;                           // /local_avoidance/debug_points (force-producing points)
pcl::PointCloud<pcl::PointXYZI> debug_cloud;        // intensity: 1=xy-ring 2=z-pillar (+2 if within trigger = 발동 유발점)
double repulsive_m, repulsive_gain, lidar_min_threshold, avoidance_trigger_m, emergency_avoidance_m, avoidance_moving_m;
double band_z_thr, band_r_thr;   // two-band gating (body frame; 센서 장착 기울기는 수신 시 보정)
double avoidance_release_delay_s = 0.4;  // 해제 지연 [s] (발동은 즉시; 스캔 드롭아웃 채터링 방지)
bool master_enable = true;       // 프로파일 yaml local_avoidance/enable (메인루프가 ~2s 마다 갱신)

// ── 파라미터 단일 소스 ───────────────────────────────────────────────────────
//  EPIC 프로파일 yaml(mid360.yaml 등, real_flight.launch 가 /exploration_node ns 에
//  로드)의 local_avoidance/<key> 가 있으면 우선하고, 없으면 자체
//  config/local_avoidance.yaml(/local_avoidance ns) 값을 쓴다.
//  -> 실비행 튜닝은 프로파일 yaml 한 곳에서. 단독 실행 시엔 자체 yaml 그대로.
template <typename T>
void getAvoidParam(const std::string& key, T& out){
    if(!ros::param::get("/exploration_node/local_avoidance/" + key, out))
        ros::param::get("/local_avoidance/" + key, out);
}

// ── 센서 장착각 보정 (sensor -> body) ────────────────────────────────────────
//  sensor_pitch_deg: 센서를 아래로 숙여 단 각도 (deg, 아래로 숙임 = 양수).
//  예) Mid360 를 전방 25° 하향 장착 -> 25.0
//  y축(pitch) 회전만 지원 (roll/yaw 장착 오프셋은 없다고 가정). y 는 불변:
//    x_b =  c*x_s + s*z_s
//    z_b = -s*x_s + c*z_s
double sensor_pitch_deg = 0.0;
double mount_c = 1.0, mount_s = 0.0;               // cos/sin (콜백마다 갱신, 비행 전 튜닝용)

inline void updateMountRotation(){
    getAvoidParam("sensor_pitch_deg", sensor_pitch_deg);
    double th = sensor_pitch_deg * M_PI / 180.0;
    mount_c = cos(th);
    mount_s = sin(th);
}



void odom_cb(const nav_msgs::OdometryConstPtr& msg){

    current_pose = msg->pose.pose;
    odom_frame_id = msg->header.frame_id;           // target is expressed in the odom's frame
    if(body_frame_id.empty() && !msg->child_frame_id.empty())
        body_frame_id = msg->child_frame_id;        // 바디 프레임 이름 latch (debug 클라우드용)
    tf::Quaternion q(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    R_body_to_map = m;                              // keep full rotation for body->map transform
    odom_received = true;
}



void local_avoidance(double min_distance){
    getAvoidParam("repulsive_m", repulsive_m);
    getAvoidParam("repulsive_gain", repulsive_gain);
    getAvoidParam("emergency_avoidance_m", emergency_avoidance_m);
    getAvoidParam("avoidance_moving_m", avoidance_moving_m);
    getAvoidParam("band_z_thr", band_z_thr);
    getAvoidParam("band_r_thr", band_r_thr);

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
    visualization_marker.header.frame_id = viz_frame_;
    visualization_marker.header.stamp = msg.header.stamp;
    visualization_marker.ns = "avoidance_target";
    visualization_marker.id = 0;
    visualization_marker.type = visualization_msgs::Marker::ARROW;
    visualization_marker.action = visualization_msgs::Marker::ADD;
    // 타겟 명령(PositionTarget) 자체를 그린다: 위치 = 회피 목표점, yaw = 명령 yaw(헤딩 유지).
    // 탈출 "방향" 화살표가 아님 — 목표점이 드론 기준 장애물 반대편에 찍히므로
    // 회피 방향은 화살표 위치(드론 대비 어디에 찍히는지)로 읽는다.
    visualization_marker.pose.position = msg.position;
    tf::Quaternion quat;
    quat.setRPY(0,0,msg.yaw);
    quat.normalize();
    visualization_marker.pose.orientation.x = quat[0];
    visualization_marker.pose.orientation.y = quat[1];
    visualization_marker.pose.orientation.z = quat[2];
    visualization_marker.pose.orientation.w = quat[3];
    visualization_marker.scale.x = 0.6;   // 화살표 길이 [m]
    visualization_marker.scale.y = 0.08;  // 몸통 두께
    visualization_marker.scale.z = 0.12;  // 머리 두께
    visualization_marker.color.r = 1.0f;  // 기존 rviz Pose 색(255,25,0)과 동일 계열
    visualization_marker.color.g = 0.1f;
    visualization_marker.color.b = 0.0f;
    visualization_marker.color.a = 1.0f;
    // 발행이 끊기면(미발동) rviz 가 이 시간 뒤 자동 제거 — DELETE 에지의 안전망
    visualization_marker.lifetime = ros::Duration(0.5);
}

// 메시지 타입 무관 공통 처리: cloud_data(센서 프레임) 채워진 뒤 호출된다.
// sensor_header 는 디버그 클라우드 발행용 (frame_id/stamp).
void processLidarCloud(const std_msgs::Header &sensor_header){
    getAvoidParam("avoidance_trigger_m", avoidance_trigger_m);
    getAvoidParam("release_delay_s", avoidance_release_delay_s);
    getAvoidParam("lidar_min_threshold", lidar_min_threshold);
    getAvoidParam("band_z_thr", band_z_thr);
    getAvoidParam("band_r_thr", band_r_thr);
    getAvoidParam("repulsive_m", repulsive_m);
    getAvoidParam("emergency_avoidance_m", emergency_avoidance_m);
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
            // rviz 표시용은 odom(월드) 좌표로 변환해 저장 — TF 의 base_link 는 FAST-LIO
            // body(=기울어진 센서 프레임)에 identity 로 붙어 있어 body 좌표로 발행하면
            // 25° 재기울임되어 보인다. mavros odom 자세(장착각 보상됨)로 직접 변환.
            pcl::PointXYZI dp;
            if(odom_received){
                tf::Vector3 pw = R_body_to_map * tf::Vector3(p.x, p.y, p.z);
                dp.x = pw.x() + current_pose.position.x;
                dp.y = pw.y() + current_pose.position.y;
                dp.z = pw.z() + current_pose.position.z;
            } else {                                  // odom 전: 바디 좌표 그대로
                dp.x = p.x; dp.y = p.y; dp.z = p.z;
            }
            double pd = ring ? rh : fabs(p.z);        // effective distance for this point
            // 1/2 = 척력만 주는 점 (trigger~repulsive_m), 3/4 = 발동시키는 점 (< trigger)
            dp.intensity = (ring ? 1.0f : 2.0f) + (pd < avoidance_trigger_m ? 2.0f : 0.0f);
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

    // 발동은 즉시, 해제는 release_delay_s 지연.
    // Mid360 비반복 스캔은 작은 장애물을 프레임마다 못 잡아 min_dist 가 "0.3m -> 없음"
    // 으로 튀고, 그때마다 ON/OFF 에지가 반복되며 로그 스팸 + 브리지 /planning/replan
    // 연발을 만든다 -> 근접점이 이 시간 연속 안 보여야 해제.
    static bool was_avoiding = false;
    static ros::Time last_close_t(0);
    const ros::Time now_t = ros::Time::now();
    const bool close_now = (minval < avoidance_trigger_m);
    if(close_now) last_close_t = now_t;
    const bool avoiding_now =
        close_now || (was_avoiding && (now_t - last_close_t).toSec() < avoidance_release_delay_s);

    if(avoiding_now){
        avoidance_enable = true;
        if(!was_avoiding)
            ROS_WARN("[local_avoidance] ACTIVATED  (min_dist=%.2f m, idx=%d)", minval, minIndex);
        else if(close_now)
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

    // publish the force-producing points for RViz debugging.
    // odom 수신 후엔 odom(월드) 프레임 좌표로 발행 (위에서 R_body_to_map 로 변환됨),
    // odom 수신 전엔 바디 좌표 그대로 + child_frame_id(없으면 라이다 msg frame) fallback.
    // 발동(ACTIVATED) 중에만 발행하고, 해제 에지에서 빈 클라우드를 한 번 보내
    // rviz 에 마지막 클라우드가 잔상으로 남는 것을 지운다.
    static bool dbg_was_active = false;
    if(avoidance_enable || dbg_was_active){
        if(!avoidance_enable) debug_cloud.clear();   // 해제 에지: 빈 클라우드로 지움
        sensor_msgs::PointCloud2 debug_msg;
        debug_cloud.width  = debug_cloud.points.size();
        debug_cloud.height = 1;
        debug_cloud.is_dense = false;
        pcl::toROSMsg(debug_cloud, debug_msg);
        debug_msg.header.frame_id = odom_received ? odom_frame_id
                                  : (body_frame_id.empty() ? sensor_header.frame_id : body_frame_id);
        debug_msg.header.stamp    = sensor_header.stamp;
        debug_pub.publish(debug_msg);
    }
    dbg_was_active = (avoidance_enable != 0);
}

#ifdef USE_LIVOX_LIDAR
// livox 드라이버 직결: CustomMsg(xfer_format=1, FAST-LIO 와 같은 스트림)에서
// CustomPoint 리스트를 직접 PCL 클라우드로 옮긴다.
void lidarCallback(const livox_ros_driver2::CustomMsg::ConstPtr &msg){
    if(!master_enable){ avoidance_enable = false; return; }   // 마스터 off: 연산 전체 스킵
    cloud_data.clear();
    cloud_data.points.reserve(msg->point_num);
    updateMountRotation();
    for(uint32_t i = 0; i < msg->point_num; i++){
        pcl::PointXYZ p;                          // 센서 -> 바디 (장착 pitch 보정)
        p.x =  mount_c*msg->points[i].x + mount_s*msg->points[i].z;
        p.y =  msg->points[i].y;
        p.z = -mount_s*msg->points[i].x + mount_c*msg->points[i].z;
        cloud_data.points.push_back(p);
    }
    processLidarCloud(msg->header);
}
#else
// 일반 라이다: sensor_msgs/PointCloud2 (센서/바디 프레임이어야 함!
// world 프레임 클라우드(/cloud_registered 등)를 물리면 밴드 게이팅이 틀어진다).
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    if(!master_enable){ avoidance_enable = false; return; }   // 마스터 off: 연산 전체 스킵
    cloud_data.clear();
    pcl::fromROSMsg(*msg, cloud_data);
    updateMountRotation();
    if(mount_s != 0.0){                           // 센서 -> 바디 (장착 pitch 보정)
        for(auto &p : cloud_data.points){
            float xb = mount_c*p.x + mount_s*p.z;
            p.z = -mount_s*p.x + mount_c*p.z;
            p.x = xb;
        }
    }
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
    set_default("/local_avoidance/sensor_pitch_deg",     0.0);   // 센서 장착 pitch (deg, 아래로 숙임=양수)

    // RViz fixed frame for the avoidance-direction arrow (defaults to the EPIC world frame).
    getAvoidParam("viz_frame", viz_frame_);

    updateMountRotation();
    ROS_INFO("[local_avoidance] sensor mount pitch = %.1f deg (sensor->body 보정 %s)",
             sensor_pitch_deg, (sensor_pitch_deg != 0.0) ? "ON" : "OFF: identity");

    // 입력 토픽도 프로파일 yaml 이 단일 소스 (local_avoidance/lidar_topic, odom_topic).
    // 키가 없으면 아래 기본값 (launch remap 도 여전히 동작).
    std::string lidar_topic = "/livox/lidar";
    std::string odom_topic  = "/mavros/local_position/odom";
    getAvoidParam("lidar_topic", lidar_topic);
    getAvoidParam("odom_topic",  odom_topic);


#ifdef USE_LIVOX_LIDAR
    ros::Subscriber lidar_sub = nh.subscribe<livox_ros_driver2::CustomMsg>(lidar_topic,1,lidarCallback);
    ROS_INFO("[local_avoidance] lidar input: livox_ros_driver2/CustomMsg on %s", lidar_topic.c_str());
#else
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic,1,lidarCallback);
    ROS_INFO("[local_avoidance] lidar input: sensor_msgs/PointCloud2 on %s "
             "(cloud must be in SENSOR frame)", lidar_topic.c_str());
#endif
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic,30, odom_cb);
    ros::Publisher position_target_pub= nh.advertise<mavros_msgs::PositionTarget>("/target_avoidance", 30);
    ros::Publisher FMS_flg_pub= nh.advertise<std_msgs::Int16>("/FSM_flag_avoidance", 30);
    ros::Publisher visualization_pub= nh.advertise<visualization_msgs::Marker>("/local_avoidance_visualization", 30);
    debug_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_avoidance/debug_points", 1);

    int param_update_inteval = 50;

    // 마스터 스위치: 프로파일 yaml 의 local_avoidance/enable (exploration_node ns 에 로드됨).
    // false 면 라이다 콜백부터 스킵하고 flag=0 만 발행 (완전 비활성).
    // 단독 실행(EPIC 없이) 시엔 /local_avoidance/enable (자체 yaml) -> 기본 true.
    auto read_master_enable = []() {
        bool en = true;
        getAvoidParam("enable", en);
        return en;
    };
    master_enable = read_master_enable();
    if(!master_enable){
        // crop 브릿지와 동일 패턴: yaml 이 꺼져 있으면 노드를 유지하지 않고 즉시 종료.
        // (켠 상태로 시작한 뒤 비행 중 rosparam 으로 끄는 것은 가능 — 아래 루프가 감지.
        //  끈 상태로 시작하면 재-launch 해야 켜진다.)
        ROS_WARN("[local_avoidance] local_avoidance/enable=false (profile yaml) -> exiting");
        return 0;
    }
    ROS_INFO("[local_avoidance] master enable = true (from profile yaml local_avoidance/enable)");
    int loop_cnt = 0;
    bool viz_shown = false;   // 화살표가 rviz 에 떠 있는 상태인지 (해제 에지 DELETE 용)

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
            visualization_pub.publish(visualization_marker);
            viz_shown = true;
        }
        else{
            FSM_flag.data=0;
            if(viz_shown){   // 해제 에지: lifetime 만료 기다리지 않고 화살표 즉시 제거
                visualization_marker.action = visualization_msgs::Marker::DELETE;
                visualization_pub.publish(visualization_marker);
                visualization_marker.action = visualization_msgs::Marker::ADD;
                viz_shown = false;
            }
        }
        FMS_flg_pub.publish(FSM_flag);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
