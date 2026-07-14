cmake_minimum_required(VERSION 3.0.2)
project(local_avoidance)

add_compile_options(-std=c++14)

# ── 라이다 입력 타입 선택 ────────────────────────────────────────────────────
#  ON  : livox_ros_driver2/CustomMsg 구독 (Mid360 등 livox 드라이버 직결)
#        -> 워크스페이스에 livox_ros_driver2 패키지 필요
#  OFF : sensor_msgs/PointCloud2 구독 (일반 라이다)
#        -> livox_ros_driver2 의존성 없이 빌드됨 (다른 PC 이식용)
#  전환 방법:
#    catkin build local_avoidance --cmake-args -DUSE_LIVOX_LIDAR=OFF
#    (영구 설정: catkin config --cmake-args -DUSE_LIVOX_LIDAR=OFF)
option(USE_LIVOX_LIDAR "Use livox_ros_driver2 CustomMsg as lidar input (OFF = sensor_msgs/PointCloud2)" OFF)

set(LIDAR_DEP "")
if(USE_LIVOX_LIDAR)
  set(LIDAR_DEP livox_ros_driver2)
  message(STATUS "[local_avoidance] lidar input = livox_ros_driver2/CustomMsg (USE_LIVOX_LIDAR=ON)")
else()
  message(STATUS "[local_avoidance] lidar input = sensor_msgs/PointCloud2 (USE_LIVOX_LIDAR=OFF, no livox dependency)")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  mavros_msgs
  tf
  pcl_ros
  pcl_conversions
  ${LIDAR_DEP}
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_msgs mavros_msgs tf pcl_ros pcl_conversions ${LIDAR_DEP}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(local_avoidance src/local_avoidance.cpp)
if(USE_LIVOX_LIDAR)
  target_compile_definitions(local_avoidance PRIVATE USE_LIVOX_LIDAR)
endif()
target_link_libraries(local_avoidance
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
