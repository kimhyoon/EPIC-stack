cmake_minimum_required(VERSION 3.0.2)
project(local_avoidance)

add_compile_options(-std=c++14)

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
  livox_ros_driver2
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_msgs mavros_msgs tf pcl_ros pcl_conversions livox_ros_driver2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(local_avoidance src/local_avoidance.cpp)
target_link_libraries(local_avoidance
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
