cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
) 
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable (map_pub src/map_publisher.cpp )
target_link_libraries(map_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

# Offline utility for the RViz-only map asset. It never runs in the MARSIM
# sensing path and cannot change the map seen by the simulator or planner.
add_executable(pcd_voxel_downsample src/pcd_voxel_downsample.cpp)
target_link_libraries(pcd_voxel_downsample
  ${PCL_LIBRARIES})

# Publishes an already-downsampled map exactly once as a latched debug topic.
# Keeping the publisher alive lets RViz reconnect without repeated PointCloud2
# serialization during planner timing measurements.
add_executable(static_pcd_publisher src/static_pcd_publisher.cpp)
target_link_libraries(static_pcd_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
