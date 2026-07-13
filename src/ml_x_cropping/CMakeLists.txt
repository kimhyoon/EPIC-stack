cmake_minimum_required(VERSION 2.8.3)
project(ml_x_cropping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

option(BUILD_CLOUD_CROP_BRIDGE "Build MID360-to-ML-X cloud crop bridge" OFF)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

if(BUILD_CLOUD_CROP_BRIDGE)
  add_executable(cloud_crop_bridge
    src/cloud_crop_bridge.cpp
  )
  add_dependencies(cloud_crop_bridge ${catkin_EXPORTED_TARGETS})
  target_link_libraries(cloud_crop_bridge
    ${catkin_LIBRARIES}
  )
endif()
