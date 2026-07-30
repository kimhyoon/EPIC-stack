cmake_minimum_required(VERSION 3.0.2)
project(ml_x_cropping)

# cloud_crop_bridge sits in the perception->planning serial path: every point of
# every 20 Hz scan goes through its crop loop, so build type is not cosmetic.
# modules/planner/epic/build_ws.sh always passes -DCMAKE_BUILD_TYPE=Release, but
# a bypassing build (catkin_make, a bare cmake, an IDE) leaves CMAKE_BUILD_TYPE
# empty, and GCC then defaults to -O0 - 10-20x slower on this loop, which turns
# straight into EPIC replan latency. Default to Release so that cannot happen.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

# !! NEVER add -ffast-math (nor -Ofast, which implies it) to this target. !!
# cloud_crop_bridge.cpp rejects non-finite points with a single range test that
# depends on IEEE semantics (all comparisons against NaN are false).
# -ffast-math lets GCC assume no NaN/Inf exists and deletes that filter, so NaN
# points would flow into EPIC's map. -march=native is likewise banned: the same
# image is built once and run on both x86 hosts and the Jetson AGX Orin.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

option(BUILD_CLOUD_CROP_BRIDGE "Build MID360-to-ML-X cloud crop bridge" OFF)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  visualization_msgs
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
