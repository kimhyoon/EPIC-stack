cmake_minimum_required(VERSION 2.8.3)
project(epic_planner)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  pointcloud_topo
  path_searching
  cv_bridge
  frontier_manager
  traj_utils
  lkh_tsp_solver
  plan_manage
  lidar_map
  cv_bridge
  message_filters
  message_generation
)

add_service_files(
  FILES
  GoalService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epic_planner
  CATKIN_DEPENDS pointcloud_topo active_perception path_searching traj_utils lkh_tsp_solver plan_manage frontier_manager lidar_map message_runtime
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fsm_utils.cpp
)

add_dependencies(exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(exploration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
  epic_planner
)

add_library(epic_planner
  src/fast_exploration_manager.cpp
)
target_link_libraries(epic_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
