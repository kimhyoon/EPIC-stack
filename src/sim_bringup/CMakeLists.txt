cmake_minimum_required(VERSION 2.8.3)
project(sim_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_result_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
