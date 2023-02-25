cmake_minimum_required(VERSION 3.0.2)

include_directories(
  include/
)

catkin_install_python(PROGRAMS 
  scripts/trajectory_test.py
  scripts/multi_robot_trajectory_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(kintrol_test src/kintrol_test.cpp)
target_link_libraries(kintrol_test kintrol ${catkin_LIBRARIES})