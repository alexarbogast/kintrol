cmake_minimum_required(VERSION 3.0.2)

include_directories(
  include/
)

add_executable(kintrol_test src/kintrol_test.cpp)
target_link_libraries(kintrol_test kintroller_manager ${catkin_LIBRARIES})