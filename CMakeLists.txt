cmake_minimum_required(VERSION 3.0.2)
project(kintrol)

find_package(catkin REQUIRED COMPONENTS
    moveit_ros_planning_interface
    ros_control_boilerplate
    geometry_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    moveit_ros_planning_interface
    ros_control_boilerplate
    geometry_msgs
    std_msgs
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(kintrol src/kintrol.cpp)
target_link_libraries(kintrol ${catkin_LIBRARIES})

add_library(kintroller_manager src/kintroller_manager.cpp)
target_link_libraries(kintroller_manager ${catkin_LIBRARIES})

add_executable(kintrol_server src/kintrol_server.cpp)
target_link_libraries(kintrol_server kintrol)

add_subdirectory(kintrol_test)

# TEMPORARY
add_executable(multi_robot_traj_test src/multi_robot_trajectory_test.cpp)
target_link_libraries(multi_robot_traj_test ${catkin_LIBRARIES})

add_executable(top_level_node src/top_level_node.cpp)
target_link_libraries(top_level_node ${catkin_LIBRARIES})

add_executable(kintrolV2 src/kintrolV2.cpp)
target_link_libraries(kintrolV2 ${catkin_LIBRARIES})

add_executable(trajectory_pub src/trajectory_pub.cpp)
target_link_libraries(trajectory_pub ${catkin_LIBRARIES})
