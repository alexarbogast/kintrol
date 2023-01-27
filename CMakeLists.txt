cmake_minimum_required(VERSION 3.0.2)
project(kintrol)

find_package(catkin REQUIRED COMPONENTS
    moveit_ros_planning_interface
    ros_control_boilerplate
    eigen_conversions
    actionlib
    actionlib_msgs
    message_generation
    geometry_msgs
    moveit_msgs
    std_msgs
)

add_service_files(
    DIRECTORY srv
    FILES 
        SwitchKintroller.srv
        SetIdleSetpoint.srv
)

add_action_files(
    DIRECTORY action
    FILES TrajectoryExecution.action
)

generate_messages(
  DEPENDENCIES 
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        kintrol
    CATKIN_DEPENDS
        moveit_ros_planning_interface
        ros_control_boilerplate
        eigen_conversions
        actionlib
        actionlib_msgs
        message_generation
        message_runtime
        geometry_msgs
        moveit_msgs
        std_msgs
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(kintrol 
    src/kintrol.cpp
    src/kintroller_manager.cpp
    src/kinematic_utilities.cpp
    src/low_pass_filter.cpp
    src/kintrollers/kintroller.cpp
    src/kintrollers/positioner_kintroller.cpp
)
target_link_libraries(kintrol ${catkin_LIBRARIES})

add_executable(kintrol_server src/kintrol_server.cpp)
target_link_libraries(kintrol_server kintrol)

add_executable(trajectory_execution_server src/trajectory_execution_server.cpp)
target_link_libraries(trajectory_execution_server ${catkin_LIBRARIES})
add_dependencies(trajectory_execution_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_subdirectory(kintrol_test)

# TEMPORARY
add_executable(my_test src/my_test.cpp)
target_link_libraries(my_test kintrol ${catkin_LIBRARIES})