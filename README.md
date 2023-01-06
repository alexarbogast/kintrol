# Kintrol

#### _A kinematic control library for robots in ROS_

This library provides kinematic control capabilities (ex. inverse jacobian, task-priority) to robots using the [MoveIt Motion Planning Framework for ROS](http://moveit.ros.org) and [ros_control](http://wiki.ros.org/ros_control).

## Multi-robot Demo

Clone dependencies to your workspace:

    git clone https://github.com/alexarbogast/za_description.git
    git clone https://github.com/alexarbogast/hydra_ros.git
 
Build and source the workspace:

    catkin build
    
Launch the demo:
    roslaunch hydra_bringup hydra_bringup.launch
    roslaunch kintrol kintrol_test.launch
    
    rosrun kintrol kintrol_test
