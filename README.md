# Kintrol

#### _A kinematic control library for robots in ROS_

This library provides kinematic control capabilities (ex. inverse jacobian, task-priority) to robots using the [MoveIt Motion Planning Framework for ROS](http://moveit.ros.org) and [ros_control](http://wiki.ros.org/ros_control).

## Installation

Clone the repository to your local workspace. If you do not already have the 
[za_description](https://github.com/alexarbogast/za_description/tree/e57f65c3f8eb0be88e7739a8b5162b4b3b875b15) package, you will need to clone recursively. 

```shell script
mkdir catkin_ws/src && cd catkin_ws/src
git clone https://github.com/alexarbogast/kintrol.git
```

Build your workspace
```shell script
catkin build
```

## Running the demos
### Dependencies

To run the demos, you will need to follow the instructions at [za_ros](https://github.com/alexarbogast/za_ros.git) to install the za_ros package. If you would also like to run the multi-robot demo, follow the instructions at [hydra_ros](https://github.com/alexarbogast/za_ros.git) to install the ros integration package for the multi-robot system. 

### Tormach Za6 Demo
To run the example demostrating the capabilities of the task priority controller

```bash shell
roslaunch za_gazebo za_robot.launch controller:="task_priority_controller"
roslaunch kintrol trajectory_execution_test.launch
```

This example utilizes the trajectory_execution_server. This is an actionlib server that executes trajectories generated online by the  [ruckig](https://github.com/pantor/ruckig) library. The example drives the robot to different cartesian positions, and the robot maximizes its manipulability along every path.

### Multi-robot Demo

Ensure that you have installed the multi-robot files from [hydra_ros](https://github.com/alexarbogast/za_ros). Launch the hydra gazebo simulation with
```bash shell
roslaunch hydra_gazebo hydra.launch controller:="robot1_task_priority_controller robot2_task_priority_controller robot3_task_priority_controller"
```

To run the same demo as above on the multi-robot system
```bash shell
roslaunch kintrol multi_robot_traj_exec.launch
```

If you're interested in sending your own trajectories to the robots, [trajectory_test.py](kintrol_test/scripts/trajectory_test.py) is a good place to start.
