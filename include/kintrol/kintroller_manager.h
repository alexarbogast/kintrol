#pragma once

#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>

namespace kintrol
{
struct RobotContext
{
    std::string name;
    std::string joint_group_controller;
    std::string trajectory_controller;

    std::string home_position;
    std::string ready_position;
};

class KintrollerManager
{
public:
    KintrollerManager(ros::NodeHandle& nh);

    bool start_joint_group_controller(const std::string& robot_name);
    bool start_trajectory_controller(const std::string& robot_name);

private:
    ros::ServiceClient ros_control_client_;
    std::map<std::string, RobotContext> robot_contexts_;
};

} // namespace kintrol