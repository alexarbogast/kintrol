#include "kintrol/kintroller_manager.h"

static const std::string HOME_POSITION = "home";
static const std::string READY_POSITION = "ready";

namespace kintrol
{
KintrollerManager::KintrollerManager(ros::NodeHandle& nh)
{
    ros_control_client_ = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    // temporary initialization
    RobotContext rob1_context;
    rob1_context.name = "robot1";
    rob1_context.joint_group_controller = "robot1_joint_velocity_controller";
    rob1_context.trajectory_controller = "robot1_trajectory_controller";
    rob1_context.home_position = HOME_POSITION;
    rob1_context.ready_position = READY_POSITION;

    RobotContext rob2_context;
    rob2_context.name = "robot2";
    rob2_context.joint_group_controller = "robot2_joint_velocity_controller";
    rob2_context.trajectory_controller = "robot2_trajectory_controller";
    rob2_context.home_position = HOME_POSITION;
    rob2_context.ready_position = READY_POSITION;

    RobotContext rob3_context;
    rob3_context.name = "robot3";
    rob3_context.joint_group_controller = "robot3_joint_velocity_controller";
    rob3_context.trajectory_controller = "robot3_trajectory_controller";
    rob3_context.home_position = HOME_POSITION;
    rob3_context.ready_position = READY_POSITION;

    robot_contexts_[rob1_context.name] = rob1_context;
    robot_contexts_[rob2_context.name] = rob2_context;
    robot_contexts_[rob3_context.name] = rob3_context;
}

bool KintrollerManager::start_joint_group_controller(const std::string& robot_name)
{
    if (!robot_contexts_.count(robot_name))
    {
        ROS_ERROR_STREAM("Robot " << robot_name <<  " does not exist in kintroller manager");
        return false;
    }

    RobotContext& robot_context = robot_contexts_[robot_name];
    controller_manager_msgs::SwitchController srv;

    srv.request.start_controllers = {robot_context.joint_group_controller};
    srv.request.stop_controllers = {robot_context.trajectory_controller};
    srv.request.strictness = srv.request.BEST_EFFORT;

    if (!ros_control_client_.call(srv))
    {
        ROS_ERROR("Failed to switch to joint_group_velocity_controller");
        return false;
    }
    return true;
}

bool KintrollerManager::start_trajectory_controller(const std::string& robot_name)
{
    if (!robot_contexts_.count(robot_name))
        {
            ROS_ERROR_STREAM("Robot " << robot_name <<  " does not exist in kintroller manager");
            return false;
        }

        RobotContext& robot_context = robot_contexts_[robot_name];
        controller_manager_msgs::SwitchController srv;

        srv.request.start_controllers = {robot_context.trajectory_controller};
        srv.request.stop_controllers = {robot_context.joint_group_controller};
        srv.request.strictness = srv.request.BEST_EFFORT;

        if (!ros_control_client_.call(srv))
        {
            ROS_ERROR("Failed to switch to joint_group_velocity_controller");
            return false;
        }
        return true;
}

} // namespace kintrol

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "kintroller_manager");
//    ros::NodeHandle nh;
//
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//
//    kintrol::KintrollerManager manager(nh);
//    manager.start_joint_group_controller("robot1");
//    manager.start_joint_group_controller("robot2");
//    manager.start_joint_group_controller("robot3");
//
//    manager.start_trajectory_controller("robot1");
//    manager.start_trajectory_controller("robot2");
//    manager.start_trajectory_controller("robot3");
//}