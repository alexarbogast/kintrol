#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/SwitchController.h>

// parameters
static const std::string PLANNING_GROUP = "manipulator";
static const std::string HOME_POSITION = "home";
static const std::string READY_POSITION = "ready";

static const std::string VELOCITY_CONTROLLER = "joint_velocity_controller";
static const std::string TRAJECTORY_CONTROLLER = "position_trajectory_controller";
static const std::string COMMAND_OUT_TOPIC = "target_pose";

class Test
{
public:
    Test(ros::NodeHandle& nh)
        : move_group_interface_(PLANNING_GROUP)
    {
        client_ = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
        target_twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(COMMAND_OUT_TOPIC, 1 /* queue */, true /* latch */);
    }

    bool start_joint_group_velocity_controller()
    {
        controller_manager_msgs::SwitchController srv;
    
        srv.request.start_controllers = {VELOCITY_CONTROLLER};
        srv.request.stop_controllers = {TRAJECTORY_CONTROLLER};
        srv.request.strictness = srv.request.BEST_EFFORT;

        if (client_.call(srv))
        {
            ROS_INFO("Switched to joint_group_velocity_controller");
        }
        else
        {
            ROS_ERROR("Failed to switch to joint_group_velocity_controller");
            return false;
        }

        return true;
    }

    bool start_trajectory_controller()
    {
        controller_manager_msgs::SwitchController srv;
    
        srv.request.start_controllers = {TRAJECTORY_CONTROLLER};
        srv.request.stop_controllers = {VELOCITY_CONTROLLER};
        srv.request.strictness = srv.request.BEST_EFFORT;

        if (client_.call(srv))
        {
            ROS_INFO("Switched to position_trajectory_controller");
        }
        else
        {
            ROS_ERROR("Failed to switch to position_trajectory_controller");
            return false;
        }

        return true;
    }

    void moveHome()
    {
        move_group_interface_.setNamedTarget(HOME_POSITION);
        move_group_interface_.move();
    }

    void moveReady()
    {
        move_group_interface_.setNamedTarget(READY_POSITION);
        move_group_interface_.move();
    }

    void poseTracking()
    {
        start_joint_group_velocity_controller();

        while (ros::ok())
        {
            geometry_msgs::TwistStamped target_twist;

            ros::Rate loop_rate(50);
            for (size_t i = 0; i < 500; ++i)
            {
                target_twist.twist.linear.x = 0.1;
                target_twist.header.stamp = ros::Time::now();
                target_twist_pub_.publish(target_twist);
            }
    
            target_twist.twist.linear.x = 0.0;
            target_twist.header.stamp = ros::Time::now();
            target_twist_pub_.publish(target_twist);
        }
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    ros::ServiceClient client_;
    ros::Publisher target_twist_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "top_level_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    Test test(nh);

    test.moveReady();
    test.start_joint_group_velocity_controller();
    test.poseTracking();
}