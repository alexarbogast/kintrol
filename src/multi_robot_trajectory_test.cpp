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
        : hydra_move_group_interface_("hydra_planning_group"),
          rob1_move_group_interface_("rob1_planning_group"),
          rob2_move_group_interface_("rob2_planning_group"),
          rob3_move_group_interface_("rob3_planning_group")
    {
        client_ = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
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
        hydra_move_group_interface_.setNamedTarget(HOME_POSITION);
        hydra_move_group_interface_.move();
    }

    void moveReady()
    {
        hydra_move_group_interface_.setNamedTarget(READY_POSITION);
        hydra_move_group_interface_.move();
    }

    void moveRobot1()
    {
        rob1_move_group_interface_.setNamedTarget(HOME_POSITION);
        rob1_move_group_interface_.move();
    }

    void moveRobot2()
    {
        rob2_move_group_interface_.setNamedTarget(HOME_POSITION);
        rob2_move_group_interface_.move();
    }

    void moveRobot3()
    {
        rob3_move_group_interface_.setNamedTarget(HOME_POSITION);
        rob3_move_group_interface_.move();
    }

private:
    moveit::planning_interface::MoveGroupInterface hydra_move_group_interface_;
    moveit::planning_interface::MoveGroupInterface rob1_move_group_interface_;
    moveit::planning_interface::MoveGroupInterface rob2_move_group_interface_;
    moveit::planning_interface::MoveGroupInterface rob3_move_group_interface_;
    

    ros::ServiceClient client_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_robot_trajectory_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    Test test(nh);

    test.moveHome();
    test.moveReady();
    
    test.moveRobot1();
    test.moveRobot2();
    test.moveRobot3();
}