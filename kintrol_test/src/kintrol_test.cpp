#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <kintrol/TrajectoryExecutionAction.h>

#include <kintrol/kintroller_manager.h>

static const std::string HOME_POSITION = "home";
static const std::string READY_POITION = "ready";

class KintrolTest
{
private:
    typedef actionlib::SimpleActionClient<kintrol::TrajectoryExecutionAction> TrajectoryExecutionServer;

public:
    KintrolTest(ros::NodeHandle& nh)
        : hydra_move_group_interface_("hydra_planning_group"),
          ac1("/robot1/trajectory_execution_action", true),
          ac2("/robot2/trajectory_execution_action", true),
          ac3("/robot3/trajectory_execution_action", true),
          manager_(nh)
    {

        ROS_INFO("Waiting for action servers...");
        ac1.waitForServer();
        ac2.waitForServer();
        ac3.waitForServer();
        ROS_INFO("Connected to action servers");
    }

    void run()
    {
        moveReady();

        manager_.start_joint_group_controller("robot1");
        manager_.start_joint_group_controller("robot2");
        manager_.start_joint_group_controller("robot3");

        sendTrajectories();
        ros::waitForShutdown();
    }

    void sendTrajectories()
    {
        kintrol::TrajectoryExecutionGoal traj1, traj2, traj3;
    
        geometry_msgs::Pose p1, p2, p3, p4, p5, p6;
        p1.position.x = 0.0;
        p1.position.y = 0.0;
        p1.position.z = 0.0;

        p2.position.x = 0.0;
        p2.position.y = 0.3;
        p2.position.z = 0.0;

        p3.position.x = 0.3;
        p3.position.y = 0.3;
        p3.position.z = 0.0;

        p4.position.x = 0.3;
        p4.position.y = -0.3;
        p4.position.z = 0.0;

        p5.position.x = 0.0;
        p5.position.y = -0.3;
        p5.position.z = 0.0;

        p6.position.x = 0.0;
        p6.position.y = 0.0;
        p6.position.z = 0.0;

        traj1.path.poses = {p1, p2, p3, p4, p5, p6};
        traj2.path.poses = {p1, p2, p3, p4, p5, p6};
        traj3.path.poses = {p1, p2, p3, p4, p5, p6};

        ac1.sendGoal(traj1);
        ac2.sendGoal(traj2);
        ac3.sendGoal(traj3);
    }

    void moveHome()
    {
        hydra_move_group_interface_.setNamedTarget(HOME_POSITION);
        hydra_move_group_interface_.move();
    }

    void moveReady()
    {
        hydra_move_group_interface_.setNamedTarget(READY_POITION);
        hydra_move_group_interface_.move();
    }

private:
    moveit::planning_interface::MoveGroupInterface hydra_move_group_interface_;
    kintrol::KintrollerManager manager_;

    // trajectory execution clients
    TrajectoryExecutionServer ac1;
    TrajectoryExecutionServer ac2;
    TrajectoryExecutionServer ac3;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kintrol_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    KintrolTest test(nh);
    test.run();
}