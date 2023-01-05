#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <kintrol/TrajectoryExecutionAction.h>

#include <kintrol/kintroller_manager.h>

static const std::string HOME_POSITION = "home";
static const std::string READY_POITION = "ready";

// temporary, replace with parameters
static const std::string ROB1_END_EFFECTOR = "rob1_typhoon_extruder";
static const std::string ROB2_END_EFFECTOR = "rob2_typhoon_extruder";
static const std::string ROB3_END_EFFECTOR = "rob3_typhoon_extruder";

static const std::string BASE_FRAME = "positioner";

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
        manager_.start_joint_group_controller("positioner");

        sendTrajectories();

        manager_.start_trajectory_controller("robot1");
        manager_.start_trajectory_controller("robot2");
        manager_.start_trajectory_controller("robot3");
        manager_.start_trajectory_controller("positioner");

        moveReady();
        ros::waitForShutdown();
    }

    void sendTrajectories()
    {
        kintrol::TrajectoryExecutionGoal traj1, traj2, traj3;

        // find initial positions
        Eigen::Isometry3d rob1_eef_pose = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB1_END_EFFECTOR);
        Eigen::Isometry3d rob2_eef_pose = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB2_END_EFFECTOR);
        Eigen::Isometry3d rob3_eef_pose = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB3_END_EFFECTOR);

        const auto& world_to_base = hydra_move_group_interface_.getCurrentState()->getFrameTransform(BASE_FRAME);
        Eigen::Isometry3d base_to_world = world_to_base.inverse();

        rob1_eef_pose = base_to_world * rob1_eef_pose;
        rob2_eef_pose = base_to_world * rob2_eef_pose;
        rob3_eef_pose = base_to_world * rob3_eef_pose;

        // rob1 path
        geometry_msgs::Pose r1p1, r1p2, r1p3, r1p4, r1p5;
        tf::poseEigenToMsg(rob1_eef_pose, r1p1);
        r1p2.position.x = -0.197;
        r1p2.position.y = -0.141;
        r1p2.position.z =  0.0;

        r1p3.position.x = -0.197;
        r1p3.position.y = -0.288;
        r1p3.position.z =  0.0;

        r1p4.position.x = -0.05;
        r1p4.position.y = -0.288;
        r1p4.position.z =  0.0;

        r1p5.position.x = -0.05;
        r1p5.position.y = -0.141;
        r1p5.position.z =  0.0;

        // rob2 path
        geometry_msgs::Pose r2p1, r2p2, r2p3, r2p4, r2p5;;
        tf::poseEigenToMsg(rob2_eef_pose, r2p1);
        r2p2.position.x =  0.175;
        r2p2.position.y = -0.073;
        r2p2.position.z =  0.0;

        r2p3.position.x =  0.322;
        r2p3.position.y = -0.073;
        r2p3.position.z =  0.0;

        r2p4.position.x =  0.322;
        r2p4.position.y =  0.073;
        r2p4.position.z =  0.0;

        r2p5.position.x =  0.175;
        r2p5.position.y =  0.073;
        r2p5.position.z =  0.0;


        // rob3 path
        geometry_msgs::Pose r3p1, r3p2, r3p3, r3p4, r3p5;
        tf::poseEigenToMsg(rob3_eef_pose, r3p1);
        r3p2.position.x = -0.05;
        r3p2.position.y =  0.288;
        r3p2.position.z =  0.0;

        r3p3.position.x = -0.197;
        r3p3.position.y =  0.288;
        r3p3.position.z =  0.0;
        
        r3p4.position.x = -0.197;
        r3p4.position.y =  0.141;
        r3p4.position.z =  0.0;

        r3p5.position.x = -0.05;
        r3p5.position.y =  0.141;
        r3p5.position.z =  0.0;


        traj1.path.poses = {r1p1, r1p2, r1p3, r1p4, r1p5, r1p2, r1p1};
        traj2.path.poses = {r2p1, r2p2, r2p3, r2p4, r2p5, r2p2, r2p1};
        traj3.path.poses = {r3p1, r3p2, r3p3, r3p4, r3p5, r3p2, r3p1};

        std::cout << traj1 << std::endl;
        std::cout << traj2 << std::endl;
        std::cout << traj3 << std::endl;

        ac1.sendGoal(traj1);
        ac2.sendGoal(traj2);
        ac3.sendGoal(traj3);

        ac1.waitForResult();
        ac2.waitForResult();
        ac3.waitForResult();
    }

    void sendTrajectories2()
    {
        kintrol::TrajectoryExecutionGoal traj1, traj2, traj3;

        // find initial positions
        Eigen::Isometry3d rob1_eef_pose = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB1_END_EFFECTOR);
        Eigen::Isometry3d rob2_eef_pose = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB2_END_EFFECTOR);
        Eigen::Isometry3d rob3_eef_pose = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB3_END_EFFECTOR);

        const auto& world_to_base = hydra_move_group_interface_.getCurrentState()->getFrameTransform(BASE_FRAME);
        Eigen::Isometry3d base_to_world = world_to_base.inverse();

        rob1_eef_pose = base_to_world * rob1_eef_pose;
        rob2_eef_pose = base_to_world * rob2_eef_pose;
        rob3_eef_pose = base_to_world * rob3_eef_pose;

        // rob1 path
        geometry_msgs::Pose r1p1, r1p2, r1p3, r1p4, r1p5;
        tf::poseEigenToMsg(rob1_eef_pose, r1p1);
        r1p2.position.x = -0.300;
        r1p2.position.y = -0.300;
        r1p2.position.z =  0.0;

        r1p3.position.x =  0.300;
        r1p3.position.y = -0.300;
        r1p3.position.z =  0.0;

        r1p4.position.x =  0.300;
        r1p4.position.y =  0.300;
        r1p4.position.z =  0.0;

        r1p5.position.x = -0.300;
        r1p5.position.y =  0.300;
        r1p5.position.z =  0.0;

        // rob2 path
        geometry_msgs::Pose r2p1, r2p2, r2p3, r2p4, r2p5;;
        tf::poseEigenToMsg(rob2_eef_pose, r2p1);
        r2p2.position.x =  0.275;
        r2p2.position.y =  0.275;
        r2p2.position.z =  0.0;

        r2p3.position.x = -0.275;
        r2p3.position.y =  0.275;
        r2p3.position.z =  0.0;

        r2p4.position.x = -0.275;
        r2p4.position.y = -0.275;
        r2p4.position.z =  0.0;

        r2p5.position.x =  0.275;
        r2p5.position.y = -0.275;
        r2p5.position.z =  0.0;


        // rob3 path
        geometry_msgs::Pose r3p1, r3p2, r3p3, r3p4, r3p5;
        tf::poseEigenToMsg(rob3_eef_pose, r3p1);
        r3p2.position.x = -0.250;
        r3p2.position.y =  0.250;
        r3p2.position.z =  0.0;

        r3p3.position.x = -0.250;
        r3p3.position.y = -0.250;
        r3p3.position.z =  0.0;
        
        r3p4.position.x =  0.250;
        r3p4.position.y = -0.250;
        r3p4.position.z =  0.0;

        r3p5.position.x =  0.250;
        r3p5.position.y =  0.250;
        r3p5.position.z =  0.0;


        traj1.path.poses = {r1p1, r1p2, r1p3, r1p4, r1p5, r1p2, r1p1};
        traj2.path.poses = {r2p1, r2p2, r2p3, r2p4, r2p5, r2p2, r2p1};
        traj3.path.poses = {r3p1, r3p2, r3p3, r3p4, r3p5, r3p2, r3p1};

        std::cout << traj1 << std::endl;
        std::cout << traj2 << std::endl;
        std::cout << traj3 << std::endl;

        ac1.sendGoal(traj1);
        ac2.sendGoal(traj2);
        ac3.sendGoal(traj3);

        ac1.waitForResult();
        ac2.waitForResult();
        ac3.waitForResult();
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

    ros::AsyncSpinner spinner(2);
    spinner.start();

    KintrolTest test(nh);
    test.run();
}