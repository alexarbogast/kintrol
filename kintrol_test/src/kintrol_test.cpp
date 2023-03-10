#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>

#include <kintrol/TrajectoryExecutionAction.h>
#include <kintrol/kintroller_manager.h>
#include <kintrol/SwitchKintroller.h>

static const std::string HOME_POSITION = "home";
static const std::string READY_POITION = "ready";

// temporary, replace with parameters
static const std::string ROB1_END_EFFECTOR = "rob1_typhoon_extruder";
static const std::string ROB2_END_EFFECTOR = "rob2_typhoon_extruder";
static const std::string ROB3_END_EFFECTOR = "rob3_typhoon_extruder";

static const std::string BASE_FRAME = "positioner_static";
static const std::string COORDINATED_BASE_FRAME = "positioner";

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

        // register service clients
        rob1_switch_kintroller_client = nh.serviceClient<kintrol::SwitchKintroller>("robot1/kintrol_server/switch_kintrollers");
        rob2_switch_kintroller_client = nh.serviceClient<kintrol::SwitchKintroller>("robot2/kintrol_server/switch_kintrollers");
        rob3_switch_kintroller_client = nh.serviceClient<kintrol::SwitchKintroller>("robot3/kintrol_server/switch_kintrollers");
    }

    void run()
    {
        moveReady();

        manager_.start_joint_group_controller("robot1");
        manager_.start_joint_group_controller("robot2");
        manager_.start_joint_group_controller("robot3");
        manager_.start_joint_group_controller("positioner");

        sendTrajectories();
        sendTrajectories2();

        manager_.start_trajectory_controller("robot1");
        manager_.start_trajectory_controller("robot2");
        manager_.start_trajectory_controller("robot3");
        manager_.start_trajectory_controller("positioner");

        moveReady();
    }

    void sendTrajectories()
    {
        kintrol::TrajectoryExecutionGoal traj1, traj2, traj3;

        // find initial positions
        Eigen::Isometry3d rob1_eef_pose_world = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB1_END_EFFECTOR);
        Eigen::Isometry3d rob2_eef_pose_world = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB2_END_EFFECTOR);
        Eigen::Isometry3d rob3_eef_pose_world = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB3_END_EFFECTOR);

        const auto& world_to_base = hydra_move_group_interface_.getCurrentState()->getFrameTransform(COORDINATED_BASE_FRAME);

        Eigen::Isometry3d rob1_eef_pose_base = world_to_base.inverse() * rob1_eef_pose_world;
        Eigen::Isometry3d rob2_eef_pose_base = world_to_base.inverse() * rob2_eef_pose_world;
        Eigen::Isometry3d rob3_eef_pose_base = world_to_base.inverse() * rob3_eef_pose_world;

        // ============== Coordinated Trajectories ======================
        // rob1 path
        geometry_msgs::Pose r1p1, r1p2, r1p3, r1p4, r1p5;
        tf::poseEigenToMsg(rob1_eef_pose_base, r1p1);
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
        tf::poseEigenToMsg(rob2_eef_pose_base, r2p1);
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
        tf::poseEigenToMsg(rob3_eef_pose_base, r3p1);
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

        traj1.path.poses = {r1p1, r1p2, r1p3, r1p4, r1p5, r1p2};
        traj2.path.poses = {r2p1, r2p2, r2p3, r2p4, r2p5, r2p2};
        traj3.path.poses = {r3p1, r3p2, r3p3, r3p4, r3p5, r3p2};

        // switch to difference kintroller using service
        kintrol::SwitchKintroller srv;
        srv.request.kintroller_name = "coordinated_kintroller";
        rob1_switch_kintroller_client.call(srv);
        rob2_switch_kintroller_client.call(srv);
        rob3_switch_kintroller_client.call(srv);

        ac1.sendGoal(traj1);
        ac2.sendGoal(traj2);
        ac3.sendGoal(traj3);

        ac1.waitForResult();
        ac2.waitForResult();
        ac3.waitForResult();

        // =================== Normal Trajectories ========================
        Eigen::Isometry3d rob1_eef_pose_after = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB1_END_EFFECTOR);
        Eigen::Isometry3d rob2_eef_pose_after = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB2_END_EFFECTOR);
        Eigen::Isometry3d rob3_eef_pose_after = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB3_END_EFFECTOR);
        const auto& world_to_base2 = hydra_move_group_interface_.getCurrentState()->getFrameTransform(BASE_FRAME);

        rob1_eef_pose_after = world_to_base2.inverse() * rob1_eef_pose_after;
        rob2_eef_pose_after = world_to_base2.inverse() * rob2_eef_pose_after;
        rob3_eef_pose_after = world_to_base2.inverse() * rob3_eef_pose_after;

        Eigen::Isometry3d rob1_eef_pose_final = world_to_base2.inverse() * rob1_eef_pose_world;
        Eigen::Isometry3d rob2_eef_pose_final = world_to_base2.inverse() * rob2_eef_pose_world;
        Eigen::Isometry3d rob3_eef_pose_final = world_to_base2.inverse() * rob3_eef_pose_world;
        

        // robot 1
        tf::poseEigenToMsg(rob1_eef_pose_after, r1p1);
        tf::poseEigenToMsg(rob1_eef_pose_final, r1p2);

        // robot 2
        tf::poseEigenToMsg(rob2_eef_pose_after, r2p1);
        tf::poseEigenToMsg(rob2_eef_pose_final, r2p2);

        // robot 3
        tf::poseEigenToMsg(rob3_eef_pose_after, r3p1);
        tf::poseEigenToMsg(rob3_eef_pose_final, r3p2);

        traj1.path.poses = {r1p1, r1p2};
        traj2.path.poses = {r2p1, r2p2};
        traj3.path.poses = {r3p1, r3p2};

        // switch to difference kintroller using service
        srv.request.kintroller_name = "kintroller";
        rob1_switch_kintroller_client.call(srv);
        rob2_switch_kintroller_client.call(srv);
        rob3_switch_kintroller_client.call(srv);

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
        Eigen::Isometry3d rob1_eef_pose_world = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB1_END_EFFECTOR);
        Eigen::Isometry3d rob2_eef_pose_world = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB2_END_EFFECTOR);
        Eigen::Isometry3d rob3_eef_pose_world = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB3_END_EFFECTOR);

        const auto& world_to_base = hydra_move_group_interface_.getCurrentState()->getFrameTransform(COORDINATED_BASE_FRAME);

        Eigen::Isometry3d rob1_eef_pose_base = world_to_base.inverse() * rob1_eef_pose_world;
        Eigen::Isometry3d rob2_eef_pose_base = world_to_base.inverse() * rob2_eef_pose_world;
        Eigen::Isometry3d rob3_eef_pose_base = world_to_base.inverse() * rob3_eef_pose_world;

        // ============== Coordinated Trajectories ======================
        // rob1 path
        geometry_msgs::Pose r1p1, r1p2, r1p3, r1p4, r1p5;
        tf::poseEigenToMsg(rob1_eef_pose_base, r1p1);
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
        tf::poseEigenToMsg(rob2_eef_pose_base, r2p1);
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
        tf::poseEigenToMsg(rob3_eef_pose_base, r3p1);
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

        // switch to difference kintroller using service
        kintrol::SwitchKintroller srv;
        srv.request.kintroller_name = "coordinated_kintroller";
        rob1_switch_kintroller_client.call(srv);
        rob2_switch_kintroller_client.call(srv);
        rob3_switch_kintroller_client.call(srv);

        ac1.sendGoal(traj1);
        ac2.sendGoal(traj2);
        ac3.sendGoal(traj3);

        ac1.waitForResult();
        ac2.waitForResult();
        ac3.waitForResult();

        // =================== Normal Trajectories ========================
        Eigen::Isometry3d rob1_eef_pose_after = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB1_END_EFFECTOR);
        Eigen::Isometry3d rob2_eef_pose_after = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB2_END_EFFECTOR);
        Eigen::Isometry3d rob3_eef_pose_after = hydra_move_group_interface_.getCurrentState()->getFrameTransform(ROB3_END_EFFECTOR);
        const auto& world_to_base2 = hydra_move_group_interface_.getCurrentState()->getFrameTransform(BASE_FRAME);

        rob1_eef_pose_after = world_to_base2.inverse() * rob1_eef_pose_after;
        rob2_eef_pose_after = world_to_base2.inverse() * rob2_eef_pose_after;
        rob3_eef_pose_after = world_to_base2.inverse() * rob3_eef_pose_after;

        Eigen::Isometry3d rob1_eef_pose_final = world_to_base2.inverse() * rob1_eef_pose_world;
        Eigen::Isometry3d rob2_eef_pose_final = world_to_base2.inverse() * rob2_eef_pose_world;
        Eigen::Isometry3d rob3_eef_pose_final = world_to_base2.inverse() * rob3_eef_pose_world;
        
        // robot 1
        tf::poseEigenToMsg(rob1_eef_pose_after, r1p1);
        tf::poseEigenToMsg(rob1_eef_pose_final, r1p2);

        // robot 2
        tf::poseEigenToMsg(rob2_eef_pose_after, r2p1);
        tf::poseEigenToMsg(rob2_eef_pose_final, r2p2);

        // robot 3
        tf::poseEigenToMsg(rob3_eef_pose_after, r3p1);
        tf::poseEigenToMsg(rob3_eef_pose_final, r3p2);

        traj1.path.poses = {r1p1, r1p2};
        traj2.path.poses = {r2p1, r2p2};
        traj3.path.poses = {r3p1, r3p2};

        // switch to difference kintroller using service
        srv.request.kintroller_name = "kintroller";
        rob1_switch_kintroller_client.call(srv);
        rob2_switch_kintroller_client.call(srv);
        rob3_switch_kintroller_client.call(srv);

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

    // service clients
    ros::ServiceClient rob1_switch_kintroller_client;
    ros::ServiceClient rob2_switch_kintroller_client;
    ros::ServiceClient rob3_switch_kintroller_client;
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