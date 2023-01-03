#pragma once

#include <ros/ros.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace kintrol
{
struct KintrolParameters
{
    std::string command_topic;
    std::string setpoint_topic;
    double control_freq;
    std::string joint_model_group;
    std::string end_effector;
    std::string pose_frame;

    size_t ros_queue_size;
};

typedef std::vector<const robot_model::JointModel*> JointModelChain;

class Kintrol
{
public:
    Kintrol(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
    void run();
private:
    // callbacks
    void twistStampedCB(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg);

    bool readParameters();
    void setIdleSetpoint();
    inline void extractPosition(Eigen::VectorXd& pose);
    inline void extractVelocity(Eigen::VectorXd& pose);
    void getJacobian(robot_state::RobotStatePtr& state,
                     const std::string& end_frame, 
                     const std::string& start_frame,
                     Eigen::MatrixXd& jacobian);
    void getKinematicChain(const robot_model::RobotModelConstPtr& robot_model,
                           const std::string& end_frame,
                           const std::string& start_frame);
    inline void psuedoInverseJacobian(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& inverse);
private:
    KintrolParameters parameters_;
    moveit_msgs::CartesianTrajectoryPoint setpoint_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher command_pub_;
    ros::Subscriber setpoint_sub_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    moveit::core::RobotStatePtr current_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    JointModelChain joint_model_chain_;
};

} // namespace kintrol