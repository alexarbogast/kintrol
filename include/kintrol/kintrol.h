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
    std::string base_frame;

    size_t ros_queue_size;
};

class Kintrol
{
public:
    Kintrol(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
    void run();
private:
    // callbacks
    void twistStampedCB(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg);

    bool readParameters();
    inline void extractPosition(Eigen::VectorXd& pose);
    inline void extractVelocity(Eigen::VectorXd& pose);
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
};

} // namespace kintrol