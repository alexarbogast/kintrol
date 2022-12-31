#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace kintrol
{
struct KintrolParameters
{
    std::string command_topic;
    std::string setpoint_topic;
    double control_freq;
    std::string joint_model_group;

    size_t ros_queue_size;
};

class Kintrol
{
public:
    Kintrol(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
    void run();
    void run_old();
private:
    // callbacks
    void twistStampedCB(const geometry_msgs::TwistStampedConstPtr& msg);

    bool readParameters();
    inline void psuedoInverseJacobian(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& inverse);
private:
    KintrolParameters parameters_;
    geometry_msgs::TwistStamped setpoint_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher command_pub_;
    ros::Subscriber setpoint_sub_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    moveit::core::RobotStatePtr current_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
};

} // namespace kintrol