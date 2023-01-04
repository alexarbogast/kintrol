#pragma once

#include <ros/ros.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "kintrol/kintrollers/kintroller.h"

namespace kintrol
{
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
    unsigned int n_variables_;
    KinematicChain kinematic_chain_;

    std::unique_ptr<kintrollers::KintrollerBase> kintroller_;
};

} // namespace kintrol