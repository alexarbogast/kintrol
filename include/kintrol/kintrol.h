#pragma once

#include <ros/ros.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <unordered_map>

#include "kintrol/kintrollers/kintroller.h"
#include "kintrol/kintrollers/positioner_kintroller.h"
#include "kintrol/SwitchKintroller.h"

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
    bool switchKintrollerService(SwitchKintroller::Request &req,
                                 SwitchKintroller::Response &res);

    bool readParameters();
    bool registerKintrollers();
    void setIdleSetpoint();

private:
    KintrolParameters parameters_;
    moveit_msgs::CartesianTrajectoryPoint setpoint_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher command_pub_;
    ros::Subscriber setpoint_sub_;

    ros::ServiceServer switch_kintroller_service_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    moveit::core::RobotStatePtr current_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    unsigned int n_variables_;
    KinematicChain kinematic_chain_;

    std::shared_ptr<KintrollerBase> active_kintroller_;
    std::unordered_map<std::string, KintrollerPtr> kintroller_map_;
};

} // namespace kintrol