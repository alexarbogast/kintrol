#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <std_msgs/Float64MultiArray.h>

#include "kintrol/kintrol.h"


static std::string JOINT_TOPIC = "joint_states";
static double ROBOT_STATE_WAIT_TIME = 10.0; // seconds

const static std::string LOGNAME = "kintrol_server";

namespace kintrol
{
Kintrol::Kintrol(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
    : pnh_(nh), planning_scene_monitor_(planning_scene_monitor)
{
    if (!readParameters())
    {
        ROS_ERROR("Failed to inialize kintrol parameters");
    }

    if (!planning_scene_monitor_->getStateMonitor())
    {
        planning_scene_monitor_->startStateMonitor("/" + JOINT_TOPIC);
    }
    planning_scene_monitor_->getStateMonitor()->enableCopyDynamics(true);

    if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(parameters_.joint_model_group, ROBOT_STATE_WAIT_TIME))
    {
        ROS_FATAL("Timeout waiting for current state");
        exit(EXIT_FAILURE);
    }

    current_state_ = planning_scene_monitor->getStateMonitor()->getCurrentState();
    joint_model_group_ = current_state_->getJointModelGroup(parameters_.joint_model_group);
    n_variables_ = joint_model_group_->getVariableCount();

    // build kintroller map
    registerKintrollers();
    setIdleSetpoint();

    // start services
    switch_kintroller_service_ = pnh_.advertiseService("switch_kintrollers", &Kintrol::switchKintrollerService, this);

    // publish command signal for ros control
    command_pub_= nh_.advertise<std_msgs::Float64MultiArray>(parameters_.command_topic, parameters_.ros_queue_size);
    setpoint_sub_ = nh_.subscribe(parameters_.setpoint_topic, parameters_.ros_queue_size, &Kintrol::twistStampedCB, this);
}

void Kintrol::run()
{
    ros::Rate rate(parameters_.control_freq);
    std_msgs::Float64MultiArray msg;

    while (ros::ok())
    {
        Eigen::VectorXd cmd_out(n_variables_);
        current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
        
        active_kintroller_->update(setpoint_, current_state_, cmd_out);

        std::vector<double> output(n_variables_);
        Eigen::VectorXd::Map(&output[0], cmd_out.size()) = cmd_out;

        msg.data = output;
        command_pub_.publish(msg);
        rate.sleep();
    }
}

bool Kintrol::readParameters()
{
    std::size_t error = 0;

    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "command_topic", parameters_.command_topic);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "setpoint_topic", parameters_.setpoint_topic);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "joint_model_group", parameters_.joint_model_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "end_effector", parameters_.end_effector);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "control_freq", parameters_.control_freq);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "prop_gain", parameters_.prop_gain);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "ros_queue_size", parameters_.ros_queue_size);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "kintroller_names", parameters_.kintroller_names);
    
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);
    return true;
}

bool Kintrol::registerKintrollers()
{
    if (parameters_.kintroller_names.empty())
    {
        ROS_ERROR("No valid kintrollers specified");
        return false;
    }

    auto& robot_model = planning_scene_monitor_->getRobotModel();

    std::size_t error = 0;
    for (auto& name : parameters_.kintroller_names)
    {
        int type;
        std::string pose_frame;
        error += !rosparam_shortcuts::get(LOGNAME, pnh_, name + "/kintroller_type", type);
        error += !rosparam_shortcuts::get(LOGNAME, pnh_, name + "/pose_frame", pose_frame);
        rosparam_shortcuts::shutdownIfError(LOGNAME, error);
        kintrol::KinematicChain kc;
        getKinematicChain(robot_model, pose_frame, parameters_.end_effector, kc);

        if (type == KintrollerType::KINTROLLER) 
        {
            kintroller_map_[name] = 
                std::make_shared<Kintroller>(name, parameters_, kc);
        }
        else if (type == KintrollerType::COORDINATED_KINTROLLER) 
        {
            kintroller_map_[name] = 
                std::make_shared<CoordinatedKintroller>(name, parameters_, kc);
        }
        else if (type == KintrollerType::POSITIONER_KINTROLLER) 
        {
            std::shared_ptr<PositionerKintroller> kintroller = 
                std::make_shared<PositionerKintroller>(name, parameters_, kc);
            
            kintroller->initializeBaseFrames(robot_model);
            kintroller_map_[name] = kintroller; 
        }
    }

    std::string default_kintroller_name = parameters_.kintroller_names[0];
    ROS_INFO_STREAM("Defaulting to kintroller: " << default_kintroller_name);
    active_kintroller_ = kintroller_map_[default_kintroller_name];
    return true;
}

void Kintrol::setIdleSetpoint()
{
    robot_state::RobotStatePtr current_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    Eigen::Isometry3d eef_frame = current_state->getGlobalLinkTransform(parameters_.end_effector);
    Eigen::Isometry3d pose_frame = current_state->getGlobalLinkTransform(active_kintroller_->getPoseFrame());
    Eigen::Isometry3d pose_eef = pose_frame.inverse() * eef_frame;
    
    setpoint_.point.pose.position.x = pose_eef.translation().x();
    setpoint_.point.pose.position.y = pose_eef.translation().y();
    setpoint_.point.pose.position.z = pose_eef.translation().z();

    setpoint_.point.velocity.linear.x = 0.0;
    setpoint_.point.velocity.linear.y = 0.0;
    setpoint_.point.velocity.linear.z = 0.0;
    setpoint_.point.velocity.angular.x = 0.0;
    setpoint_.point.velocity.angular.y = 0.0;
    setpoint_.point.velocity.angular.z = 0.0;
}

/* Callbacks */
void Kintrol::twistStampedCB(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg)
{
    setpoint_ = *msg;
}

bool Kintrol::switchKintrollerService(SwitchKintroller::Request &req,
                             SwitchKintroller::Response &res)
{
    if (kintroller_map_.count(req.kintroller_name) > 0)
    {
        active_kintroller_ = kintroller_map_[req.kintroller_name];
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Failed to switch to kintroller: " << req.kintroller_name);
        return false;
    }
}

} // namespace kintrol