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

    getKinematicChain(current_state_->getRobotModel(), 
                      parameters_.pose_frame,
                      parameters_.end_effector,
                      kinematic_chain_);

    kintroller_ = std::make_unique<kintrollers::Kintroller>(parameters_, kinematic_chain_);
    setIdleSetpoint();

    // publish command signal for ros control
    command_pub_= nh_.advertise<std_msgs::Float64MultiArray>(parameters_.command_topic, parameters_.ros_queue_size);
    setpoint_sub_ = nh_.subscribe(parameters_.setpoint_topic, parameters_.ros_queue_size, &Kintrol::twistStampedCB, this);
}

void Kintrol::run()
{
    ros::Rate rate(parameters_.control_freq);
    
    std_msgs::Float64MultiArray msg;
    
    Eigen::VectorXd vel(6); 
    Eigen::VectorXd pose(3);
    extractVelocity(vel);
    extractPosition(pose);
    
    std::string base_frame_name = joint_model_group_->getLinkModelNames()[0];
    const robot_model::JointModel* root_joint_model = joint_model_group_->getJointModels()[0];
    const robot_model::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
    Eigen::Isometry3d base_frame = current_state_->getFrameTransform(base_frame_name);
    Eigen::Isometry3d pose_frame = current_state_->getFrameTransform(parameters_.pose_frame);

    Eigen::Isometry3d base_to_pose = pose_frame.inverse() * base_frame;

    while (ros::ok())
    {
        extractVelocity(vel);
        extractPosition(pose);

        current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
        Eigen::Isometry3d eef_frame = current_state_->getFrameTransform(parameters_.end_effector);
        eef_frame = pose_frame.inverse() * eef_frame;

        auto pose_error = parameters_.prop_gain * (pose - eef_frame.translation());

        vel[0] += pose_error.x();
        vel[1] += pose_error.y();
        vel[2] += pose_error.z();

        Eigen::MatrixXd jacobian;
        getJacobian(current_state_, kinematic_chain_, jacobian);
        Eigen::MatrixXd psuedo_inverse;
        psuedoInverseJacobian(jacobian, psuedo_inverse);

        Eigen::VectorXd cmd = psuedo_inverse * vel;
        std::vector<double> output(6);
        Eigen::VectorXd::Map(&output[0], cmd.size()) = cmd;

        msg.data = output;
        command_pub_.publish(msg);
        rate.sleep();
    }
}

void Kintrol::run2()
{
    ros::Rate rate(parameters_.control_freq);
    std_msgs::Float64MultiArray msg;

    while (ros::ok())
    {
        Eigen::VectorXd cmd_out(n_variables_);
        current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
        
        kintroller_->update(setpoint_, current_state_, cmd_out);

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
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "pose_frame", parameters_.pose_frame);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "control_freq", parameters_.control_freq);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "prop_gain", parameters_.prop_gain);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "ros_queue_size", parameters_.ros_queue_size);

    rosparam_shortcuts::shutdownIfError(LOGNAME, error);
    return true;
}

void Kintrol::setIdleSetpoint()
{
    robot_state::RobotStatePtr current_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    Eigen::Isometry3d eef_frame = current_state->getGlobalLinkTransform(parameters_.end_effector);
    Eigen::Isometry3d pose_frame = current_state->getGlobalLinkTransform(parameters_.pose_frame);
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

void Kintrol::extractPosition(Eigen::VectorXd& pose)
{
    pose << setpoint_.point.pose.position.x,
            setpoint_.point.pose.position.y,
            setpoint_.point.pose.position.z;
}

void Kintrol::extractVelocity(Eigen::VectorXd& vel)
{
    vel << setpoint_.point.velocity.linear.x, 
           setpoint_.point.velocity.linear.y,
           setpoint_.point.velocity.linear.z,
           setpoint_.point.velocity.angular.x,
           setpoint_.point.velocity.angular.y,
           setpoint_.point.velocity.angular.z;
}

/* Callbacks */
void Kintrol::twistStampedCB(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg)
{
    setpoint_ = *msg;
}

} // namespace servo