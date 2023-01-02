#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <std_msgs/Float64MultiArray.h>

#include "kintrol/kintrol.h"


static std::string JOINT_TOPIC = "joint_states";
static double ROBOT_STATE_WAIT_TIME = 10.0; // seconds

static std::string LOGNAME = "kintrol_server";

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

    // publish command signal for ros control
    command_pub_= nh_.advertise<std_msgs::Float64MultiArray>(parameters_.command_topic, parameters_.ros_queue_size);
    setpoint_sub_ = nh_.subscribe(parameters_.setpoint_topic, parameters_.ros_queue_size, &Kintrol::twistStampedCB, this);
}

void Kintrol::run()
{
    ros::Rate rate(parameters_.control_freq);
    
    std_msgs::Float64MultiArray msg;
    
    Eigen::VectorXd vel(6);
    vel << setpoint_.point.velocity.linear.x, 
           setpoint_.point.velocity.linear.y,
           setpoint_.point.velocity.linear.z,
           setpoint_.point.velocity.angular.x,
           setpoint_.point.velocity.angular.y,
           setpoint_.point.velocity.angular.z; 


    while (ros::ok())
    {
        vel << setpoint_.point.velocity.linear.x, 
               setpoint_.point.velocity.linear.y,
               setpoint_.point.velocity.linear.z,
               setpoint_.point.velocity.angular.x,
               setpoint_.point.velocity.angular.y,
               setpoint_.point.velocity.angular.z; 

        current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
        Eigen::MatrixXd jacobian = current_state_->getJacobian(joint_model_group_);
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

bool Kintrol::readParameters()
{
    std::size_t error = 0;

    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "command_topic", parameters_.command_topic);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "control_freq", parameters_.control_freq);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "joint_model_group", parameters_.joint_model_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "ros_queue_size", parameters_.ros_queue_size);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "setpoint_topic", parameters_.setpoint_topic);

    rosparam_shortcuts::shutdownIfError(LOGNAME, error);
    return true;
}

void Kintrol::psuedoInverseJacobian(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& inverse)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian,  Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();
    inverse = svd.matrixV() * sigma.inverse() * svd.matrixU().transpose();
}

void Kintrol::twistStampedCB(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg)
{
    setpoint_ = *msg;
}

} // namespace servo