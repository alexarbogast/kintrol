#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <std_msgs/Float64MultiArray.h>

#include "kintrol/kintrol.h"


static std::string JOINT_TOPIC = "joint_states";
static double ROBOT_STATE_WAIT_TIME = 10.0; // seconds

static std::string LOGNAME = "kintrol_server";

static const double PROP_GAIN = 0.1;

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

    getKinematicChain(current_state_->getRobotModel(), 
                      parameters_.end_effector,
                      parameters_.pose_frame);

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

    std::cout << base_frame.translation() << std::endl;
    std::cout << pose_frame.translation() << std::endl;

    Eigen::Isometry3d base_to_pose = pose_frame.inverse() * base_frame;

    while (ros::ok())
    {
        extractVelocity(vel);
        extractPosition(pose);

        current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
        Eigen::Isometry3d eef_frame = current_state_->getFrameTransform(parameters_.end_effector);
        eef_frame = pose_frame.inverse() * eef_frame;

        auto pose_error = PROP_GAIN * (pose - eef_frame.translation());

        vel[0] += pose_error.x();
        vel[1] += pose_error.y();
        vel[2] += pose_error.z();

        //Eigen::MatrixXd jacobian = current_state_->getJacobian(joint_model_group_);
        Eigen::MatrixXd jacobian;
        getJacobian(current_state_, parameters_.end_effector, parameters_.pose_frame, jacobian);
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
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "end_effector", parameters_.end_effector);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "pose_frame", parameters_.pose_frame);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "ros_queue_size", parameters_.ros_queue_size);
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "setpoint_topic", parameters_.setpoint_topic);

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

// this can be seperated into another file
void Kintrol::getJacobian(robot_state::RobotStatePtr& state,
                          const std::string& end_frame, 
                          const std::string& start_frame,
                          Eigen::MatrixXd& jacobian)
{
    auto& robot_model = state->getRobotModel();

    const robot_model::LinkModel* end_link = robot_model->getLinkModel(end_frame);
    const robot_model::LinkModel* start_link = robot_model->getLinkModel(start_frame);

    Eigen::Isometry3d reference_transform =
        start_link ? state->getGlobalLinkTransform(start_link).inverse() : Eigen::Isometry3d::Identity(); // T_base_world
    int rows = 6;
    int columns = joint_model_chain_.size();
    jacobian = Eigen::MatrixXd::Zero(rows, columns);

    Eigen::Isometry3d link_transform = reference_transform * state->getGlobalLinkTransform(end_link);
    Eigen::Vector3d point_transform = link_transform.translation();
    
    Eigen::Vector3d joint_axis; 
    Eigen::Isometry3d joint_transform;    

    unsigned int joint_index = 0;
    for (const auto& joint_model : joint_model_chain_)
    {
        if (joint_model->getType() == robot_model::JointModel::REVOLUTE)
        {
            const std::string& link_name = joint_model->getChildLinkModel()->getName();
            joint_transform = reference_transform * state->getGlobalLinkTransform(link_name);
            joint_axis = joint_transform.rotation() * 
                         static_cast<const robot_model::RevoluteJointModel*>(joint_model)->getAxis();

            jacobian.block<3, 1>(0, joint_index) = 
                jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
            jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
        }
        joint_index++;
    }
}

void Kintrol::getKinematicChain(const robot_model::RobotModelConstPtr& robot_model, 
                                const std::string& end_frame,
                                const std::string& start_frame)
{
    joint_model_chain_.clear();

    const robot_state::LinkModel* end_link = robot_model->getLinkModel(end_frame);
    const robot_state::LinkModel* start_link = robot_model->getLinkModel(start_frame);

    std::vector<std::string> end_link_names;
    std::vector<std::string> start_link_names;
    while (end_link)
    {
        end_link_names.push_back(end_link->getName());
        end_link = end_link->getParentLinkModel();
    }

    while (start_link)
    {
        start_link_names.push_back(start_link->getName());
        start_link = start_link->getParentLinkModel();
    }
    
    std::vector<std::string>::iterator start_link_it = --start_link_names.end();
    std::vector<std::string>::iterator end_link_it = --end_link_names.end();
    
    for (; start_link_it != start_link_names.begin(); start_link_it--)
    {

        if (*std::prev(start_link_it) == *std::prev(end_link_it))
        {
            start_link_names.erase(start_link_it);
            end_link_names.erase(end_link_it);
        }
        else
        {
            // leave one common root
            start_link_names.erase(start_link_it);
            break;
        }
        end_link_it--;
    }

    std::vector<std::string> frame_chain;
    frame_chain.reserve(start_link_names.size() + end_link_names.size());
    frame_chain.insert(frame_chain.end(), start_link_names.begin(), start_link_names.end());
    frame_chain.insert(frame_chain.end(), end_link_names.rbegin(), end_link_names.rend());

    for (std::string& link_name : frame_chain)
    {
        const robot_model::LinkModel* link_model = robot_model->getLinkModel(link_name);
        const robot_model::JointModel* pjm = link_model->getParentJointModel();

        if (pjm->getVariableCount() > 0)
            joint_model_chain_.emplace_back(pjm);
    }
}

void Kintrol::psuedoInverseJacobian(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& inverse)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian,  Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();
    inverse = svd.matrixV() * sigma.inverse() * svd.matrixU().transpose();
}

/* Callbacks */
void Kintrol::twistStampedCB(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg)
{
    setpoint_ = *msg;
}

} // namespace servo