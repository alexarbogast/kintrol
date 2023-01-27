#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <eigen_conversions/eigen_msg.h>
#include "kintrol/kintrollers/kintroller.h"

const static std::string LOGNAME = "kintrol_server";

namespace kintrol
{
KintrollerBase::KintrollerBase(const std::string& name, const KintrolParameters& params, const KinematicChain& kc)
    : name_(name), parameters_(params), kinematic_chain_(kc)
{
    pose_frame_ = kinematic_chain_.start_link->getName();
}

void KintrollerBase::extractPosition(const Setpoint& setpoint, Eigen::Isometry3d& position)
{
    tf::poseMsgToEigen(setpoint.point.pose, position);
}

void KintrollerBase::extractVelocity(const Setpoint& setpoint, Eigen::VectorXd& velocity)
{
    velocity << setpoint.point.velocity.linear.x, 
                setpoint.point.velocity.linear.y,
                setpoint.point.velocity.linear.z,
                setpoint.point.velocity.angular.x,
                setpoint.point.velocity.angular.y,
                setpoint.point.velocity.angular.z;
}

Kintroller::Kintroller(const std::string& name, const KintrolParameters& params, const KinematicChain& kc)
    : KintrollerBase(name, params, kc)
{
}

void Kintroller::update(const Setpoint& setpoint, 
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out)
{
    Eigen::Isometry3d pose_frame = robot_state->getFrameTransform(pose_frame_);
    Eigen::Isometry3d eef_frame = robot_state->getFrameTransform(parameters_.end_effector);
    Eigen::Isometry3d eef_frame_base = pose_frame.inverse() * eef_frame;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::VectorXd vel(6);
    extractPosition(setpoint, pose);
    extractVelocity(setpoint, vel);

    auto pose_error = parameters_.prop_gain * (pose.translation() - eef_frame_base.translation());
    vel[0] += pose_error.x();
    vel[1] += pose_error.y();
    vel[2] += pose_error.z();

    Eigen::Quaterniond pose_quat(pose.linear());
    Eigen::Quaterniond eef_quat(eef_frame_base.linear());
    static double Ko = 1.0;

    auto orient_error = pose_quat * eef_quat.inverse();
    vel[3] += Ko * orient_error.x();
    vel[4] += Ko * orient_error.y();
    vel[5] += Ko * orient_error.z();

    Eigen::MatrixXd jacobian;
    getJacobian(robot_state, kinematic_chain_, jacobian);
    Eigen::MatrixXd psuedo_inverse = pseudoInverse(jacobian);

    cmd_out = psuedo_inverse * vel;
}


CoordinatedKintroller::CoordinatedKintroller(const std::string& name,
                                             const KintrolParameters& params, 
                                             const KinematicChain& kc)
    : KintrollerBase(name, params, kc)
{
    positioner_cmd_.resize(1, 0.0);

    std::string positioner_command_topic;
    std::size_t error = 0;
    ros::NodeHandle pnh("~" + name);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "positioner_command_topic", positioner_command_topic);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "constant_orient", constant_orient_);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    cmd_sub_ = nh_.subscribe(positioner_command_topic,
                             parameters_.ros_queue_size,
                             &CoordinatedKintroller::positionerCmdCB,
                             this);
}

void CoordinatedKintroller::update(const Setpoint& setpoint,
                                   robot_state::RobotStatePtr& robot_state,
                                   Eigen::VectorXd& cmd_out)
{
    Eigen::Isometry3d pose_frame = robot_state->getFrameTransform(pose_frame_);
    Eigen::Isometry3d eef_frame = robot_state->getFrameTransform(parameters_.end_effector);
    Eigen::Isometry3d eef_frame_base = pose_frame.inverse() * eef_frame;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::VectorXd vel(6);
    extractPosition(setpoint, pose);
    extractVelocity(setpoint, vel);

    auto pose_error = parameters_.prop_gain * (pose.translation() - eef_frame_base.translation());
    vel[0] += pose_error.x();
    vel[1] += pose_error.y();
    vel[2] += pose_error.z();

    Eigen::Quaterniond pose_quat(pose.linear());
    Eigen::Quaterniond eef_quat(eef_frame.linear());
    static double Ko = 1.0;

    auto orient_error = pose_quat * eef_quat.inverse();
    vel[3] += Ko * orient_error.x();
    vel[4] += Ko * orient_error.y();
    vel[5] += Ko * orient_error.z();

    // find jacobian of combined robot + positioner
    Eigen::MatrixXd jacobian;
    getJacobian(robot_state, kinematic_chain_, jacobian);

    // spererate jacobian into robot and positioner parts
    Eigen::MatrixXd pos_jac = jacobian.leftCols(1);
    Eigen::MatrixXd rob_jac = jacobian.rightCols(6);

    // TODO: keep constant world frame orientation
    if (constant_orient_)
    {
        Eigen::Vector3d zeros(0, 0, 0);
        pos_jac.block<3, 1>(3, 0) = zeros;
    }

    Eigen::VectorXd pos_cmd = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>
                              (positioner_cmd_.data(), positioner_cmd_.size());

    Eigen::MatrixXd rob_jac_inv = pseudoInverse(rob_jac);

    // why is this a plus
    cmd_out = rob_jac_inv * (vel + (pos_jac * pos_cmd));
}

void CoordinatedKintroller::positionerCmdCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    positioner_cmd_ = msg->data;
}

} // namespace kintrol