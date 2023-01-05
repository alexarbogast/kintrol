#include "kintrol/kintrollers/kintroller.h"

namespace kintrol
{
namespace kintrollers
{

KintrollerBase::KintrollerBase(const KintrolParameters& params, const KinematicChain& kc)
    : parameters_(params), kinematic_chain_(kc)
{
}

void KintrollerBase::extractPosition(const Setpoint& setpoint, Eigen::VectorXd& position)
{
    position << setpoint.point.pose.position.x,
                setpoint.point.pose.position.y,
                setpoint.point.pose.position.z;
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

Kintroller::Kintroller(const KintrolParameters& params, const KinematicChain& kc)
    : KintrollerBase(params, kc)
{
}

void Kintroller::update(const Setpoint& setpoint, 
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out)
{
    Eigen::Isometry3d pose_frame = robot_state->getFrameTransform(parameters_.pose_frame);
    Eigen::Isometry3d eef_frame = robot_state->getFrameTransform(parameters_.end_effector);
    eef_frame = pose_frame.inverse() * eef_frame;

    Eigen::VectorXd pose(3);
    Eigen::VectorXd vel(6);
    extractPosition(setpoint, pose);
    extractVelocity(setpoint, vel);

    auto pose_error = parameters_.prop_gain * (pose - eef_frame.translation());
    vel[0] += pose_error.x();
    vel[1] += pose_error.y();
    vel[2] += pose_error.z();

    Eigen::MatrixXd jacobian;
    getJacobian(robot_state, kinematic_chain_, jacobian);
    Eigen::MatrixXd psuedo_inverse = pseudoInverse(jacobian);
    //psuedoInverseJacobian(jacobian, psuedo_inverse);

    cmd_out = psuedo_inverse * vel;
}


CoordinatedKintroller::CoordinatedKintroller(const KintrolParameters& params, 
                                             const KinematicChain& kc)
    : KintrollerBase(params, kc)
{
    positioner_cmd_.resize(1, 0.0);

    cmd_sub_ = nh_.subscribe(parameters_.positioner_command_topic,
                             parameters_.ros_queue_size,
                             &CoordinatedKintroller::positionerCmdCB,
                             this);

    std::string orientation_frame = "positioner_static";

}

void CoordinatedKintroller::update(const Setpoint& setpoint,
                                   robot_state::RobotStatePtr& robot_state,
                                   Eigen::VectorXd& cmd_out)
{
    Eigen::Isometry3d pose_frame = robot_state->getFrameTransform(parameters_.pose_frame);
    Eigen::Isometry3d eef_frame = robot_state->getFrameTransform(parameters_.end_effector);
    eef_frame = pose_frame.inverse() * eef_frame;

    Eigen::VectorXd pose(3);
    Eigen::VectorXd vel(6);
    extractPosition(setpoint, pose);
    extractVelocity(setpoint, vel);

    auto pose_error = parameters_.prop_gain * (pose - eef_frame.translation());
    vel[0] += pose_error.x();
    vel[1] += pose_error.y();
    vel[2] += pose_error.z();

    // find jacobian of combined robot + positioner
    Eigen::MatrixXd jacobian;
    getJacobian(robot_state, kinematic_chain_, jacobian);

    // spererate jacobian into robot and positioner parts
    Eigen::MatrixXd pos_jac = jacobian.leftCols(1);
    Eigen::MatrixXd rob_jac = jacobian.rightCols(6);

    // TODO: keep constant world frame orientation
    if (parameters_.constant_orient)
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

} // namespace kintrollers
} // namespace kintrol