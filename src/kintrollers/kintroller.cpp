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
    Eigen::MatrixXd psuedo_inverse;
    psuedoInverseJacobian(jacobian, psuedo_inverse);

    cmd_out = psuedo_inverse * vel;
}

} // namespace kintrollers
} // namespace kintrol