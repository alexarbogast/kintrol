#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>

#include <kintrol/kintrol_parameters.h> 
#include <kintrol/kinematic_utilities.h>

namespace kintrol
{
namespace kintrollers
{
typedef moveit_msgs::CartesianTrajectoryPoint Setpoint;

class KintrollerBase
{
public:
    KintrollerBase(const KintrolParameters& params, const KinematicChain& kc);

    virtual void update(const Setpoint& setpoint, 
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out) = 0;

protected:
    void extractPosition(const Setpoint& setpoint, Eigen::VectorXd& position);
    void extractVelocity(const Setpoint& setpoint, Eigen::VectorXd& velocity);
protected:
    KintrolParameters parameters_;
    KinematicChain kinematic_chain_;

    Eigen::Isometry3d pose_frame_;
};


class Kintroller : public KintrollerBase
{
public:
    Kintroller(const KintrolParameters& params, const KinematicChain& kc);

    virtual void update(const Setpoint& setpoint, 
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out) override; 
};

} // namespace kintrollers
} // namespace kintrol