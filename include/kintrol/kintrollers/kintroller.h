#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>

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


class CoordinatedKintroller : public KintrollerBase
{
public:
    CoordinatedKintroller(const KintrolParameters& params, const KinematicChain& kc);

    virtual void update(const Setpoint& setpoint,
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out) override;
private:
    void positionerCmdCB(const std_msgs::Float64MultiArrayConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;

    // number of positioner columns in left half of jacobian .
    unsigned int pos_cols_from_left; 
    std::vector<double> positioner_cmd_;
};

} // namespace kintrollers
} // namespace kintrol