#pragma once

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>

#include <kintrol/kintrol_parameters.h> 
#include <kintrol/kinematic_utilities.h>

namespace kintrol
{
typedef moveit_msgs::CartesianTrajectoryPoint Setpoint;

class KintrollerBase
{
public:
    KintrollerBase(const std::string& name, const KintrolParameters& params, const KinematicChain& kc);

    virtual void update(const Setpoint& setpoint, 
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out) = 0;

    inline const std::string& getPoseFrame() const { return pose_frame_; };

protected:
    void extractPosition(const Setpoint& setpoint, Eigen::Isometry3d& position);
    void extractVelocity(const Setpoint& setpoint, Eigen::VectorXd& velocity);
protected:
    std::string name_;
    ros::NodeHandle nh_;

    KintrolParameters parameters_;
    KinematicChain kinematic_chain_;

    std::string pose_frame_;
};
typedef std::shared_ptr<KintrollerBase> KintrollerPtr;


class Kintroller : public KintrollerBase
{
public:
    Kintroller(const std::string& name, const KintrolParameters& params, const KinematicChain& kc);

    virtual void update(const Setpoint& setpoint, 
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out) override; 
};


class CoordinatedKintroller : public KintrollerBase
{
public:
    CoordinatedKintroller(const std::string& name, const KintrolParameters& params, const KinematicChain& kc);

    virtual void update(const Setpoint& setpoint,
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out) override;
private:
    void positionerCmdCB(const std_msgs::Float64MultiArrayConstPtr& msg);
private:
    ros::Subscriber cmd_sub_;

    bool constant_orient_;

    // number of positioner columns in left half of jacobian .
    unsigned int pos_cols_from_left; 
    std::vector<double> positioner_cmd_;
};

} // namespace kintrol