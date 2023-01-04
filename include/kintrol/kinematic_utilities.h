#pragma once
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace kintrol
{
typedef std::vector<const robot_model::JointModel*> JointModelChain;

struct KinematicChain
{
    const robot_model::LinkModel* start_link;
    const robot_model::LinkModel* end_link; 
    JointModelChain jmc;
};

/* 
return a vector of joint models that create a chain between frames 
start_frame and end_frame
*/
void getKinematicChain(const robot_model::RobotModelConstPtr& robot_model,
                       const std::string& start_frame,
                       const std::string& end_frame,
                       KinematicChain& kc);

void getJacobian(const robot_state::RobotStatePtr& state, const KinematicChain& kc, Eigen::MatrixXd& jacobian);
void psuedoInverseJacobian(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& inverse);

} // namespace kintrol 