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

// method for calculating the pseudo-Inverse as recommended by Eigen developers
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > 
           tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

/* deprecated */
void psuedoInverseJacobian(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& inverse);

} // namespace kintrol 