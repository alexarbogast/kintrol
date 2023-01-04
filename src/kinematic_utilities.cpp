#include "kintrol/kinematic_utilities.h"

namespace kintrol
{

void getKinematicChain(const robot_model::RobotModelConstPtr& robot_model,
                       const std::string& start_frame,
                       const std::string& end_frame,
                       KinematicChain& kc)
{
    kc.jmc.clear();

    const robot_state::LinkModel* end_link = robot_model->getLinkModel(end_frame);
    const robot_state::LinkModel* start_link = robot_model->getLinkModel(start_frame);

    kc.start_link = start_link;
    kc.end_link = end_link;

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
            kc.jmc.emplace_back(pjm);
    }
}

void getJacobian(const robot_state::RobotStatePtr& state, const KinematicChain& kc, Eigen::MatrixXd& jacobian)
{
    Eigen::Isometry3d reference_transform =
        kc.start_link ? state->getGlobalLinkTransform(kc.start_link).inverse() : Eigen::Isometry3d::Identity();
    int rows = 6;
    int columns = kc.jmc.size();
    jacobian = Eigen::MatrixXd::Zero(rows, columns);

    Eigen::Isometry3d link_transform = reference_transform * state->getGlobalLinkTransform(kc.end_link);
    Eigen::Vector3d point_transform = link_transform.translation();
    
    Eigen::Vector3d joint_axis; 
    Eigen::Isometry3d joint_transform;   

    unsigned int joint_index = 0;
    for (const auto& joint_model : kc.jmc)
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

void psuedoInverseJacobian(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& inverse)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian,  Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();
    inverse = svd.matrixV() * sigma.inverse() * svd.matrixU().transpose();
}


} // namespace kintrol