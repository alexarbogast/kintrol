#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Core>

#include "kintrol/kinematic_utilities.h"

typedef std::vector<const robot_model::JointModel*> JointModelChain;

void getKinematicChain(const robot_model::RobotModelConstPtr& robot_model, 
                       const std::string& end_frame,
                       const std::string& start_frame,
                       JointModelChain& jmc)
{
    const robot_model::JointModelGroup* jmg = robot_model->getJointModelGroup("hydra_planning_group");
    const robot_model::LinkModel* end_link = robot_model->getLinkModel(end_frame);
    const robot_model::LinkModel* start_link = robot_model->getLinkModel(start_frame);

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
            jmc.emplace_back(pjm);
    }
}

void getJacobian(robot_state::RobotStatePtr& state,
                 const std::string& end_frame,
                 const std::string& start_frame,
                 Eigen::MatrixXd& jacobian)
{
    auto& robot_model = state->getRobotModel();

    JointModelChain jmc;
    getKinematicChain(robot_model, end_frame, start_frame, jmc);

    const robot_model::LinkModel* end_link = robot_model->getLinkModel(end_frame);
    const robot_model::LinkModel* start_link = robot_model->getLinkModel(start_frame);

    Eigen::Isometry3d reference_transform =
        start_link ? state->getGlobalLinkTransform(start_link).inverse() : Eigen::Isometry3d::Identity(); // T_base_world
    int rows = 6;
    int columns = jmc.size();
    jacobian = Eigen::MatrixXd::Zero(rows, columns);

    Eigen::Isometry3d link_transform = reference_transform * state->getGlobalLinkTransform(end_link);
    Eigen::Vector3d point_transform = link_transform.translation();
    
    Eigen::Vector3d joint_axis; 
    Eigen::Isometry3d joint_transform;    

    unsigned int joint_index = 0;
    for (const auto& joint_model : jmc)
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group_interface_("hydra_planning_group");
    robot_model::RobotStatePtr state = move_group_interface_.getCurrentState();

    const std::string start_frame = "positioner";
    const std::string end_frame = "rob1_tool0";

    kintrol::KinematicChain kc;
    kintrol::getKinematicChain(state->getRobotModel(), start_frame, end_frame, kc);

    // Set state
    state->setJointGroupPositions("hydra_planning_group", {0.0, -0.4, 0.7, 0, 1.3, 0.0, 0, -0.4, 0.7, 0, 1.3, 0.0, 0, -0.4, 0.7, 0, 1.3, 0.0, 0.5});

    // Find jacobian
    Eigen::MatrixXd jacobian_positioner; 
    kintrol::getJacobian(state, kc, jacobian_positioner);
    std::cout << "Positioner Jacobian:" << std::endl;
    std::cout << jacobian_positioner << std::endl;
    
    Eigen::MatrixXd pos_jac = jacobian_positioner.leftCols(1);
    Eigen::MatrixXd rob_jac = jacobian_positioner.rightCols(6);

    Eigen::MatrixXd jacobian_world;
    getJacobian(state, end_frame, "world", jacobian_world);
    std::cout << "World Jacobian" << std::endl;
    std::cout << jacobian_world << std::endl;

    Eigen::MatrixXd rob_jac_inv;
    kintrol::psuedoInverseJacobian(rob_jac, rob_jac_inv);
    
    Eigen::MatrixXd new_pinv = kintrol::pseudoInverse<Eigen::MatrixXd>(rob_jac);
    //std::cout << new_pinv << std::endl;

    Eigen::MatrixXd pinv = rob_jac.completeOrthogonalDecomposition().pseudoInverse();
    //std::cout << pinv << std::endl;

    Eigen::VectorXd pos_cmd(1);
    pos_cmd << 3.0;

    Eigen::VectorXd vel(6);
    vel << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;

    auto& cmd_out = new_pinv * (vel - (pos_jac * pos_cmd));
    std::cout << cmd_out << std::endl;
}