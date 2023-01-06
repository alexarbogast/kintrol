#include "kintrol/kintrollers/positioner_kintroller.h"

const static std::string LOGNAME = "kintrol_server";

namespace kintrol
{
PositionerKintroller::PositionerKintroller(const std::string& name, 
                                           const KintrolParameters& params, 
                                           const KinematicChain& kc)
    : KintrollerBase(name, params, kc)
{
    std::vector<std::string> coord_group_names;
    std::size_t error = 0;
    ros::NodeHandle pnh("~" + name);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "coord_group_names", coord_group_names);

    // get coordinated robot joint model groups
    for (auto& group_name : coord_group_names)
    {
        CoordinatedUnitContext cuc;

        ros::NodeHandle nh("/" + group_name + "/" + LOGNAME);
        std::string jmg_name; 
        error += !rosparam_shortcuts::get(LOGNAME, nh, "joint_model_group", cuc.joint_model_group);
        error += !rosparam_shortcuts::get(LOGNAME, nh, "end_effector", cuc.end_effector);
        rosparam_shortcuts::shutdownIfError(LOGNAME, error);
        
        coord_unit_contexts_.push_back(cuc);
    }
}

void PositionerKintroller::initializeBaseFrames(robot_model::RobotModelConstPtr robot_model)
{
    for (CoordinatedUnitContext& unit : coord_unit_contexts_)
    {
        const robot_model::JointModelGroup* jmg = robot_model->getJointModelGroup(unit.joint_model_group);
        unit.base_frame = jmg->getLinkModelNames()[0];
    }
}

void PositionerKintroller::update(const Setpoint& setpoint,
                                 robot_state::RobotStatePtr& robot_state,
                                 Eigen::VectorXd& cmd_out) 
{
    static unsigned int n_vars = robot_state->getJointModelGroup(parameters_.joint_model_group)->getVariableCount();
    static double K = 2.0;

    double eef_error = 0.0;
    for (auto& unit : coord_unit_contexts_)
    {
        // find error between base frame and eef vector
        Eigen::Isometry3d eef_frame = robot_state->getGlobalLinkTransform(unit.end_effector);
        Eigen::Isometry3d base_frame = robot_state->getGlobalLinkTransform(unit.base_frame);

        eef_frame.translation().z() = 0.0;
        base_frame.translation().z() = 0.0;

        Eigen::Vector3d eef_frame_trans = eef_frame.translation() / eef_frame.translation().norm(); 
        Eigen::Vector3d base_frame_trans = base_frame.translation() / base_frame.translation().norm(); 

        double error = eef_frame_trans.cross(base_frame_trans).z();
        eef_error += error;
    }

    std::cout << eef_error << std::endl << std::endl;

    Eigen::VectorXd cmd(n_vars);
    cmd << K * eef_error;

    cmd_out = cmd;
}

} // namespace kintrol