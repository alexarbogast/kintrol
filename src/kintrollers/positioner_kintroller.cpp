#include "kintrol/kintrollers/positioner_kintroller.h"

const static std::string LOGNAME = "kintrol_server";

// TODO: make these parameters
const static double MAX_VELOCITY = 3.0; // rad/s
const static double LOWPASS_CUTOFF = 5.0; // Hz

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

    filter_.reconfigureFilter(LOWPASS_CUTOFF, 1 / parameters_.control_freq);
}

void PositionerKintroller::initialize(robot_model::RobotModelConstPtr robot_model)
{
    for (CoordinatedUnitContext& unit : coord_unit_contexts_)
    {
        const robot_model::JointModelGroup* jmg = robot_model->getJointModelGroup(unit.joint_model_group);
        unit.base_frame = jmg->getLinkModelNames()[0];
    }

    n_vars_ = robot_model->getJointModelGroup(parameters_.joint_model_group)->getVariableCount();
    output_history.resize(n_vars_);
}

void PositionerKintroller::update(const Setpoint& setpoint,
                                 robot_state::RobotStatePtr& robot_state,
                                 Eigen::VectorXd& cmd_out) 
{
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

    double output = K * eef_error;
    
    output = output > MAX_VELOCITY ? MAX_VELOCITY : output;
    output = output < -MAX_VELOCITY ? -MAX_VELOCITY : output;

    // low-pass filter output
    output = filter_.update(output);
    
    Eigen::VectorXd output_vec(n_vars_);
    output_vec << output;
    cmd_out = output_vec;

    output_history = output_vec;
    previous_ = ros::Time::now();
}

} // namespace kintrol