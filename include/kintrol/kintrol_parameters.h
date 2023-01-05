#pragma once
#include <string>

namespace kintrol
{
enum KintrollerType { KINTROLLER, COORDINATED_KINTROLLER };

struct KintrolParameters
{
    std::string command_topic;
    std::string setpoint_topic;
    std::string joint_model_group;
    std::string end_effector;
    std::string pose_frame;
    std::string orient_frame;
    double control_freq;
    double prop_gain;

    size_t ros_queue_size;
    int kintroller_type;

    // Kintroller parameters
    std::string positioner_joint_model_group;
    std::string positioner_command_topic;
    bool constant_orient;
};

} // namespace kintrol