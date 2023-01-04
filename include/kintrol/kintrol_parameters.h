#pragma once
#include <string>

namespace kintrol
{
struct KintrolParameters
{
    std::string command_topic;
    std::string setpoint_topic;
    std::string joint_model_group;
    std::string end_effector;
    std::string pose_frame;
    double control_freq;
    double prop_gain;

    size_t ros_queue_size;
};

} // namespace kintrol