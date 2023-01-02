#pragma once
#include <ros/ros.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <actionlib/server/simple_action_server.h>
#include "kintrol/TrajectoryExecutionAction.h"

namespace  kintrol
{
struct Path
{
    Path() = default;

    Path(EigenSTL::vector_Vector3d segments)
        : segments(segments)
    {
        reset_pointer();
    }

    void reset_pointer()
    {
        current_segment = ++segments.begin();
    }

    void find_times(double travel_velocity)
    {
        elapsed.reserve(segments.size() - 1);

        for (size_t i = 1; i < segments.size(); i++)
        {
            double dist = (segments[i] - segments[i - 1]).norm();
            elapsed.emplace_back(dist / travel_velocity);
        }
    }

public:
    std::vector<double> elapsed;
    EigenSTL::vector_Vector3d segments;
    EigenSTL::vector_Vector3d::iterator current_segment;

};


class TrajectoryPublisher
{
public:
    TrajectoryPublisher(ros::NodeHandle& nh);
    void run();
private:
    void findSetpoint();
protected:
    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub_;

    Path path_;

    double control_freq_;
};

} // namespace  kintrol

class TrajectoryExecutionAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<kintrol::TrajectoryExecutionAction> as_;
    kintrol::TrajectoryExecutionActionFeedback feedback;
    kintrol::TrajectoryExecutionActionResult result;

    std::string action_name;

public:
    TrajectoryExecutionAction(std::string name);

    void preemptCB();
    void executeCB(const kintrol::TrajectoryExecutionGoalConstPtr& goal);
};
