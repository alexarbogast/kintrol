#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/TwistStamped.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>

#include "kintrol/trajectory_pub.h"

static double TRAVEL_VELOCITY = 100.0 / 1000.0; // mm/s

namespace kintrol
{
TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle& nh)
    : nh_(nh)
{
    std::string setpoint_topic;
    size_t ros_queue_size;

    ros::NodeHandle rpnh(nh_, "kintrol_server");
    rosparam_shortcuts::get("/kintrol_server", rpnh, "setpoint_topic", setpoint_topic);
    rosparam_shortcuts::get("/kintrol_server", rpnh, "control_freq", control_freq_);
    rosparam_shortcuts::get("/kintrol_server", rpnh, "ros_queue_size", ros_queue_size);

    // setpoint_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(setpoint_topic, ros_queue_size);
    setpoint_pub_ = nh_.advertise<moveit_msgs::CartesianTrajectoryPoint>(setpoint_topic, ros_queue_size);

    // create path
    EigenSTL::vector_Vector3d segments;
    segments.emplace_back(0.0, 0.0, 0.0);
    segments.emplace_back(0.0, 0.3, 0.0);
    segments.emplace_back(0.3, 0.3, 0.0);
    segments.emplace_back(0.3, -0.3, 0.0);
    segments.emplace_back(0.0, -0.3, 0.0);
    segments.emplace_back(0.0, 0.0, 0.0);

    path_.segments = segments;
    path_.reset_pointer();
    path_.find_times(TRAVEL_VELOCITY);
}

void TrajectoryPublisher::run()
{
    ros::Rate rate(control_freq_);
    moveit_msgs::CartesianTrajectoryPoint msg;

    auto& current_segment = path_.current_segment;
    auto elapsed_cache = path_.elapsed.begin();

    Eigen::Vector3d direction = *current_segment - *std::prev(current_segment);

    double segment_elapsed = 0.0;
    double previous = ros::Time::now().toSec();
    while (current_segment < path_.segments.end())
    {
        segment_elapsed += ros::Time::now().toSec() - previous;
        previous = ros::Time::now().toSec();

        if (segment_elapsed >=  *elapsed_cache)
        {
            ++current_segment;
            ++elapsed_cache;

            direction = *current_segment - *std::prev(current_segment);
            segment_elapsed = 0.0;
        }

        double perc_elapsed = segment_elapsed/(*elapsed_cache);
        auto position = *std::prev(current_segment) + perc_elapsed * direction;
        msg.point.pose.position.x = position.x();        
        msg.point.pose.position.y = position.y();
        msg.point.pose.position.z = position.z();       

        auto velocity_vector = direction / direction.norm() * TRAVEL_VELOCITY;
        msg.point.velocity.linear.x = velocity_vector.x();
        msg.point.velocity.linear.y = velocity_vector.y();
        msg.point.velocity.linear.z = velocity_vector.z();

        setpoint_pub_.publish(msg);
        rate.sleep();
    }

    msg.point.velocity.linear.x = 0.0;
    msg.point.velocity.linear.y = 0.0;
    msg.point.velocity.linear.z = 0.0;
    setpoint_pub_.publish(msg);
}

void TrajectoryPublisher::findSetpoint()
{

}

} // namespace kintrol

TrajectoryExecutionAction::TrajectoryExecutionAction(std::string name)
    : as_(nh_, name, boost::bind(&TrajectoryExecutionAction::executeCB, this, _1), false),
      action_name(name)
{
    as_.registerPreemptCallback(boost::bind(&TrajectoryExecutionAction::preemptCB, this));
    as_.start();
}

void TrajectoryExecutionAction::preemptCB()
{

}

void TrajectoryExecutionAction::executeCB(const kintrol::TrajectoryExecutionGoalConstPtr& goal)
{
    if (!as_.isActive() || as_.isPreemptRequested()) return;

}