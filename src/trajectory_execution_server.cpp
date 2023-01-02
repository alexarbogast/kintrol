#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>
#include <eigen_conversions/eigen_msg.h>

#include "kintrol/TrajectoryExecutionAction.h"

static const std::string ACTION_NAME = "trajectory_execution_action";
static double TRAVEL_VELOCITY = 100.0 / 1000.0; // mm/s

class TrajectoryExecutionAction
{
private:
    typedef actionlib::SimpleActionServer<kintrol::TrajectoryExecutionAction> TrajectoryExecutionServer;
    typedef TrajectoryExecutionServer::GoalHandle GoalHandle;
public:
    TrajectoryExecutionAction(ros::NodeHandle& nh)
        : nh_(nh),
          as_(nh_, ACTION_NAME,
              boost::bind(&TrajectoryExecutionAction::executeCB, this, _1),
              false),
          has_active_goal_(false)
    {
        std::string setpoint_topic;
        size_t ros_queue_size;

        ros::NodeHandle rpnh(nh_, "kintrol_server");
        rosparam_shortcuts::get("/kintrol_server", rpnh, "setpoint_topic", setpoint_topic);
        rosparam_shortcuts::get("/kintrol_server", rpnh, "control_freq", control_freq_);
        rosparam_shortcuts::get("/kintrol_server", rpnh, "ros_queue_size", ros_queue_size);

        setpoint_pub_ = nh_.advertise<moveit_msgs::CartesianTrajectoryPoint>(setpoint_topic, ros_queue_size);

        as_.registerPreemptCallback(boost::bind(&TrajectoryExecutionAction::preemptCB, this));
        as_.start();
    }

private:
    void preemptCB()
    {
        
    }

    void executeCB(const kintrol::TrajectoryExecutionGoalConstPtr& goal)
    {
        bool success = true;

        ros::Rate rate(control_freq_);
        moveit_msgs::CartesianTrajectoryPoint msg;

        const auto& path = goal->path.poses;
        auto current_segment = std::next(path.begin());

        Eigen::Isometry3d start_pose, end_pose;
        tf::poseMsgToEigen(*current_segment, end_pose);
        tf::poseMsgToEigen(*std::prev(current_segment), start_pose);

        Eigen::Vector3d direction = end_pose.translation() - start_pose.translation();
        double segment_duration = direction.norm() / TRAVEL_VELOCITY;

        double segment_elapsed = 0.0;
        double previous = ros::Time::now().toSec();
        while (current_segment < path.end())
        {
            segment_elapsed += ros::Time::now().toSec() - previous;
            previous = ros::Time::now().toSec();

            if (segment_elapsed > segment_duration)
            {
                ++current_segment;
                
                tf::poseMsgToEigen(*current_segment, end_pose);
                tf::poseMsgToEigen(*std::prev(current_segment), start_pose);
                
                direction = end_pose.translation() - start_pose.translation();
                segment_duration = direction.norm() / TRAVEL_VELOCITY;
                segment_elapsed = 0.0;
            }

            double perc_elapsed = segment_elapsed / segment_duration;
            auto position = start_pose.translation() + (perc_elapsed * direction);
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

        if (success)
        {
            as_.setSucceeded();
        }
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub_;
    TrajectoryExecutionServer as_;

    bool has_active_goal_;
    GoalHandle active_goal_;

    double control_freq_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_execution");
    ros::NodeHandle nh;
    TrajectoryExecutionAction trajectory_execution(nh);
    
    ros::spin();
}