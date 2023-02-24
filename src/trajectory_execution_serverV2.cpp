#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>

#include <kintrol/TrajectoryExecutionAction.h>
#include <kintrol/PosVelSetpoint.h>


static const std::string ACTION_NAME = "trajectory_execution_action";

void poseToSetpoint(const Eigen::Vector3d& pos, 
                    const Eigen::Quaterniond& orient,
                    kintrol::PosVelSetpoint& setpoint) {
    setpoint.pose.position.x = pos.x();
    setpoint.pose.position.y = pos.y();
    setpoint.pose.position.z = pos.z();

    setpoint.pose.orientation.w = orient.w();
    setpoint.pose.orientation.x = orient.x();
    setpoint.pose.orientation.y = orient.y();
    setpoint.pose.orientation.z = orient.z();
}

void velocityToSetpoint(const Eigen::Vector3d& linear,
                        const Eigen::Vector3d& angular,
                        kintrol::PosVelSetpoint& setpoint) {
    setpoint.twist.twist.linear.x = linear.x();
    setpoint.twist.twist.linear.y = linear.y();
    setpoint.twist.twist.linear.z = linear.z();

    setpoint.twist.twist.angular.x = angular.x();
    setpoint.twist.twist.angular.y = angular.y();
    setpoint.twist.twist.angular.z = angular.z();
}

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

        rosparam_shortcuts::get(nh_.getNamespace(), nh_, "setpoint_topic", setpoint_topic);
        rosparam_shortcuts::get(nh_.getNamespace(), nh_, "control_freq", control_freq_);
        rosparam_shortcuts::get(nh_.getNamespace(), nh_, "ros_queue_size", ros_queue_size);

        setpoint_pub_ = nh_.advertise<kintrol::PosVelSetpoint>(setpoint_topic, ros_queue_size);

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

        const auto& path = goal->path.poses;
        auto current_segment = std::next(path.begin());

        Eigen::Isometry3d start_pose, end_pose;

        // initialize ruckig online trajectory generation
        const static size_t DOFs {1};
        ruckig::Ruckig<DOFs> otg {1.0 / control_freq_};
        ruckig::InputParameter<DOFs> input;
        ruckig::OutputParameter<DOFs> output;

        input.max_velocity = {goal->limits.v_max};
        input.max_acceleration = {goal->limits.a_max};
        input.max_jerk = {goal->limits.j_max};

        // run control loop
        kintrol::PosVelSetpoint setpoint;
        ros::Rate rate(control_freq_);
        double calculation_duration {0.0};

        for (; current_segment < path.end(); current_segment++) 
        {
            tf::poseMsgToEigen(*current_segment, end_pose);
            tf::poseMsgToEigen(*std::prev(current_segment), start_pose);
            
            Eigen::Vector3d dir = end_pose.translation() - start_pose.translation();
            Eigen::Vector3d unit_dir = dir.normalized();
            auto path = [start_pose, unit_dir](auto s) { 
                return start_pose.translation() + s * unit_dir;
            };

            auto vel = [unit_dir](auto v) {
                return unit_dir * v;
            };

            input.current_position = {0.0};
            input.current_velocity = {0.0};
            input.current_acceleration = {0.0};

            input.target_position = {dir.norm()};
            output.time = 0.0;

            while (otg.update(input, output) == ruckig::Result::Working) {
                auto& s = output.new_position;
                auto& v = output.new_velocity;

                Eigen::Vector3d position = path(s[0]);
                Eigen::Vector3d velocity = vel(v[0]);
                
                // TODO: interpolate orientation
                Eigen::Quaterniond orient(end_pose.linear());
                poseToSetpoint(position, orient, setpoint);
                velocityToSetpoint(velocity, {0.0, 0.0, 0.0}, setpoint);

                output.pass_to_input(input);
                setpoint_pub_.publish(setpoint);
                rate.sleep();
            }
        }

        // TODO: interpolate orientation
        Eigen::Quaterniond orient(end_pose.linear());
        poseToSetpoint(end_pose.translation(), orient, setpoint);
        velocityToSetpoint({0, 0, 0}, {0, 0, 0}, setpoint);

        setpoint_pub_.publish(setpoint);
        if (success)
            as_.setSucceeded();
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