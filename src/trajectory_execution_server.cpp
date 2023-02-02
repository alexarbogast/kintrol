#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/CartesianTrajectoryPoint.h>
#include <eigen_conversions/eigen_msg.h>

#include "kintrol/TrajectoryExecutionAction.h"

static const std::string ACTION_NAME = "trajectory_execution_action";
static double TRAVEL_VELOCITY = 100.0 / 1000.0; // m/s

static double V_MAX = 100.0 / 1000.0; // m/s
static double A_MAX = 300.0 / 1000.0; // m/s^2
static double J_MAX = 2000.0 / 1000.0; // m/s^3

// temporary class for time-parameterization of smooth s_curve trajectory
class ScurveTrajectory
{
/*
    Mathematics for Real-Time S-Curve Profile Generator
    (Marian BLEJAN, Robert BLEJAN)
*/
public:
    ScurveTrajectory(double s, double v_max, double a_max, double j_max)
        : s_(s), v_max_(v_max), a_max_(a_max), j_max_(j_max) 
    {
        double va = a_max*a_max / j_max;
        double sa = 2 * pow(a_max, 3) / (j_max * j_max);

        double tj, ta, tv;

        if (v_max < va && s > sa) {
            tj = sqrt(v_max / j_max);
            ta = tj;
            tv = s / v_max;
        }
        else if (v_max > va && s < sa) {
            tj = pow(s / (2*j_max), 1/3);
            ta = tj;
            tv = 2 * tj;
        }
        else if (v_max < va && s < sa) {
            double sv = 2 * v_max * sqrt(v_max / j_max);
            if (s >= sv) {
                tj = sqrt(v_max / j_max);
                ta = tj;
                tv = s / v_max;
            }
            else {
                tj = pow(s / (2*j_max), 1/3);
                ta = tj;
                tv = 2 * tj;
            }
        }
        else { // v_max > va && s > sa
            double sv = v_max * (v_max/a_max + a_max/j_max);
            if (s >= sv) {
                tj = a_max / j_max;
                ta = v_max / a_max;
                tv = s / v_max;
            }
            else {
                tj = a_max / j_max;
                ta = 0.5 * (sqrt((4*s*(j_max*j_max)+pow(a_max, 3))/(a_max*(j_max*j_max))) - a_max/j_max);
                tv = ta + tj;
            }
        }

        tt[0] = tj;
        tt[1] = ta;
        tt[2] = tj + ta;
        tt[3] = tv;
        tt[4] = tj + tv;
        tt[5] = tv + ta;
        tt[6] = tv + ta + tj;

        double dt = tt[0];
        pp[0] = j_max * pow(dt, 3)/6;
        vv[0] = j_max * pow(dt, 2)/2;
        aa[0] = j_max * dt;

        dt = tt[1] - tt[0];
        pp[1] = pp[0] + vv[0]*dt + aa[0]*pow(dt, 2)/2;
        vv[1] = vv[0] + aa[0]*dt;
        aa[1] = aa[0];

        dt = tt[2] - tt[1];
        pp[2] = pp[1] + vv[1]*dt + aa[1]*pow(dt, 2)/2 - j_max*pow(dt, 3)/6;
        vv[2] = vv[1] + aa[1]*dt - j_max*pow(dt, 2)/2;
        aa[2] = aa[1] - j_max*dt;

        dt = tt[3] - tt[2];
        pp[3] = pp[2] + vv[2]*dt;
        vv[3] = vv[2];
        aa[3] = 0;

        dt = tt[4] - tt[3];
        pp[4] = pp[3] + vv[3]*dt - j_max*pow(dt, 3)/6;
        vv[4] = vv[3] - j_max*pow(dt, 2)/2;
        aa[4] = -j_max*dt;

        dt = tt[5] - tt[4];
        pp[5] = pp[4] + vv[4]*dt + aa[4]*pow(dt, 2)/2;
        vv[5] = vv[4] - a_max*dt;
        aa[5] = aa[4];

        pp[6] = s;
        vv[6] = 0;
        aa[6] = 0;
    }

    std::tuple<double, double> get_point(double t)
    {
        double p = 0.0, v = 0.0;

        if (t < tt[0])         // j(t) = j_max
        {
            p = j_max_ * pow(t, 3)/6;
            v = j_max_ * pow(t, 2)/2;
        }
        else if (t < tt[1])    // j(t) = 0
        {
            double dt = t - tt[0];
            p = pp[0] + vv[0]*dt + aa[0]*pow(dt, 2)/2;
            v = vv[0] + aa[0]*dt;
        }
        else if (t < tt[2])    // j(t) = -j_max
        {
            double dt = t - tt[1];
            p = pp[1] + vv[1]*dt + aa[1]*pow(dt, 2)/2 - j_max_*pow(dt, 3)/6;
            v = vv[1] + aa[1]*dt - j_max_*pow(dt, 2)/2;
        }
        else if (t < tt[3])    // j(t) = 0
        {
            double dt = t - tt[2];
            p = pp[2] + vv[2]*dt;
            v = vv[2];
        }
        else if (t < tt[4])    // j(t) = -j_max
        {
            double dt = t - tt[3];
            p = pp[3] + vv[3]*dt - j_max_*pow(dt, 3)/6;
            v = vv[3] - j_max_*pow(dt, 2)/2;
        }
        else if (t < tt[5])   // j(t) = 0
        {
            double dt = t - tt[4];
            p = pp[4] + vv[4]*dt + aa[4]*pow(dt, 2)/2;
            v = vv[4] - a_max_*dt;
        }
        else if (t < tt[6])   // j(t) = j_max
        {
            double dt = t - tt[5];
            p = pp[5] + vv[5]*dt + aa[5]*pow(dt, 2)/2 + j_max_*pow(dt, 3)/6;
            v = vv[5] + aa[5]*dt + j_max_*pow(dt, 2)/2;
        }
        else
        {
            p = pp[6];
            v = vv[6];
        }

        return std::make_tuple(p, v);
    }

    inline double get_duration() const { return tt[6]; }
    
private:
    // values of time corresponding to movement phases
    double tj, ta, tv;
    double tt[7];

    // kinematic parameters corresponding to movement phases
    double pp[7], vv[7], aa[7];
    double s_, v_max_, a_max_, j_max_; 
};


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

        // constant_velocity_traj(goal);
        s_curve_traj(goal);

        if (success)
            as_.setSucceeded();
    }

    void constant_velocity_traj(const kintrol::TrajectoryExecutionGoalConstPtr& goal)
    {
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
                
                if (current_segment == path.end())
                    continue;

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
            
            // TODO: interpolate orientation
            Eigen::Quaterniond orient(end_pose.linear());
            msg.point.pose.orientation.w = orient.w();
            msg.point.pose.orientation.x = orient.x();
            msg.point.pose.orientation.y = orient.y();
            msg.point.pose.orientation.z = orient.z();

            auto velocity_vector = direction / direction.norm() * TRAVEL_VELOCITY;
            msg.point.velocity.linear.x = velocity_vector.x();
            msg.point.velocity.linear.y = velocity_vector.y();
            msg.point.velocity.linear.z = velocity_vector.z();

            setpoint_pub_.publish(msg);
            rate.sleep();
        }

        msg.point.pose.position.x = end_pose.translation().x();        
        msg.point.pose.position.y = end_pose.translation().y();
        msg.point.pose.position.z = end_pose.translation().z(); 

        // TODO: interpolate orientation
        Eigen::Quaterniond orient(end_pose.linear());
        msg.point.pose.orientation.w = orient.w();
        msg.point.pose.orientation.x = orient.x();
        msg.point.pose.orientation.y = orient.y();
        msg.point.pose.orientation.z = orient.z(); 

        msg.point.velocity.linear.x = 0.0;
        msg.point.velocity.linear.y = 0.0;
        msg.point.velocity.linear.z = 0.0;

        setpoint_pub_.publish(msg);
    }

    void s_curve_traj(const kintrol::TrajectoryExecutionGoalConstPtr& goal)
    {
        ros::Rate rate(control_freq_);
        moveit_msgs::CartesianTrajectoryPoint msg;

        const auto& path = goal->path.poses;
        auto current_segment = std::next(path.begin());

        Eigen::Isometry3d start_pose, end_pose;
        tf::poseMsgToEigen(*current_segment, end_pose);
        tf::poseMsgToEigen(*std::prev(current_segment), start_pose);

        Eigen::Vector3d direction = end_pose.translation() - start_pose.translation();
        double segment_length = direction.norm();
        Eigen::Vector3d unit_dir = direction / segment_length;

        ScurveTrajectory traj(segment_length, V_MAX, A_MAX, J_MAX);

        double segment_elapsed = 0.0;
        double previous = ros::Time::now().toSec();

        while (current_segment < path.end())
        {
            segment_elapsed += ros::Time::now().toSec() - previous;
            previous = ros::Time::now().toSec();

            if (segment_elapsed > traj.get_duration())
            {
                ++current_segment;
                if (current_segment == path.end())
                    continue;

                tf::poseMsgToEigen(*current_segment, end_pose);
                tf::poseMsgToEigen(*std::prev(current_segment), start_pose);
                
                direction = end_pose.translation() - start_pose.translation();
                segment_length = direction.norm();
                unit_dir = direction / segment_length;

                ScurveTrajectory new_traj(segment_length, V_MAX, A_MAX, J_MAX);
                traj = new_traj;
                segment_elapsed = 0.0;
            }

            double p, v;
            std::tie(p, v) = traj.get_point(segment_elapsed);

            auto position = start_pose.translation() + (unit_dir * p);
            auto velocity = unit_dir * v;

            msg.point.pose.position.x = position.x();        
            msg.point.pose.position.y = position.y();
            msg.point.pose.position.z = position.z();
            msg.point.velocity.linear.x = velocity.x();
            msg.point.velocity.linear.y = velocity.y();
            msg.point.velocity.linear.z = velocity.z();
            
            // TODO: interpolate orientation
            Eigen::Quaterniond orient(end_pose.linear());
            msg.point.pose.orientation.w = orient.w();
            msg.point.pose.orientation.x = orient.x();
            msg.point.pose.orientation.y = orient.y();
            msg.point.pose.orientation.z = orient.z();

            setpoint_pub_.publish(msg);
            rate.sleep();
        }

        msg.point.pose.position.x = end_pose.translation().x();        
        msg.point.pose.position.y = end_pose.translation().y();
        msg.point.pose.position.z = end_pose.translation().z(); 

        // TODO: interpolate orientation
        Eigen::Quaterniond orient(end_pose.linear());
        msg.point.pose.orientation.w = orient.w();
        msg.point.pose.orientation.x = orient.x();
        msg.point.pose.orientation.y = orient.y();
        msg.point.pose.orientation.z = orient.z(); 

        msg.point.velocity.linear.x = 0.0;
        msg.point.velocity.linear.y = 0.0;
        msg.point.velocity.linear.z = 0.0;
        setpoint_pub_.publish(msg);
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