#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/TwistStamped.h>


namespace kintrol
{
class TrajectoryPublisher
{
public:
    TrajectoryPublisher(ros::NodeHandle& nh)
        : nh_(nh)
    {
        std::string setpoint_topic;
        size_t ros_queue_size;

        ros::NodeHandle rpnh(nh_, "kintrol_server");
        rosparam_shortcuts::get("/kintrol_server", rpnh, "setpoint_topic", setpoint_topic);
        rosparam_shortcuts::get("/kintrol_server", rpnh, "control_freq", control_freq_);
        rosparam_shortcuts::get("/kintrol_server", rpnh, "ros_queue_size", ros_queue_size);
        
        setpoint_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(setpoint_topic, ros_queue_size);
    }

    void run()
    {
        ros::Rate rate(control_freq_);
        geometry_msgs::TwistStamped msg;

        while (ros::ok())
        {
            msg.twist.linear.y = 0.1;
            msg.twist.linear.z = 0.1;

            setpoint_pub_.publish(msg);
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub_;

    double control_freq_;
};

} // namespace kintrol

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_pub");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    kintrol::TrajectoryPublisher traj_pub(nh);
    traj_pub.run();

    ros::waitForShutdown();
    return 0;
}