#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

const static std::string CMD_TOPIC = "/positioner_joint_velocity_controller/command";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kintrol_test");
    ros::NodeHandle nh;

    ros::Publisher pos_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>(CMD_TOPIC, 2);
    ros::Rate rate(100);

    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;

        double vel = 0.3 * sin(ros::Time::now().toSec());
        msg.data.push_back(vel);
        pos_cmd_pub.publish(msg) ;    
        rate.sleep();
    }
}