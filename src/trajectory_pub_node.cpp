#include <ros/ros.h>
#include "kintrol/trajectory_pub.h"

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