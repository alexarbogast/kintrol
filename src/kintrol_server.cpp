#include "kintrol/kintrol.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

static std::string LOGNAME = "kintrol_server";

int main(int argc, char** argv)
{
    ros::init(argc, argv, LOGNAME);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh("~");
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    kintrol::Kintrol kintrol(nh, planning_scene_monitor);
    kintrol.run();

    ros::waitForShutdown();
    return 0;
}