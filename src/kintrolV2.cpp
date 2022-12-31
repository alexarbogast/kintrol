#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

namespace kintrol
{
class KintrolV2
{
public:
    KintrolV2(ros::NodeHandle& nh)
    {
        moveit_cpp_ptr_.reset(new moveit_cpp::MoveItCpp(nh));
        moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

        //planning_component1_.reset(new moveit_cpp::PlanningComponent("hydra_planning_group", moveit_cpp_ptr_));
    }

    void moveReady()
    {
        planning_component1_->setStartStateToCurrentState();
        planning_component1_->setGoal("ready");
        planning_component1_->plan();
        planning_component1_->execute();
    }

private:
    moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
    moveit_cpp::PlanningComponentPtr planning_component1_;
};

} //namespace kintrol V2

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kintrol_server");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh("/kintrol_server");

    kintrol::KintrolV2 kintrol(nh);
    //kintrol.moveReady();

    ros::waitForShutdown();
    return 0;
}