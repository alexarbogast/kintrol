#include <moveit/move_group_interface/move_group_interface.h>
#include <kintrol/kintroller_manager.h>

static const std::string HOME_POSITION = "home";
static const std::string READY_POITION = "ready";

class KintrolTest
{
public:
    KintrolTest(ros::NodeHandle& nh)
        : hydra_move_group_interface_("hydra_planning_group"),
          manager_(nh)
    {
    }

    void run()
    {
        moveReady();

        manager_.start_joint_group_controller("robot1");
        manager_.start_joint_group_controller("robot2");
        manager_.start_joint_group_controller("robot3");
    }

    void moveHome()
    {
        hydra_move_group_interface_.setNamedTarget(HOME_POSITION);
        hydra_move_group_interface_.move();
    }

    void moveReady()
    {
        hydra_move_group_interface_.setNamedTarget(READY_POITION);
        hydra_move_group_interface_.move();
    }

private:
    moveit::planning_interface::MoveGroupInterface hydra_move_group_interface_;
    kintrol::KintrollerManager manager_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kintrol_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    KintrolTest test(nh);
    test.run();
}