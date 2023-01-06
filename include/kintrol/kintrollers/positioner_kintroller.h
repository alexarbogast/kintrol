# pragma once

#include "kintrol/kintrollers/kintroller.h"
#include "kintrol/low_pass_filter.h"

namespace kintrol
{
class PositionerKintroller : public KintrollerBase
{
public:
    struct CoordinatedUnitContext
    {
        std::string joint_model_group;
        std::string end_effector;

        std::string base_frame;
    };

    PositionerKintroller(const std::string& name, const KintrolParameters& params, const KinematicChain& kc);
    
    void initialize(robot_model::RobotModelConstPtr robot_model);
    virtual void update(const Setpoint& setpoint,
                        robot_state::RobotStatePtr& robot_state,
                        Eigen::VectorXd& cmd_out) override;

    inline const std::string& getPoseFrame() const {return pose_frame_; }

private:  
    std::vector<CoordinatedUnitContext> coord_unit_contexts_;
    LowPassFilter filter_;

    unsigned int n_vars_;
    Eigen::VectorXd output_history;
    ros::Time previous_;
};

} // namespace kintrol