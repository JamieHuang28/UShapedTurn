#pragma once
#include "human_experience.h"
#include <vector>
#include <eigen3/Eigen/Core>

namespace u_shaped_turn
{

class PathPlanner: public HumanExperienceInterface
{
private:
    Eigen::Vector3d target_pose_;
    bool is_target_pose_set_;

public:
    PathPlanner(/* args */);
    ~PathPlanner();
    
    void setTargetPose(const Eigen::Vector3d &target_pose) {
        target_pose_ = target_pose;
        is_target_pose_set_ = true;
    }

    std::vector<Eigen::Vector2d> getControls(const Eigen::Vector3d &ego_pose) const override;
};

} // namespace u_shaped_turn
