#pragma once
#include <vector>
#include <eigen3/Eigen/Core>
#include <memory>

#include "vehicle.h"
#include "human_experience.h"

namespace u_shaped_turn
{
class PlanAgent
{
private:
    std::shared_ptr<Vehicle> vehicle_;
    std::vector<Eigen::Vector3d> trace_;
    Eigen::Vector3d ego_pose_;
    Eigen::Vector3d target_pose_;
public:
    PlanAgent(const std::shared_ptr<Vehicle> &vehicle);
    ~PlanAgent();

    void setEgoPose(const Eigen::Vector3d ego_pose) {
        trace_.clear();
        trace_.push_back(ego_pose);
    }
    Eigen::Vector3d getEgoPose() {
        return trace_.back(); 
    }
    void setTarget(const Eigen::Vector3d target_pose) {
        target_pose_ = target_pose;
    }

    void moveOnce(std::shared_ptr<HumanExperienceInterface> &human_experience, double step_size);
    std::vector<Eigen::Vector3d> getTrace() {
        return trace_;
    }
};

} // namespace u_shaped_turn
