#pragma once
#include <vector>
#include <eigen3/Eigen/Core>

namespace u_shaped_turn
{
class HumanExperienceInterface
{
public:
    virtual ~HumanExperienceInterface() = default;

    virtual std::vector<Eigen::Vector2d> getControls(const Eigen::Vector3d &ego_pose) const = 0;
};

} // namespace u_shaped_turn
