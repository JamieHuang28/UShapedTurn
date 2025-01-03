#pragma once
#include <vector>
#include <eigen3/Eigen/Core>
#include "human_experience.h"
#include "drive_path.h"

namespace u_shaped_turn
{
class NeuralModel: public HumanExperienceInterface
{
private:
    DrivePath drive_path_;
public:
    NeuralModel(const DrivePath &drive_path);
    ~NeuralModel();

    std::vector<Eigen::Vector2d> getControls(const Eigen::Vector3d &ego_pose) const override;
};

} // namespace u_shaped_turn
