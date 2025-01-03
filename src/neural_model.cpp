#include "neural_model.h"

namespace u_shaped_turn
{

NeuralModel::NeuralModel(const DrivePath &drive_path)
    : drive_path_(drive_path)
{
}

NeuralModel::~NeuralModel()
{
}

std::vector<Eigen::Vector2d> NeuralModel::getControls(const Eigen::Vector3d &ego_pose) const {
    std::vector<Eigen::Vector2d> controls(1);
    
    // project ego_pose to drive_path

    // get control with respect to projection

    return controls;
}

} // namespace u_shaped_turn
