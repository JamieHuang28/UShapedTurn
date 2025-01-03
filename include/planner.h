#pragma once
#include <vector>
#include <eigen3/Eigen/Core>
#include "drive_path.h"

namespace u_shaped_turn
{

class Planner
{
public:
    Planner();
    ~Planner();

    std::vector<Eigen::Vector3d> plan(const Eigen::Vector3d start_pose, const Eigen::Vector3d target_pose, const DrivePath &drive_path, double step_size);
};


} // namespace u_shaped_turn
