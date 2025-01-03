#include "path_planner.h"

namespace u_shaped_turn
{

PathPlanner::PathPlanner(/* args */)
{
}

PathPlanner::~PathPlanner()
{
}

std::vector<Eigen::Vector2d> PathPlanner::getControls(const Eigen::Vector3d &ego_pose) const {
    return std::vector<Eigen::Vector2d>();
}

} // namespace u_shaped_turn
