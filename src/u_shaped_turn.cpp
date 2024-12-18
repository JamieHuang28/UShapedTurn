#include "u_shaped_turn.h"

std::vector<double> drive(std::vector<double> pts, Eigen::Ref<Eigen::Vector3d> start_pose) {
    auto path = std::vector<double>({0.0, 0.1, 0.2});
    path.insert(path.begin(), start_pose(0));
    return path;
}