#pragma once
#include <vector>
#include <eigen3/Eigen/Core>

std::vector<double> drive(std::vector<double> pts, Eigen::Ref<Eigen::Vector3d> start_pose);