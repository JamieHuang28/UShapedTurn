#pragma once
#include <vector>
#include <eigen3/Eigen/Core>

namespace u_shaped_turn
{
class Vehicle
{
public:
    Vehicle(double wheel_base);
    ~Vehicle();

    void setEgoPose(const Eigen::Vector3d &pose) {
        ego_pose_ = pose;
        is_ego_pose_set_ = true;
    }

    std::vector<Eigen::Vector3d> move(const std::vector<Eigen::Vector2d> controls);

    static Eigen::Vector3d calcKinetic(const Eigen::Vector3d &current_state, const Eigen::Vector2d &control, double wheel_base, double time_step);
private:
    double wheel_base_;
    bool is_ego_pose_set_;
    Eigen::Vector3d ego_pose_;
};

} // namespace u_shaped_turn
