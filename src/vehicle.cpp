#include "vehicle.h"
#include <cmath>
#include <algorithm>
#include "config.h"

namespace u_shaped_turn
{

Vehicle::Vehicle(double wheel_base)
    : is_ego_pose_set_(false), wheel_base_(wheel_base)
{
}

Vehicle::~Vehicle()
{
}

std::vector<Eigen::Vector3d> Vehicle::move(const std::vector<Eigen::Vector2d> controls) {
    std::vector<Eigen::Vector3d> trace(controls.size());
    if (!is_ego_pose_set_) {
        printf("[error]: move before ego pose is set!\n");
        return std::vector<Eigen::Vector3d>();
    }

    // // print controls
    // for (const auto &control : controls) {
    //     printf("control: %lf, %lf\n", control(0), control(1));
    // }

    std::transform(controls.begin(), controls.end(), trace.begin(), [this, kStepSize](const Eigen::Vector2d &control) {
        this->ego_pose_ = Vehicle::calcKinetic(this->ego_pose_, control, this->wheel_base_, kStepSize);
        return this->ego_pose_;
    });
    // // print the trace
    // for (const auto &pose : trace) {
    //     printf("pose: %lf, %lf, %lf\n", pose(0), pose(1), pose(2));
    // }

    return trace;
}

inline double normalizeAngle(double angle) {
    // Use fmod to handle all cases of the angle
    double normalized = std::fmod(angle + M_PI, 2 * M_PI);
    
    // If the result is negative, adjust it to be within [-PI, PI]
    if (normalized < 0) {
        normalized += 2 * M_PI;
    }
    
    // Shift back to the desired range
    return normalized - M_PI;
}

Eigen::Vector3d Vehicle::calcKinetic(const Eigen::Vector3d &current_state, const Eigen::Vector2d &control, double wheel_base, double time_step)
{
    double velocity = control(0);
    double steer = control(1);

    double curvature = std::tan(steer) / wheel_base;

    Eigen::Vector3d next_state;
    next_state(0) = current_state(0) + velocity * time_step * std::cos(current_state(2));
    next_state(1) = current_state(1) + velocity * time_step * std::sin(current_state(2));
    next_state(2) = normalizeAngle(current_state(2) + velocity * time_step * curvature);

    return next_state;
}

} // namespace u_shaped_turn
