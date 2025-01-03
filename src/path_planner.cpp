#include "path_planner.h"
#include "hybrid_astar_algorithm/include/dubins.h"
#include "config.h"
#include <array>

namespace u_shaped_turn
{

PathPlanner::PathPlanner(/* args */)
    : is_target_pose_set_(false)
{
}

PathPlanner::~PathPlanner()
{
}

std::vector<Eigen::Vector2d> PathPlanner::getControls(const Eigen::Vector3d &ego_pose) const {
    if (!is_target_pose_set_) {
        throw std::runtime_error("Target pose is not set");
    }
    
    using namespace HybridAStar;
    // implement with dubins path
    double q0[] = {ego_pose[0], ego_pose[1], ego_pose[2]};
    double q1[] = {target_pose_[0], target_pose_[1], target_pose_[2]};
    DubinsPath path;
    double rho = kWheelBase / std::tan(kMaxSteer);
    dubins_init(q0, q1, rho, &path);
    double q[3];
    
    // convert q to control, control(0) is always 1.0, control(1) is steer which is kMaxSteer if 'L' or -kMaxSteer if 'R' or 0.0 if 'S'
    std::vector<Eigen::Vector2d> controls;
    // LSL, LSR, RSL, RSR, RLR, LRL correspond to
    // {kMaxSteer, 0.0, kMaxSteer}
    // {kMaxSteer, 0.0, -kMaxSteer}
    // {-kMaxSteer, 0.0, kMaxSteer}
    // {-kMaxSteer, 0.0, -kMaxSteer}
    // {-kMaxSteer, kMaxSteer, -kMaxSteer}
    // {kMaxSteer, -kMaxSteer, kMaxSteer}
    std::array<std::array<int, 3>, 6> type_to_steer_map = {
        std::array<int, 3>{1, 0, 1},
        std::array<int, 3>{1, 0, -1},
        std::array<int, 3>{-1, 0, 1},
        std::array<int, 3>{-1, 0, -1},
        std::array<int, 3>{-1, 1, -1},
        std::array<int, 3>{1, -1, 1}
    };

    // for every segment of DubinsPath
    for (int i = 0; i < 3; ++i) {
        double t = 0.0;
        // printf("param[%d]: %lf\n", i, path.param[i]);
        while (t < path.param[i] * rho) {
            dubins_path_sample(&path, t, q);
            Eigen::Vector2d control;
            control[0] = 1.0;
            control[1] = type_to_steer_map[path.type][i] * kMaxSteer;
            controls.push_back(control);
            t += kStepSize;
        }
    }
    

    return controls;
}

} // namespace u_shaped_turn
