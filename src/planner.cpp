#include "planner.h"
#include <memory>

#include "vehicle.h"
#include "path_planner.h"
#include "neural_model.h"
#include "plan_agent.h"
#include "stage_checker.h"
#include "config.h"

namespace u_shaped_turn
{

Planner::Planner()
{
}

Planner::~Planner()
{
}

inline bool isEgoNearTarget(const Eigen::Vector3d &ego_pose, const Eigen::Vector3d &target_pose) {
    return (ego_pose - target_pose).norm() < kStepSize;
}

std::vector<Eigen::Vector3d> Planner::plan(const Eigen::Vector3d start_pose, const Eigen::Vector3d target_pose, const DrivePath &drive_path) {
    constexpr double step_size = kStepSize;
    std::shared_ptr<HumanExperienceInterface> neural_model_experience = std::make_shared<NeuralModel>(drive_path);
    auto path_planner = std::make_shared<PathPlanner>();
    path_planner->setTargetPose(target_pose);
    std::shared_ptr<HumanExperienceInterface> path_planner_experience = path_planner;

    std::shared_ptr<Vehicle> vehicle = std::make_shared<Vehicle>(kWheelBase);
    vehicle->setEgoPose(start_pose);
    
    PlanAgent plan_agent(vehicle);
    plan_agent.setEgoPose(start_pose);
    plan_agent.setTarget(target_pose);

    int iter = 0;
    while (!isEgoNearTarget(plan_agent.getEgoPose(), target_pose) && iter++ < 1000) {
        if (!StageChecker::check(plan_agent.getEgoPose(), target_pose)) {
            plan_agent.moveOnce(neural_model_experience, step_size);
        } else {
            plan_agent.moveOnce(path_planner_experience, step_size);
        }
    }
    
    return plan_agent.getTrace();
}

} // namespace u_shaped_turn
