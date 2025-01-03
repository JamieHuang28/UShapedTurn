#include "plan_agent.h"
#include <cstdio>

namespace u_shaped_turn
{

PlanAgent::PlanAgent(const std::shared_ptr<Vehicle> &vehicle)
    : vehicle_(vehicle)
{
}

PlanAgent::~PlanAgent()
{
}

void PlanAgent::moveOnce(std::shared_ptr<HumanExperienceInterface> &human_experience, double step_size)
{
    /*
    TODO(zhengming.huang@horizon.auto): by dividing controls and move, the precision is lost
    It is better to combine them.
    */
    std::vector<Eigen::Vector2d> controls = human_experience->getControls(getEgoPose());
    std::vector<Eigen::Vector3d> new_path = vehicle_->move(controls);
    // copy the new_path to the end of trace
    trace_.insert(trace_.end(), new_path.begin(), new_path.end());
    // printf("moveOnce\n");
}

} // namespace u_shaped_turn
