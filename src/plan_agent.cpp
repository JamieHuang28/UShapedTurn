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
    std::vector<Eigen::Vector2d> controls = human_experience->getControls(getEgoPose());
    std::vector<Eigen::Vector3d> new_path = vehicle_->move(controls);
    std::copy(new_path.begin(), new_path.end(), trace_.begin());
    printf("moveOnce\n");
}

} // namespace u_shaped_turn
