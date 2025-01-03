#pragma once
#include <eigen3/Eigen/Core>

namespace u_shaped_turn
{

class StageChecker
{
private:
    /* data */
public:
    StageChecker();
    ~StageChecker();
    static bool check(const Eigen::Vector3d &start_pose, const Eigen::Vector3d &target_pose) {
        return true;
    }
};

}