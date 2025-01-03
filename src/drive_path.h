#pragma once
#include <vector>
#include <eigen3/Eigen/Core>


namespace u_shaped_turn
{

class DrivePath
{
private:
    std::vector<Eigen::Vector3d> poses_;
public:
    DrivePath();
    ~DrivePath();
};

} // namespace u_shaped_turn
