#include <iostream>
#include "u_shaped_turn.cpp"

int main(int argc, char **argv) {
    Eigen::Vector3d start_pose{10.0, 0.0, 0.0};
    auto path = drive(std::vector<double>(), start_pose);
    for (const double& pt: path) {
        std::cout << pt << std::endl;
    }
    return 0;
}