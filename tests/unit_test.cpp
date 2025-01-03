#include "gtest/gtest.h"
#include "../src/vehicle.h"

namespace {
    TEST(Exp1Test, add)
    {
        double res;
        res = 1.0 + 2.0;
        ASSERT_NEAR(res, 3.0, 1.0e-11);
    }

    TEST(Exp1Test, subtract)
    {
        double res;
        res = 1.0 - 2.0;
        ASSERT_NEAR(res, -1.0, 1.0e-11);
    }

    TEST(Vehicle, calcKinematic) {
        using namespace u_shaped_turn;

        Eigen::Vector2d control(1.0, 0.5);
        Eigen::Vector3d current_state(0.0, 0.0, 0.0);
        double wheel_base = 3.0;
        double time_step = 0.1;
        // auto new_state = Vehicle::calcKinetic(current_state, control, wheel_base, time_step);

        // drive a full circle
        double round_length = 2 * M_PI * wheel_base / std::tan(control(1));
        size_t num_steps = size_t(round_length / (control(0) * time_step));
        printf("round_length: %lf, step: %lu", round_length, num_steps);
        for (size_t i = 0; i < num_steps; ++i) {
            current_state = Vehicle::calcKinetic(current_state, control, wheel_base, time_step);
        }
        EXPECT_NEAR(current_state(2), 0.0, 1e-3); // direction is near 2pi

        // turn steer backward
        control(1) = -0.5;
        for (size_t i = 0; i < num_steps; ++i) {
            current_state = Vehicle::calcKinetic(current_state, control, wheel_base, time_step);
        }
        EXPECT_NEAR(current_state(2), 0.0, 1e-3); // direction is near 2pi
    }
}
