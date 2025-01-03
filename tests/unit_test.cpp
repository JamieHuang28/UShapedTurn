#include "gtest/gtest.h"
#include "../src/vehicle.h"
#include "../src/path_planner.h"
#include "../src/config.h"
#include "planner.h"

namespace {

    class VehicleTest : public testing::Test {
    protected:
        virtual void SetUp() {
            current_state = Eigen::Vector3d(0.0, 0.0, 0.0);
            control = Eigen::Vector2d(1.0, u_shaped_turn::kMaxSteer);
        }
        virtual void TearDown() {

        }
        double getRadius() {
            return wheel_base / std::tan(control(1));
        }
        double getStepSize() {
            return time_step * control(0);
        }

        Eigen::Vector2d control;
        Eigen::Vector3d current_state;
        double wheel_base = 3.0;
        double time_step = 0.1;
    };

    TEST_F(VehicleTest, calcKinematic) {
        using namespace u_shaped_turn;

        double radius = getRadius();
        double step_size = control(0) * time_step;
        // auto new_state = Vehicle::calcKinetic(current_state, control, wheel_base, time_step);

        // drive a full circle
        double round_length = 2 * M_PI * radius;
        size_t num_steps = size_t(round_length / step_size);
        // printf("round_length: %lf, step: %lu", round_length, num_steps);
        for (size_t i = 0; i < num_steps; ++i) {
            current_state = Vehicle::calcKinetic(current_state, control, wheel_base, time_step);
        }
        EXPECT_NEAR(current_state(2), 0.0, 1e-3); // direction

        // drive a half circle
        current_state =  Eigen::Vector3d(0.0, 0.0, 0.0);
        for (size_t i = 0; i < num_steps / 2; ++i) {
            current_state = Vehicle::calcKinetic(current_state, control, wheel_base, time_step);
        }
        EXPECT_NEAR(current_state(1), 2 * radius, 1e-2); // y value

        
        
        // turn steer backward ad repeat process above
        control(1) = -0.5;
        current_state =  Eigen::Vector3d(0.0, 0.0, 0.0);
        for (size_t i = 0; i < num_steps; ++i) {
            current_state = Vehicle::calcKinetic(current_state, control, wheel_base, time_step);
        }
        EXPECT_NEAR(std::cos(current_state(2)), std::cos(0.0), 1e-3); // direction
        EXPECT_NEAR(std::sin(current_state(2)), std::sin(0.0), 1e-3); // direction
    }

    TEST_F(VehicleTest, VehicleMove1) {
        using namespace u_shaped_turn;
        Vehicle vehicle(wheel_base);
        
        // generate full circle controls
        std::vector<Eigen::Vector2d> full_circle_controls(size_t(getRadius() * 2 * M_PI / getStepSize()), control);

        vehicle.setEgoPose(current_state);
        std::vector<Eigen::Vector3d> full_circle_path = vehicle.move(full_circle_controls);

        // check direction
        const auto &end_pose = full_circle_path.back();
        EXPECT_NEAR(std::cos(end_pose(2)), std::cos(0.0), 1e-3);
        EXPECT_NEAR(std::sin(end_pose(2)), std::sin(0.0), 1e-3);

        // check y value
        const auto &mid_pose = full_circle_path.at(full_circle_controls.size() / 2);
        EXPECT_NEAR(mid_pose(1), 2 * getRadius(), 1e-2);
    }

    TEST_F(VehicleTest, VehicleMove2) {
        using namespace u_shaped_turn;
        Vehicle vehicle(wheel_base);
        
        // generate full circle controls
        std::vector<Eigen::Vector2d> full_circle_controls(size_t(getRadius() * 2 * M_PI / getStepSize()), {control(0),-control(1)});

        vehicle.setEgoPose(current_state);
        std::vector<Eigen::Vector3d> full_circle_path = vehicle.move(full_circle_controls);

        // check direction
        const auto &end_pose = full_circle_path.back();
        EXPECT_NEAR(std::cos(end_pose(2)), std::cos(0.0), 1e-3);
        EXPECT_NEAR(std::sin(end_pose(2)), std::sin(0.0), 1e-3);

        // check y value
        const auto &mid_pose = full_circle_path.at(full_circle_controls.size() / 2);
        EXPECT_NEAR(mid_pose(1), -2 * getRadius(), 1e-2);
    }

    TEST(PathPlanner, getControlsStraight) {
        using namespace u_shaped_turn;
        PathPlanner path_planner;
        path_planner.setTargetPose(Eigen::Vector3d(10.0, 0.0, 0.0));
        auto controls = path_planner.getControls(Eigen::Vector3d(0.0, 0.0, 0.0));
        // check the control(1) of the mid control
        const auto &mid_control = controls.at(controls.size() / 2);
        EXPECT_NEAR(mid_control(1), 0.0, 1e-3);
    }

    TEST(PathPlanner, getControlsUTurn) {
        using namespace u_shaped_turn;
        PathPlanner path_planner;
        path_planner.setTargetPose(Eigen::Vector3d(0.0, -10.0, M_PI));
        auto controls = path_planner.getControls(Eigen::Vector3d(0.0, 0.0, 0.0));
        // check the control(1) of the mid control
        const auto &mid_control = controls.at(controls.size() / 2);
        EXPECT_NEAR(mid_control(1), -kMaxSteer, 1e-3);
        
        // // print controls
        // for (const auto &control : controls) {
        //     printf("control: %lf, %lf\n", control(0), control(1));
        // }
    }

    TEST(Planner, Basic) {
        using namespace u_shaped_turn;
        Planner planner;

        DrivePath drive_path;
        Eigen::Vector3d start_pose(0.0, 0.0, 0.0);
        Eigen::Vector3d end_pose(0.0, -10.0, M_PI);
        auto path = planner.plan(start_pose, end_pose, drive_path);

        // check the direction of the end pose
        const auto &end_pose_ = path.back();
        EXPECT_NEAR(std::cos(end_pose_(2)), std::cos(M_PI), 1e-3);
        EXPECT_NEAR(std::sin(end_pose_(2)), std::sin(M_PI), 1e-3);
    }
}
