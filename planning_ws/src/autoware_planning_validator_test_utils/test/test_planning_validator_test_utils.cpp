#include <autoware_planning_validator_test_utils/planning_validator_test_utils.hpp>
#include <autoware_planning_validator_test_utils/test_parameters.hpp>

#include <gtest/gtest.h>
#include <cmath>

namespace u = autoware::planning_validator::test_utils;

TEST(PlanningValidatorTestUtils, GeneratesAllTrajectoryFamilies) {
  const auto constant = u::generateTrajectoryWithConstantAcceleration(2.0, 5.0, 0.0, 4, -1.0);
  ASSERT_EQ(constant.points.size(), 4U);
  EXPECT_FLOAT_EQ(constant.points[0].longitudinal_velocity_mps, 5.0F);
  EXPECT_FLOAT_EQ(constant.points[1].longitudinal_velocity_mps, 4.6F);
  EXPECT_NEAR(constant.points[1].pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(constant.points[0].pose.orientation.w, 1.0, 1e-6);

  const auto sinusoidal = u::generateTrajectoryWithSinusoidalAcceleration(1.0, 2.0, 0.2, 5, 3.0, 4.0);
  ASSERT_EQ(sinusoidal.points.size(), 5U);
  EXPECT_NEAR(sinusoidal.points[1].acceleration_mps2, 3.0, 1e-6);
  const auto stepped = u::generateTrajectoryWithStepAcceleration(1.0, 2.0, 0.0, 5, {1.0, -2.0}, 2);
  EXPECT_FLOAT_EQ(stepped.points[0].acceleration_mps2, 1.0F);
  EXPECT_FLOAT_EQ(stepped.points[2].acceleration_mps2, -2.0F);

  const auto straight = u::generateTrajectoryWithConstantCurvature(1.0, 2.0, 0.0, 4, 3.5);
  EXPECT_EQ(straight.points.size(), 4U);  // upstream zero-curvature path delegates to generateTrajectory
  const auto curved = u::generateTrajectoryWithConstantCurvature(1.0, 2.0, 0.1, 4, 3.5);
  EXPECT_EQ(curved.points.size(), 5U);
  EXPECT_GT(curved.points.back().pose.position.y, 0.0);
  const auto steering = u::generateTrajectoryWithConstantSteering(1.0, 2.0, 0.2, 3, 3.5);
  EXPECT_NEAR(steering.points.front().front_wheel_angle_rad, 0.2, 1e-6);
  const auto steering_rate = u::generateTrajectoryWithConstantSteeringRate(1.0, 2.0, 1.0, 3, 3.5);
  EXPECT_GT(steering_rate.points[1].front_wheel_angle_rad, 0.0);

  const auto shifted = u::generateShiftedTrajectory(constant, 1.0, 2.0, 2);
  ASSERT_EQ(shifted.points.size(), 2U);
  EXPECT_NEAR(shifted.points[0].pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(shifted.points[0].pose.position.y, 1.0, 1e-6);
  EXPECT_TRUE(std::isnan(u::generateNanTrajectory().points.front().pose.position.x));
  EXPECT_TRUE(std::isinf(u::generateInfTrajectory().points.front().pose.position.x));
  EXPECT_EQ(u::generateBadCurvatureTrajectory().points.size(), 11U);
}

TEST(PlanningValidatorTestUtils, ProvidesAndAppliesCompleteDefaultParameterSet) {
  const auto p = u::getDefaultParameters();
  EXPECT_EQ(p.integers.size(), 2U);
  EXPECT_EQ(p.booleans.size(), 35U);
  EXPECT_EQ(p.doubles.size(), 34U);
  EXPECT_DOUBLE_EQ(p.doubles.at("planning_hz"), u::PLANNING_HZ);
  EXPECT_DOUBLE_EQ(p.doubles.at("wheel_base"), u::WHEELBASE);
  EXPECT_TRUE(p.booleans.at("trajectory_checker.trajectory_shift.is_critical"));
  ros::NodeHandle nh("planning_validator_test_utils_oracle");
  p.apply(nh);
  double planning_hz = 0.0, wheel_base = 0.0;
  bool trajectory_shift_critical = false;
  int handling_type = -1;
  ASSERT_TRUE(nh.getParam("planning_hz", planning_hz));
  ASSERT_TRUE(nh.getParam("wheel_base", wheel_base));
  ASSERT_TRUE(nh.getParam("trajectory_checker/trajectory_shift/is_critical", trajectory_shift_critical));
  ASSERT_TRUE(nh.getParam("default_handling_type", handling_type));
  EXPECT_DOUBLE_EQ(planning_hz, 10.0);
  EXPECT_DOUBLE_EQ(wheel_base, 3.5);
  EXPECT_TRUE(trajectory_shift_critical);
  EXPECT_EQ(handling_type, 0);
}

TEST(PlanningValidatorTestUtils, GeneratesDefaultVehicleStateInputs) {
  const auto odom = u::generateDefaultOdometry(3.0, -4.0, 5.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, -4.0);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 5.0);
  EXPECT_DOUBLE_EQ(u::generateDefaultAcceleration(-2.0).accel.accel.linear.x, -2.0);
}

int main(int argc, char ** argv) { ros::init(argc, argv, "planning_validator_test_utils_oracle"); ros::Time::init(); testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
