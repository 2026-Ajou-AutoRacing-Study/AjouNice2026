#include "autoware/planning_validator/diagnostic_render_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace
{
autoware_planning_validator::PlanningValidatorStatus valid_status()
{
  autoware_planning_validator::PlanningValidatorStatus status;
  status.is_valid_size = status.is_valid_finite_value = status.is_valid_interval = true;
  status.is_valid_relative_angle = status.is_valid_curvature = status.is_valid_lateral_acc = true;
  status.is_valid_lateral_jerk = status.is_valid_longitudinal_max_acc = true;
  status.is_valid_longitudinal_min_acc = status.is_valid_steering = status.is_valid_steering_rate = true;
  status.is_valid_velocity_deviation = status.is_valid_distance_deviation = true;
  status.is_valid_longitudinal_distance_deviation = status.is_valid_forward_trajectory_length = true;
  status.is_valid_latency = status.is_valid_yaw_deviation = status.is_valid_trajectory_shift = true;
  status.is_valid_intersection_collision_check = status.is_valid_rear_collision_check = true;
  return status;
}

const diagnostic_msgs::DiagnosticStatus & by_name(
  const std::vector<diagnostic_msgs::DiagnosticStatus> & statuses, const std::string & name)
{
  const auto found = std::find_if(statuses.begin(), statuses.end(), [&](const auto & status) {
    return status.name == name;
  });
  EXPECT_NE(found, statuses.end());
  return *found;
}

TEST(DiagnosticRenderCore, RendersAllAndOnlyTheNineteenSourceRegisteredChecks)
{
  const auto rendered = autoware::planning_validator::render_source_shaped_diagnostics(valid_status(), {});
  ASSERT_EQ(rendered.size(), 19U);
  for (const auto & item : rendered) {
    EXPECT_EQ(item.level, diagnostic_msgs::DiagnosticStatus::OK);
    EXPECT_EQ(item.message, "validated.");
  }
  EXPECT_EQ(std::count_if(rendered.begin(), rendered.end(), [](const auto & item) {
    return item.name == "trajectory_validation_lateral_jerk";
  }), 0);
}

TEST(DiagnosticRenderCore, PreservesWarnThresholdErrorAndTrajectoryOnlyOverride)
{
  auto status = valid_status();
  status.is_valid_finite_value = false;
  status.invalid_count = 0;
  autoware::planning_validator::DiagnosticRenderParameters threshold;
  threshold.error_count_threshold = 1;
  auto rendered = autoware::planning_validator::render_source_shaped_diagnostics(status, threshold);
  const auto & warn = by_name(rendered, "trajectory_validation_finite");
  EXPECT_EQ(warn.level, diagnostic_msgs::DiagnosticStatus::WARN);
  EXPECT_EQ(warn.message, "infinite value is found (invalid count is less than error threshold: 0 < 1)");
  status.invalid_count = 1;
  rendered = autoware::planning_validator::render_source_shaped_diagnostics(status, threshold);
  EXPECT_EQ(by_name(rendered, "trajectory_validation_finite").level, diagnostic_msgs::DiagnosticStatus::ERROR);
  threshold.trajectory_override_all_errors = true;
  rendered = autoware::planning_validator::render_source_shaped_diagnostics(status, threshold);
  const auto & overridden = by_name(rendered, "trajectory_validation_finite");
  EXPECT_EQ(overridden.level, diagnostic_msgs::DiagnosticStatus::WARN);
  EXPECT_EQ(overridden.message, "infinite value is found (error diag was overriden internally)");
  status.is_valid_rear_collision_check = false;
  EXPECT_EQ(by_name(rendered, "rear_collision_check").level, diagnostic_msgs::DiagnosticStatus::OK);
  rendered = autoware::planning_validator::render_source_shaped_diagnostics(status, threshold);
  EXPECT_EQ(by_name(rendered, "rear_collision_check").level, diagnostic_msgs::DiagnosticStatus::ERROR);
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
