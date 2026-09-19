#include "autoware/planning_validator/debug_marker_ros1.hpp"
#include <gtest/gtest.h>

TEST(DebugMarkerRos1, ListsOnlyInvalidSourceStatusLabels)
{
  autoware_planning_validator::PlanningValidatorStatus status;
  status.is_valid_latency = status.is_valid_finite_value = status.is_valid_relative_angle = true;
  status.is_valid_distance_deviation = status.is_valid_longitudinal_distance_deviation = true;
  status.is_valid_velocity_deviation = status.is_valid_longitudinal_max_acc = true;
  status.is_valid_longitudinal_min_acc = status.is_valid_lateral_acc = status.is_valid_lateral_jerk = true;
  status.is_valid_yaw_deviation = status.is_valid_curvature = status.is_valid_steering = true;
  status.is_valid_steering_rate = status.is_valid_forward_trajectory_length = true;
  status.is_valid_trajectory_shift = status.is_valid_intersection_collision_check = true;
  status.is_valid_size = false; status.is_valid_interval = false;
  EXPECT_EQ(autoware::planning_validator::DebugMarkerRos1::status_string(status), "[traj_size | interval]");
}

int main(int argc, char ** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
