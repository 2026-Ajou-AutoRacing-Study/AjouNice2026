#include "autoware/planning_validator/rear_shift_activation_core.hpp"

#include <gtest/gtest.h>

namespace
{
lanelet::ConstLanelet straight_lane()
{
  lanelet::Points3d left{lanelet::Point3d(1, 0.0, 2.0, 0.0),
    lanelet::Point3d(2, 20.0, 2.0, 0.0)};
  lanelet::Points3d right{lanelet::Point3d(3, 0.0, -2.0, 0.0),
    lanelet::Point3d(4, 20.0, -2.0, 0.0)};
  return lanelet::Lanelet(10, lanelet::LineString3d(11, left), lanelet::LineString3d(12, right));
}

autoware_planning_msgs::Trajectory trajectory_with_endpoint(const double endpoint_y)
{
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & item : {std::pair{0.0, 0.0}, std::pair{10.0, endpoint_y}}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = item.first;
    point.pose.position.y = item.second;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

autoware::planning_validator::RearMotionEnvelope long_envelope()
{
  autoware::planning_validator::RearMotionEnvelope envelope;
  envelope.lookahead_left.position.x = 100.0;
  envelope.lookahead_left.position.y = 1.0;
  envelope.lookahead_right.position.x = 100.0;
  envelope.lookahead_right.position.y = -1.0;
  envelope.predicted_stop_left.position.x = 0.0;
  envelope.predicted_stop_right.position.x = 0.0;
  envelope.lookahead_distance = 100.0;
  envelope.predicted_stop_distance = 0.0;
  return envelope;
}

autoware::vehicle_info_utils::VehicleInfo vehicle()
{
  autoware::vehicle_info_utils::VehicleInfo info;
  info.vehicle_width_m = 2.0;
  return info;
}
}  // namespace

TEST(RearShiftActivationCore, ActivatesOnLeftBoundaryCrossing)
{
  const auto lane = straight_lane();
  const auto trajectory = trajectory_with_endpoint(3.0);
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  const auto result = autoware::planning_validator::evaluate_rear_shift_activation(
    {{lane}, trajectory, odometry, vehicle(), long_envelope(), false}, {});
  EXPECT_EQ(result.behavior, autoware::planning_validator::RearBehavior::SHIFT_LEFT);
  EXPECT_GT(result.distance_to_conflict, 0.0);
}

TEST(RearShiftActivationCore, DoesNotActivateForStraightTrajectory)
{
  const auto lane = straight_lane();
  const auto trajectory = trajectory_with_endpoint(0.0);
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  const auto result = autoware::planning_validator::evaluate_rear_shift_activation(
    {{lane}, trajectory, odometry, vehicle(), long_envelope(), false}, {});
  EXPECT_EQ(result.behavior, autoware::planning_validator::RearBehavior::NONE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
