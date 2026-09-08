#include "autoware/planning_validator/rear_object_projection_core.hpp"

#include <gtest/gtest.h>

namespace
{
autoware_internal_planning_msgs::PathWithLaneId straight_path()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int x = 0; x <= 4; x += 2) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(x);
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 2.0F;
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

TEST(RearObjectProjectionCore, KeepsSourceNearestObjectToLaneEnd)
{
  autoware::planning_validator::RearPointCloud points;
  points.push_back({1.0F, 0.0F, 0.5F});
  points.push_back({3.0F, 0.0F, 0.5F});
  const auto result = autoware::planning_validator::project_rear_object_to_centerline(points, straight_path());
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->pose.position.x, 3.0, 1.0e-6);
  EXPECT_NEAR(result->absolute_distance, 1.0, 1.0e-6);
}

TEST(RearObjectProjectionCore, RejectsTooShortCenterline)
{
  autoware::planning_validator::RearPointCloud points;
  points.push_back({1.0F, 0.0F, 0.5F});
  autoware_internal_planning_msgs::PathWithLaneId short_path;
  EXPECT_FALSE(autoware::planning_validator::project_rear_object_to_centerline(points, short_path).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
