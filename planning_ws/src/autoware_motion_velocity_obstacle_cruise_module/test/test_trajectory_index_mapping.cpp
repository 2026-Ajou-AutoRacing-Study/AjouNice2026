#include "trajectory_index_mapping.hpp"

#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
using autoware::motion_velocity_planner::map_collision_index_to_original;
using Point = autoware_planning_msgs::TrajectoryPoint;

Point point(double x, double y = 0.0, double yaw = 0.0)
{
  Point p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.longitudinal_velocity_mps = 10.0;
  return p;
}

TEST(TrajectoryIndexMapping, CroppedResampledCollisionKeepsCorrectFrontRearDecision)
{
  std::vector<Point> raw;
  for (int i = 0; i <= 1000; ++i) raw.push_back(point(i * 0.2));
  const auto decimated = autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
    raw, point(100.0).pose, 3.0, 1.0, 2.0, 0.0);
  const auto mapped = map_collision_index_to_original(raw, decimated, 10);
  ASSERT_TRUE(mapped);
  EXPECT_NEAR(raw.at(*mapped).pose.position.x, 120.0, 0.21);
  const auto object_idx = autoware::motion_utils::findNearestIndex(raw, point(118.0).pose.position);
  EXPECT_LT(autoware::motion_utils::calcSignedArcLength(raw, object_idx, 10), -100.0);
  EXPECT_NEAR(autoware::motion_utils::calcSignedArcLength(raw, object_idx, *mapped), 2.0, 0.21);
}

TEST(TrajectoryIndexMapping, HandlesUnevenSpacingAndInsertedStopSample)
{
  const std::vector<Point> raw{point(0), point(0.7), point(2.1), point(3.3), point(4.9), point(8)};
  const std::vector<Point> resampled{point(2.1), point(3.3), point(4.1), point(6.1), point(8)};
  ASSERT_TRUE(map_collision_index_to_original(raw, resampled, 1));
  EXPECT_EQ(*map_collision_index_to_original(raw, resampled, 1), 3U);
  EXPECT_EQ(*map_collision_index_to_original(raw, resampled, 4), 5U);
}

TEST(TrajectoryIndexMapping, UsesPoseHeadingAtOverlappingOppositeDirectionSegments)
{
  const std::vector<Point> raw{point(0), point(2), point(4), point(2, 0, M_PI), point(0, 0, M_PI)};
  const std::vector<Point> resampled{point(4, 0, M_PI), point(2, 0, M_PI)};
  ASSERT_TRUE(map_collision_index_to_original(raw, resampled, 1));
  EXPECT_EQ(*map_collision_index_to_original(raw, resampled, 1), 3U);
}

TEST(TrajectoryIndexMapping, MapsOnCurvedPathAndWhenResampledArrayHasMorePoints)
{
  const std::vector<Point> raw{point(0), point(1), point(2, 1, M_PI_2), point(2, 2, M_PI_2)};
  const std::vector<Point> resampled{
    point(1), point(1.5, 0.5, M_PI_4), point(2, 1, M_PI_2),
    point(2, 1.4, M_PI_2), point(2, 1.8, M_PI_2), point(2, 2, M_PI_2)};
  ASSERT_TRUE(map_collision_index_to_original(raw, resampled, 5));
  EXPECT_EQ(*map_collision_index_to_original(raw, resampled, 5), 3U);
}

TEST(TrajectoryIndexMapping, InvalidInputDoesNotProduceAnIndex)
{
  const std::vector<Point> raw{point(0), point(1)};
  EXPECT_FALSE(map_collision_index_to_original({}, raw, 0));
  EXPECT_FALSE(map_collision_index_to_original(raw, {}, 0));
  EXPECT_FALSE(map_collision_index_to_original(raw, raw, raw.size()));
  EXPECT_FALSE(map_collision_index_to_original(raw, {point(NAN)}, 0));
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
