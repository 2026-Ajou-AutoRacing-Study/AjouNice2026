#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>

#include <gtest/gtest.h>

namespace
{
autoware_internal_planning_msgs::PathWithLaneId makePath()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i = 0; i != 6; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i);
    point.lane_ids.push_back(100);
    path.points.push_back(point);
  }
  return path;
}

geometry_msgs::Point point(const double x, const double y)
{
  geometry_msgs::Point value;
  value.x = x;
  value.y = y;
  return value;
}
}  // namespace

namespace arc = autoware::behavior_velocity_planner::arc_lane_utils;
using autoware::behavior_velocity_planner::LineString2d;
using autoware::behavior_velocity_planner::Point2d;

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(ArcLaneUtil, FindCollisionSegment)
{
  const auto path = makePath();
  LineString2d stop_line{Point2d(3.5, 3.0), Point2d(3.5, -3.0)};
  const auto segment = arc::findCollisionSegment(path, stop_line, {100});
  ASSERT_TRUE(segment.has_value());
  EXPECT_EQ(3U, segment->first);
  EXPECT_DOUBLE_EQ(3.5, segment->second.x);
  EXPECT_DOUBLE_EQ(0.0, segment->second.y);
}

TEST(ArcLaneUtil, FindForwardOffsetSegment)
{
  const auto path = makePath();
  const auto collision = std::make_pair(3U, point(3.5, 0.0));
  const auto segment = arc::findOffsetSegment(path, collision, 1.0);
  ASSERT_TRUE(segment.has_value());
  EXPECT_EQ(4U, segment->first);
  EXPECT_DOUBLE_EQ(0.5, segment->second);
  EXPECT_FALSE(arc::findOffsetSegment(path, collision, INFINITY).has_value());
}

TEST(ArcLaneUtil, FindBackwardOffsetSegment)
{
  const auto path = makePath();
  const auto collision = std::make_pair(3U, point(3.5, 0.0));
  const auto segment = arc::findOffsetSegment(path, collision, -1.0);
  ASSERT_TRUE(segment.has_value());
  EXPECT_EQ(2U, segment->first);
  EXPECT_DOUBLE_EQ(0.5, segment->second);
  EXPECT_FALSE(arc::findOffsetSegment(path, collision, -INFINITY).has_value());
}

TEST(ArcLaneUtil, RestoredSignedDistanceAndIndexedOffset)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  EXPECT_DOUBLE_EQ(2.0, arc::calcSignedDistance(pose, point(2.0, 0.0)));
  EXPECT_DOUBLE_EQ(-2.0, arc::calcSignedDistance(pose, point(-2.0, 0.0)));
  const auto path = makePath();
  const auto offset = arc::findOffsetSegment(path, 3U, -1.5);
  ASSERT_TRUE(offset.has_value());
  EXPECT_EQ(1U, offset->first);
  EXPECT_DOUBLE_EQ(0.5, offset->second);
}

TEST(ArcLaneUtil, RestoredTargetPoseAndStopLineTarget)
{
  const auto path = makePath();
  const auto pose = arc::calcTargetPose(path, std::make_pair(2U, 0.25));
  EXPECT_DOUBLE_EQ(2.25, pose.position.x);
  EXPECT_DOUBLE_EQ(0.0, pose.position.y);
  EXPECT_DOUBLE_EQ(0.0, pose.orientation.z);
  EXPECT_DOUBLE_EQ(1.0, pose.orientation.w);
  LineString2d stop_line{Point2d(3.5, 3.0), Point2d(3.5, -3.0)};
  const auto target = arc::createTargetPoint(path, stop_line, 1.0, 0.0, {100});
  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(2U, target->first);
  EXPECT_DOUBLE_EQ(2.5, target->second.position.x);
}
