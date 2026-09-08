#include "autoware/planning_validator_intersection_collision_checker/lanelet_selection_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::LaneletRoute route_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  primitive.primitive_type = "lanelet";
  autoware_planning_msgs::LaneletSegment segment;
  segment.preferred_primitive = primitive;
  segment.primitives.push_back(primitive);
  autoware_planning_msgs::LaneletRoute route;
  route.segments.push_back(segment);
  return route;
}

autoware_planning_msgs::Trajectory trajectory_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & point : lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = point.x();
    trajectory_point.pose.position.y = point.y();
    trajectory_point.pose.position.z = point.z();
    trajectory_point.pose.orientation.w = 1.0;
    trajectory.points.push_back(trajectory_point);
  }
  return trajectory;
}
}  // namespace

TEST(LaneletSelectionCore, FailsClosedWithoutAReadyRoute)
{
  autoware::route_handler::RouteHandler route_handler;
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points.resize(2U);
  geometry_msgs::Pose ego_pose;
  EXPECT_FALSE(autoware::planning_validator::intersection_collision_checker::select_trajectory_lanelets(
    trajectory, route_handler, ego_pose).has_value());
}

TEST(LaneletSelectionCore, UsesOnlyTheRouteBoundLaneletSequence)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto route_lane = *map->laneletLayer.begin();
  const auto trajectory = trajectory_for(route_lane);
  ASSERT_GE(trajectory.points.size(), 2U);
  autoware::route_handler::RouteHandler route_handler(map);
  route_handler.setRoute(route_for(route_lane));
  ASSERT_TRUE(route_handler.isHandlerReady());
  geometry_msgs::Pose ego_pose = trajectory.points.front().pose;

  const auto selected = autoware::planning_validator::intersection_collision_checker::select_trajectory_lanelets(
    trajectory, route_handler, ego_pose);
  ASSERT_TRUE(selected.has_value());
  ASSERT_FALSE(selected->trajectory_lanelets.empty());
  EXPECT_EQ(selected->trajectory_lanelets.front().id(), route_lane.id());
}
