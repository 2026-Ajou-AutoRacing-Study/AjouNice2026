#include "autoware/planning_validator_intersection_collision_checker/left_active_target_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::LaneletRoute route_for(
  const lanelet::ConstLanelet & first, const lanelet::ConstLanelet & second)
{
  autoware_planning_msgs::LaneletRoute route;
  for (const auto & lane : {first, second}) {
    autoware_planning_msgs::LaneletPrimitive primitive;
    primitive.id = lane.id();
    primitive.primitive_type = "lanelet";
    autoware_planning_msgs::LaneletSegment segment;
    segment.preferred_primitive = primitive;
    segment.primitives.push_back(primitive);
    route.segments.push_back(segment);
  }
  return route;
}

autoware_planning_msgs::Trajectory trajectory_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::Trajectory trajectory;
  int32_t time = 0;
  for (const auto & point : lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint item;
    item.pose.position.x = point.x();
    item.pose.position.y = point.y();
    item.pose.position.z = point.z();
    item.pose.orientation.w = 1.0;
    item.time_from_start.sec = time++;
    trajectory.points.push_back(item);
  }
  return trajectory;
}
}  // namespace

TEST(LeftActiveTargetCore, UsesActualRouteNextAndPredecessorTopology)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_INTERSECTION_FIXTURE_PATH);
  ASSERT_TRUE(map);
  autoware::route_handler::RouteHandler route_handler(map);
  lanelet::ConstLanelet turn;
  lanelet::ConstLanelet next;
  for (const auto & lane : map->laneletLayer) {
    if (!lane.hasAttribute("turn_direction") ||
        lane.attribute("turn_direction").value() != "left") {
      continue;
    }
    const auto next_lanes = route_handler.getNextLanelets(lane);
    if (!next_lanes.empty() && !route_handler.getPreviousLanelets(next_lanes.front()).empty()) {
      turn = lane;
      next = next_lanes.front();
      break;
    }
  }
  ASSERT_NE(turn.id(), lanelet::InvalId);
  route_handler.setRoute(route_for(turn, next));
  ASSERT_TRUE(route_handler.isHandlerReady());
  lanelet::ConstLanelet route_next;
  ASSERT_TRUE(route_handler.getNextLaneletWithinRoute(turn, &route_next));
  EXPECT_EQ(route_next.id(), next.id());
  EXPECT_FALSE(route_handler.getPreviousLanelets(route_next).empty());

  autoware::planning_validator::intersection_collision_checker::LaneletSelection ego_lanes;
  ego_lanes.turn_lanelets = {turn};
  ego_lanes.trajectory_lanelets = {turn};
  autoware::planning_validator::intersection_collision_checker_ros1::Params params{};
  params.detection_range = 100.0;
  params.left_turn.check_turning_lanes = true;
  params.left_turn.check_traffic_signal = true;
  const auto targets =
    autoware::planning_validator::intersection_collision_checker::make_left_active_targets(
      route_handler, trajectory_for(turn), ego_lanes, 0U, 100.0, params, false);
  for (const auto & target : targets) {
    EXPECT_NE(target.id, turn.id());
    EXPECT_EQ(target.lanelets.back().id(), target.id);
  }
  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::make_left_active_targets(
      route_handler, trajectory_for(turn), ego_lanes, 0U, 100.0, params, true).empty());
}
