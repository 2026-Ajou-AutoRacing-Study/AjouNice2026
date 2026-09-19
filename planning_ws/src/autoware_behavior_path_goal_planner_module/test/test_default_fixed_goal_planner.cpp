#include <autoware/behavior_path_goal_planner_module/default_fixed_goal_planner.hpp>

#include <autoware_planning_msgs/LaneletRoute.h>
#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace
{
lanelet::Lanelet straightLane()
{
  lanelet::LineString3d left(1, {lanelet::Point3d(2, 0.0, 1.0, 0.0), lanelet::Point3d(3, 12.0, 1.0, 0.0)});
  lanelet::LineString3d right(4, {lanelet::Point3d(5, 0.0, -1.0, 0.0), lanelet::Point3d(6, 12.0, -1.0, 0.0)});
  return lanelet::Lanelet(10, left, right);
}

autoware_internal_planning_msgs::PathWithLaneId sourcePath()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.header.frame_id = "map";
  for (int index = 0; index != 7; ++index) {
    autoware_internal_planning_msgs::PathPointWithLaneId point{};
    point.point.pose.position.x = 2.0 * index;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 5.0;
    point.lane_ids = {10};
    path.points.push_back(point);
  }
  return path;
}

std::shared_ptr<autoware::behavior_path_planner::PlannerData> plannerData(
  const lanelet::Lanelet & lane)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  auto data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  data->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  autoware_planning_msgs::LaneletRoute route{};
  route.start_pose.orientation.w = 1.0;
  route.goal_pose.position.x = 9.0;
  route.goal_pose.orientation.w = 1.0;
  autoware_planning_msgs::LaneletPrimitive primitive{};
  primitive.id = lane.id();
  primitive.primitive_type = "lanelet";
  autoware_planning_msgs::LaneletSegment segment{};
  segment.preferred_primitive = primitive;
  segment.primitives.push_back(primitive);
  route.segments.push_back(segment);
  data->route_handler->setRoute(route);
  data->parameters.refine_goal_search_radius_range = 4.0;
  data->parameters.output_path_interval = 0.5;
  return data;
}
}  // namespace

TEST(DefaultFixedGoalPlanner, ReconstructsPathToRouteGoalAndRetainsReferencePath)
{
  const auto lane = straightLane();
  const auto data = plannerData(lane);
  ASSERT_TRUE(data->route_handler->isHandlerReady());
  autoware::behavior_path_planner::BehaviorModuleOutput upstream{};
  upstream.path = sourcePath();
  upstream.reference_path.header.frame_id = "upstream_reference";
  autoware::behavior_path_planner::DefaultFixedGoalPlanner planner;
  planner.setPreviousModuleOutput(upstream);

  const auto output = planner.plan(data);
  ASSERT_FALSE(output.path.points.empty());
  EXPECT_NEAR(output.path.points.back().point.pose.position.x, 9.0, 1e-6);
  EXPECT_DOUBLE_EQ(output.path.points.back().point.longitudinal_velocity_mps, 0.0);
  EXPECT_EQ(output.reference_path.header.frame_id, "upstream_reference");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
