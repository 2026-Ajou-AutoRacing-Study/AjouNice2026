#include <autoware/behavior_path_goal_planner_module/freespace_bpp_adapter.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_lane_adapter.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <gtest/gtest.h>

namespace
{
nav_msgs::OccupancyGrid freeCostmap(const geometry_msgs::Pose & center)
{
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.2F;
  map.info.width = 400;
  map.info.height = 400;
  map.info.origin.position.x = center.position.x - 40.0;
  map.info.origin.position.y = center.position.y - 40.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}
}

TEST(BppGoalAdapterPipeline, ConvertsReadyBppDataIntoFailClosedFreespacePullOver)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  lanelet::ConstLanelets lanes;
  geometry_msgs::Pose start;
  geometry_msgs::Pose goal;
  for (const auto & lane : map->laneletLayer) {
    const auto centerline = lane.centerline();
    if (centerline.size() < 2) continue;
    start.position.x = centerline.front().x(); start.position.y = centerline.front().y();
    const auto dx = centerline.back().x() - centerline.front().x();
    const auto dy = centerline.back().y() - centerline.front().y();
    const auto length = std::hypot(dx, dy);
    if (length < 2.0) continue;
    const auto goal_distance = std::min(10.0, length * 0.5);
    goal.position.x = centerline.front().x() + dx * goal_distance / length;
    goal.position.y = centerline.front().y() + dy * goal_distance / length;
    const auto yaw = std::atan2(dy, dx);
    start.orientation.z = std::sin(yaw * 0.5); start.orientation.w = std::cos(yaw * 0.5);
    goal.orientation = start.orientation;
    if (route_handler->planPathLaneletsBetweenCheckpoints(start, goal, &lanes)) break;
  }
  ASSERT_FALSE(lanes.empty());
  route_handler->setRouteLanelets(lanes);
  const auto selected_lanes = autoware::behavior_path_planner::selectPullOverLanes(
    route_handler, goal, false, 0.0, 0.0);
  ASSERT_TRUE(selected_lanes);
  ASSERT_FALSE(selected_lanes->empty());
  auto data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  data->route_handler = route_handler;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = start;
  data->self_odometry = odometry;
  data->costmap = boost::make_shared<nav_msgs::OccupancyGrid>(freeCostmap(start));
  auto empty_objects = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  data->dynamic_object = empty_objects;
  ros::NodeHandle node("/bpp_goal_adapter_pipeline_oracle");
  node.setParam("astar/search_method", std::string("forward"));
  node.setParam("astar/only_behind_solutions", false);
  node.setParam("astar/use_back", true);
  node.setParam("astar/adapt_expansion_distance", true);
  node.setParam("astar/expansion_distance", 0.4);
  node.setParam("astar/near_goal_distance", 4.0);
  node.setParam("astar/distance_heuristic_weight", 2.0);
  node.setParam("astar/smoothness_weight", 0.5);
  node.setParam("astar/obstacle_distance_weight", 1.7);
  node.setParam("astar/goal_lat_distance_weight", 1.0);
  node.setParam("theta_size", 120);
  const autoware::freespace_planning_algorithms::VehicleShape vehicle{4.6, 1.9, 2.7, 0.6, 1.0};
  autoware::behavior_path_planner::FreespacePullOverCore core(node, vehicle);
  core.setMap(*data->costmap);
  ASSERT_TRUE(core.plan(start, goal));
  ASSERT_TRUE(core.planCompletedPullOverPath(start, goal, goal, 2.0, *selected_lanes, 88, false, 0.0));
  autoware::behavior_path_planner::FreespaceBppAdapter adapter(node, vehicle, 2.0, 0.0);
  const auto output = adapter.plan(data, goal, goal, 88);
  ASSERT_TRUE(output);
  EXPECT_FALSE(output->path.points.empty());
  auto occupied_objects = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  occupied_objects->objects.emplace_back();
  data->dynamic_object = occupied_objects;
  EXPECT_FALSE(adapter.plan(data, goal, goal, 88));
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bpp_goal_adapter_pipeline_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
