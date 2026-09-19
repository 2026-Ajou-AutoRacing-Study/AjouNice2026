#include <autoware/behavior_path_goal_planner_module/freespace_pull_over_core.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_path_core.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_lane_adapter.hpp>
#include <autoware/behavior_path_goal_planner_module/lane_change_context_core.hpp>
#include <autoware/behavior_path_goal_planner_module/path_decision_state_core.hpp>
#include <autoware/behavior_path_goal_planner_module/freespace_bpp_adapter.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace
{
void setAstarParameters(ros::NodeHandle & node)
{
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
}

nav_msgs::OccupancyGrid freeMap()
{
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.2F;
  map.info.width = 100;
  map.info.height = 100;
  map.info.origin.position.x = -10.0;
  map.info.origin.position.y = -10.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}

lanelet::Lanelet broadLane()
{
  lanelet::LineString3d left(1, {lanelet::Point3d(2, -10.0, 5.0, 0.0), lanelet::Point3d(3, 10.0, 5.0, 0.0)});
  lanelet::LineString3d right(4, {lanelet::Point3d(5, -10.0, -5.0, 0.0), lanelet::Point3d(6, 10.0, -5.0, 0.0)});
  return lanelet::Lanelet(101, left, right);
}
}  // namespace

TEST(FreespacePullOverCore, FailsClosedBeforeMapThenReturnsAstarWaypoints)
{
  ros::NodeHandle node("/goal_freespace_core_oracle");
  setAstarParameters(node);
  const autoware::freespace_planning_algorithms::VehicleShape vehicle{4.6, 1.9, 2.7, 0.6, 1.0};
  autoware::behavior_path_planner::FreespacePullOverCore core(node, vehicle);
  geometry_msgs::Pose start;
  start.orientation.w = 1.0;
  geometry_msgs::Pose goal = start;
  goal.position.x = 4.0;
  EXPECT_FALSE(core.plan(start, goal));
  core.setMap(freeMap());
  const auto waypoints = core.plan(start, goal);
  ASSERT_TRUE(waypoints);
  EXPECT_FALSE(waypoints->waypoints.empty());
}

TEST(FreespacePullOverCore, ConvertsAstarResultToBppPath)
{
  ros::NodeHandle node("/goal_freespace_path_oracle");
  setAstarParameters(node);
  const autoware::freespace_planning_algorithms::VehicleShape vehicle{4.6, 1.9, 2.7, 0.6, 1.0};
  autoware::behavior_path_planner::FreespacePullOverCore core(node, vehicle);
  core.setMap(freeMap());
  geometry_msgs::Pose start;
  start.orientation.w = 1.0;
  auto goal = start;
  goal.position.x = 4.0;
  const auto path = core.planPath(start, goal, 2.0, {broadLane()});
  ASSERT_TRUE(path);
  ASSERT_FALSE(path->points.empty());
  EXPECT_FALSE(path->points.front().lane_ids.empty());
  EXPECT_EQ(path->points.front().lane_ids.front(), 101);
  EXPECT_GT(path->points.front().point.longitudinal_velocity_mps, 0.0F);
}

TEST(FreespacePullOverCore, PreparesForwardPartialPathWithTerminalStop)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.points.resize(3);
  for (size_t index = 0; index < path.points.size(); ++index) {
    path.points.at(index).point.pose.position.x = static_cast<double>(index);
    path.points.at(index).point.longitudinal_velocity_mps = 1.0F;
  }
  const auto result = autoware::behavior_path_planner::FreespacePullOverCore::prepareForwardPartialPaths(
    path, 2.0, 0.5);
  ASSERT_TRUE(result);
  ASSERT_EQ(result->paths.size(), 1U);
  ASSERT_EQ(result->terminal_velocity_and_accel.size(), 1U);
  EXPECT_DOUBLE_EQ(result->terminal_velocity_and_accel.front().first, 2.0);
  EXPECT_DOUBLE_EQ(result->terminal_velocity_and_accel.front().second, 0.5);
  EXPECT_FLOAT_EQ(result->paths.front().points.back().point.longitudinal_velocity_mps, 0.0F);
}

TEST(FreespacePullOverCore, RejectsReversePartialPathLikeUpstream)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.points.resize(3);
  for (size_t index = 0; index < path.points.size(); ++index) {
    path.points.at(index).point.pose.position.x = static_cast<double>(index);
  }
  path.points.at(0).point.longitudinal_velocity_mps = 1.0F;
  path.points.at(1).point.longitudinal_velocity_mps = -1.0F;
  path.points.at(2).point.longitudinal_velocity_mps = -1.0F;
  EXPECT_FALSE(autoware::behavior_path_planner::FreespacePullOverCore::prepareForwardPartialPaths(
    path, 2.0));
}

TEST(FreespacePullOverCore, CompletesGoalPathWithSourceTrimAndDirectionPolicy)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const double x : {0.0, 2.0, 3.4, 4.2}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.orientation.w = 1.0;
    point.lane_ids = {101};
    path.points.push_back(point);
  }
  geometry_msgs::Pose end_pose;
  end_pose.position.x = 5.0; end_pose.orientation.w = 1.0;
  geometry_msgs::Pose goal_pose = end_pose;
  goal_pose.position.x = 6.0;
  const auto backward = autoware::behavior_path_planner::FreespacePullOverCore::completeGoalPath(
    path, end_pose, goal_pose, true);
  ASSERT_TRUE(backward);
  ASSERT_EQ(backward->points.size(), 4U);
  EXPECT_DOUBLE_EQ(backward->points.back().point.pose.position.x, 5.0);
  const auto forward = autoware::behavior_path_planner::FreespacePullOverCore::completeGoalPath(
    path, end_pose, goal_pose, false);
  ASSERT_TRUE(forward);
  EXPECT_GT(forward->points.size(), backward->points.size());
  EXPECT_DOUBLE_EQ(forward->points.back().point.pose.position.x, 6.0);
  EXPECT_EQ(forward->points.back().lane_ids.front(), 101);
}

TEST(FreespacePullOverCore, PullOverPathCoreOwnsDeduplicatedPartialPathSequence)
{
  auto make_path = [](std::initializer_list<double> values) {
    autoware_internal_planning_msgs::PathWithLaneId path;
    for (const auto x : values) { autoware_internal_planning_msgs::PathPointWithLaneId point; point.point.pose.position.x = x; point.point.pose.orientation.w = 1.0; path.points.push_back(point); }
    return path;
  };
  geometry_msgs::Pose start; start.position.x = 1.0;
  autoware::behavior_path_planner::GoalCandidateCore goal; goal.id = 17; goal.goal_pose.position.x = 4.0;
  auto result = autoware::behavior_path_planner::PullOverPathCore::create(
    {make_path({0.0, 1.0, 2.0}), make_path({2.0, 3.0, 4.0})}, start, goal, {{2.0, 0.0}, {0.0, -1.0}});
  ASSERT_TRUE(result);
  EXPECT_EQ(result->goalId(), 17U);
  EXPECT_EQ(result->fullPath().points.size(), 5U);
  EXPECT_EQ(result->parkingPath().points.size(), 4U);
  EXPECT_DOUBLE_EQ(result->terminalPair().first, 2.0);
  EXPECT_TRUE(result->incrementPathIndex());
  EXPECT_DOUBLE_EQ(result->terminalPair().second, -1.0);
  EXPECT_FALSE(result->incrementPathIndex());
  EXPECT_FALSE(autoware::behavior_path_planner::PullOverPathCore::create({}, start, goal, {}));
}

TEST(FreespacePullOverCore, ConnectsAstarToCompletedPullOverPathOwnership)
{
  ros::NodeHandle node("/goal_freespace_complete_oracle");
  setAstarParameters(node);
  const autoware::freespace_planning_algorithms::VehicleShape vehicle{4.6, 1.9, 2.7, 0.6, 1.0};
  autoware::behavior_path_planner::FreespacePullOverCore core(node, vehicle);
  geometry_msgs::Pose start;
  start.orientation.w = 1.0;
  geometry_msgs::Pose end = start;
  end.position.x = 4.0;
  geometry_msgs::Pose goal = end;
  goal.position.x = 5.0;
  EXPECT_FALSE(core.planCompletedPullOverPath(start, end, goal, 2.0, {broadLane()}, 31, false));
  core.setMap(freeMap());
  const auto result = core.planCompletedPullOverPath(start, end, goal, 2.0, {broadLane()}, 31, false, 0.5);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->goalId(), 31U);
  EXPECT_GE(result->fullPath().points.size(), 3U);
  EXPECT_DOUBLE_EQ(result->fullPath().points.back().point.pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(result->terminalPair().first, 2.0);
}

TEST(FreespacePullOverCore, PullOverLaneAdapterFailsClosedThenUsesReadyRouteOuterLane)
{
  EXPECT_FALSE(autoware::behavior_path_planner::selectPullOverLanes({}, {}, false, 5.0, 5.0));
  auto map = std::make_shared<lanelet::LaneletMap>();
  const auto lane = broadLane();
  map->add(lane);
  auto handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  handler->setRouteLanelets({lane});
  ASSERT_TRUE(handler->isHandlerReady());
  geometry_msgs::Pose goal;
  goal.position.x = 2.0; goal.position.y = 0.0; goal.orientation.w = 1.0;
  const auto lanes = autoware::behavior_path_planner::selectPullOverLanes(handler, goal, false, 5.0, 5.0);
  ASSERT_TRUE(lanes);
  ASSERT_EQ(lanes->size(), 1U);
  EXPECT_EQ(lanes->front().id(), lane.id());
}

TEST(GoalPlannerLaneChangeContext, PreservesSourceTransitionConsistencyPolicy)
{
  using Context = autoware::behavior_path_planner::LaneChangeContextCore;
  const auto t0 = ros::Time(10.0);
  const Context::State idle{Context::NotLaneChanging{}};
  const Context::State started{Context::Started{101, t0}};
  const Context::State executing{Context::Executing{std::get<Context::Started>(started)}};
  const Context::State completed{Context::Completed{std::get<Context::Executing>(executing)}};
  EXPECT_TRUE(Context::isNotConsistentTransition(idle, started));
  EXPECT_TRUE(Context::isNotConsistentTransition(idle, executing));
  EXPECT_FALSE(Context::isNotConsistentTransition(started, executing));
  EXPECT_FALSE(Context::isNotConsistentTransition(executing, completed));
  EXPECT_TRUE(Context::isNotConsistentTransition(executing, idle));

  Context context;
  context.setState(started);
  EXPECT_FALSE(context.isInConsistentTransition());
  context.setState(executing);
  EXPECT_TRUE(context.isInConsistentTransition());
  context.setState(completed);
  EXPECT_TRUE(context.isInConsistentTransition());
  EXPECT_EQ(Context::stateToString(context.currentState()), "Completed");

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(broadLane());
  autoware::route_handler::RouteHandler incomplete_handler(map);
  const autoware_internal_planning_msgs::PathWithLaneId empty_path;
  const auto sticky = context.getNextState(empty_path, 101, incomplete_handler, ros::Time(20.0));
  EXPECT_TRUE(Context::isState<Context::Completed>(sticky));
}

TEST(GoalPlannerPathDecisionState, RequiresStableSafetyThenLatchesDecision)
{
  using Core = autoware::behavior_path_planner::PathDecisionStateCore;
  Core core;
  Core::Parameters parameters{1.0, 2.0, 10.0};
  Core::Input input;
  input.has_pull_over_path = true;
  input.current_path_safe = true;
  input.goal_safe = true;
  input.distance_to_parking_start = 5.0;
  core.transition(input, ros::Time(10.0), parameters);
  EXPECT_EQ(core.state().kind, Core::DecisionKind::NotDecided);
  core.transition(input, ros::Time(11.1), parameters);
  EXPECT_EQ(core.state().kind, Core::DecisionKind::Deciding);
  core.transition(input, ros::Time(12.0), parameters);
  EXPECT_EQ(core.state().kind, Core::DecisionKind::Deciding);
  core.transition(input, ros::Time(13.2), parameters);
  EXPECT_EQ(core.state().kind, Core::DecisionKind::Decided);
  input.current_path_safe = false;
  input.upstream_has_stopline = true;
  core.transition(input, ros::Time(14.0), parameters);
  EXPECT_EQ(core.state().kind, Core::DecisionKind::Decided);
}

TEST(GoalPlannerPathDecisionState, RejectsUnsafeStoplineCollisionAndFarStart)
{
  using Core = autoware::behavior_path_planner::PathDecisionStateCore;
  Core::Parameters parameters{0.0, 1.0, 3.0};
  Core::Input input;
  input.has_pull_over_path = true;
  input.current_path_safe = true;
  input.goal_safe = true;
  input.distance_to_parking_start = 4.0;
  Core far_core;
  far_core.transition(input, ros::Time(1.0), parameters);
  far_core.transition(input, ros::Time(1.1), parameters);
  EXPECT_EQ(far_core.state().kind, Core::DecisionKind::NotDecided);
  input.distance_to_parking_start = 1.0;
  input.parking_path_collision = true;
  far_core.transition(input, ros::Time(2.0), parameters);
  EXPECT_EQ(far_core.state().kind, Core::DecisionKind::NotDecided);
  EXPECT_FALSE(far_core.state().deciding_start_time);
}

TEST(GoalPlannerFreespaceBppAdapter, RequiresObservedEmptyObjectsAndActualBppInputs)
{
  ros::NodeHandle node("/goal_freespace_bpp_adapter_oracle");
  setAstarParameters(node);
  const autoware::freespace_planning_algorithms::VehicleShape vehicle{4.6, 1.9, 2.7, 0.6, 1.0};
  autoware::behavior_path_planner::FreespaceBppAdapter adapter(node, vehicle, 2.0, 0.5);
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  const auto lane = broadLane();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  planner_data->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  planner_data->route_handler->setRouteLanelets({lane});
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  planner_data->self_odometry = odometry;
  auto costmap = boost::make_shared<nav_msgs::OccupancyGrid>(freeMap());
  planner_data->costmap = costmap;
  geometry_msgs::Pose goal;
  goal.position.x = 4.0;
  goal.orientation.w = 1.0;
  EXPECT_FALSE(adapter.plan(planner_data, goal, goal, 42));

  auto objects = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  planner_data->dynamic_object = objects;
  const auto output = adapter.plan(planner_data, goal, goal, 42);
  ASSERT_TRUE(output);
  EXPECT_FALSE(output->path.points.empty());
  EXPECT_DOUBLE_EQ(output->path.points.back().point.pose.position.x, 4.0);

  objects->objects.emplace_back();
  EXPECT_FALSE(adapter.plan(planner_data, goal, goal, 42));
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "goal_freespace_pull_over_core_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
