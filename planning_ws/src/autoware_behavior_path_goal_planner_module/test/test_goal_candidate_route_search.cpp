#include <autoware/behavior_path_goal_planner_module/goal_candidate_route_search.hpp>

#include <gtest/gtest.h>

namespace
{
lanelet::Lanelet broadLane()
{
  lanelet::LineString3d left(1, {lanelet::Point3d(2, -10.0, 5.0, 0.0), lanelet::Point3d(3, 10.0, 5.0, 0.0)});
  lanelet::LineString3d right(4, {lanelet::Point3d(5, -10.0, -5.0, 0.0), lanelet::Point3d(6, 10.0, -5.0, 0.0)});
  return lanelet::Lanelet(101, left, right);
}

geometry_msgs::Pose poseAt(const double x)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  return pose;
}

autoware_utils_geometry::LinearRing2d footprint()
{
  return {{0.25, 0.25}, {0.25, -0.25}, {-0.25, -0.25}, {-0.25, 0.25}, {0.25, 0.25}};
}

autoware_utils_geometry::Polygon2d rectangle(const double min_x, const double min_y, const double max_x, const double max_y)
{
  autoware_utils_geometry::Polygon2d polygon;
  polygon.outer() = {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}, {min_x, min_y}};
  return polygon;
}

autoware::behavior_path_planner::GoalCandidateSearchParameters parameters()
{
  autoware::behavior_path_planner::GoalCandidateSearchParameters result;
  result.left_side_parking = true;
  result.base_link2front = 2.0;
  result.vehicle_length = 4.0;
  result.vehicle_width = 2.0;
  result.margin_from_boundary = 0.5;
  result.backward_goal_search_length = 2.0;
  result.forward_goal_search_length = 2.0;
  result.longitudinal_interval = 2.0;
  result.max_lateral_offset = 1.0;
  result.lateral_offset_interval = 1.0;
  return result;
}
}  // namespace

TEST(GoalCandidateRouteSearch, FailsClosedWithoutReadyRouteHandler)
{
  autoware::behavior_path_planner::GoalCandidateAreaConstraints areas;
  areas.departure_check_area = rectangle(-20.0, -20.0, 20.0, 20.0);
  EXPECT_TRUE(autoware::behavior_path_planner::searchGoalCandidatesFromRoute(
    {}, poseAt(0.0), parameters(), footprint(), areas).empty());
}

TEST(GoalCandidateRouteSearch, SelectsPullOverBoundaryFromReadyRoute)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  const auto lane = broadLane();
  map->add(lane);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  route_handler->setRouteLanelets({lane});
  ASSERT_TRUE(route_handler->isHandlerReady());
  autoware::behavior_path_planner::GoalCandidateAreaConstraints areas;
  areas.departure_check_area = rectangle(-20.0, -20.0, 20.0, 20.0);
  const auto candidates = autoware::behavior_path_planner::searchGoalCandidatesFromRoute(
    route_handler, poseAt(0.0), parameters(), footprint(), areas);
  EXPECT_FALSE(candidates.empty());
  EXPECT_EQ(candidates.front().id, 0U);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
