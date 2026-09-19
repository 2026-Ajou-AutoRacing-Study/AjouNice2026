#include <autoware/behavior_path_goal_planner_module/goal_candidate_area_filter.hpp>

#include <gtest/gtest.h>

namespace
{
geometry_msgs::Pose poseAt(const double x, const double y)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

autoware_utils_geometry::LinearRing2d footprint()
{
  return {{0.5, 0.5}, {0.5, -0.5}, {-0.5, -0.5}, {-0.5, 0.5}, {0.5, 0.5}};
}

autoware_utils_geometry::Polygon2d rectangle(
  const double min_x, const double min_y, const double max_x, const double max_y)
{
  autoware_utils_geometry::Polygon2d polygon;
  polygon.outer() = {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}, {min_x, min_y}};
  return polygon;
}
}  // namespace

TEST(GoalCandidateAreaFilter, AppliesSourceAreaOrder)
{
  autoware::behavior_path_planner::GoalCandidateAreaConstraints constraints;
  constraints.use_bus_stop_area = true;
  constraints.bus_stop_areas = {rectangle(-5.0, -5.0, 5.0, 5.0)};
  constraints.departure_check_area = rectangle(-6.0, -6.0, 6.0, 6.0);
  EXPECT_TRUE(autoware::behavior_path_planner::isGoalCandidatePoseAllowedByAreas(
    footprint(), poseAt(0.0, 0.0), constraints));

  constraints.no_parking_areas = {rectangle(-1.0, -1.0, 1.0, 1.0)};
  EXPECT_FALSE(autoware::behavior_path_planner::isGoalCandidatePoseAllowedByAreas(
    footprint(), poseAt(0.0, 0.0), constraints));
}

TEST(GoalCandidateAreaFilter, RejectsOutsideBusStopAndDepartureAreas)
{
  autoware::behavior_path_planner::GoalCandidateAreaConstraints constraints;
  constraints.use_bus_stop_area = true;
  constraints.bus_stop_areas = {rectangle(-2.0, -2.0, 2.0, 2.0)};
  constraints.departure_check_area = rectangle(-3.0, -3.0, 3.0, 3.0);
  EXPECT_FALSE(autoware::behavior_path_planner::isGoalCandidatePoseAllowedByAreas(
    footprint(), poseAt(4.0, 0.0), constraints));

  constraints.use_bus_stop_area = false;
  EXPECT_FALSE(autoware::behavior_path_planner::isGoalCandidatePoseAllowedByAreas(
    footprint(), poseAt(4.0, 0.0), constraints));
}

TEST(GoalCandidateAreaFilter, ConvertsClosedLaneletAreaPolygonToRos1BoostPolygon)
{
  lanelet::Polygon3d polygon(
    10, {lanelet::Point3d(1, 0.0, 0.0, 0.0), lanelet::Point3d(2, 2.0, 0.0, 0.0),
         lanelet::Point3d(3, 2.0, 2.0, 0.0), lanelet::Point3d(4, 0.0, 2.0, 0.0)});
  const auto converted = autoware::behavior_path_planner::toGoalCandidateAreaPolygon(polygon);
  ASSERT_EQ(converted.outer().size(), 5U);
  EXPECT_NEAR(converted.outer().front().x(), converted.outer().back().x(), 1e-9);
  EXPECT_NEAR(converted.outer().front().y(), converted.outer().back().y(), 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
