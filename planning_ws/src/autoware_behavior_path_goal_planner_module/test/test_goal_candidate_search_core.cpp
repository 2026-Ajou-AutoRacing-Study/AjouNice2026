#include <autoware/behavior_path_goal_planner_module/goal_candidate_search_core.hpp>

#include <gtest/gtest.h>

namespace
{
geometry_msgs::Pose poseAt(const double x, const double y = 0.0)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

lanelet::ConstLineString3d boundary()
{
  lanelet::Points3d points;
  points.emplace_back(1, 0.0, 4.0, 0.0);
  points.emplace_back(2, 20.0, 4.0, 0.0);
  return lanelet::ConstLineString3d(10, points);
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
}  // namespace

TEST(GoalCandidateSearchCore, ConnectsBoundarySamplingMarginAndLateralCandidates)
{
  autoware::behavior_path_planner::GoalCandidateSearchParameters parameters;
  parameters.left_side_parking = true;
  parameters.base_link2front = 2.0;
  parameters.vehicle_length = 4.0;
  parameters.vehicle_width = 2.0;
  parameters.margin_from_boundary = 0.5;
  parameters.backward_goal_search_length = 2.0;
  parameters.forward_goal_search_length = 2.0;
  parameters.longitudinal_interval = 2.0;
  parameters.max_lateral_offset = 1.0;
  parameters.lateral_offset_interval = 1.0;
  autoware::behavior_path_planner::GoalCandidateAreaConstraints areas;
  areas.departure_check_area = rectangle(-10.0, -10.0, 30.0, 10.0);

  const auto candidates = autoware::behavior_path_planner::searchGoalCandidatesAlongBoundary(
    boundary(), poseAt(10.0), parameters, footprint(), areas);
  ASSERT_EQ(candidates.size(), 4U);
  // cropBoundary is centered on the front reference (x=12), so its first
  // sample is x=10 and the base-link goal is x=8. The staged signed-boundary
  // provider determines the positive local-y minimum-margin direction here.
  EXPECT_NEAR(candidates.at(0).goal_pose.position.x, 8.0, 1e-9);
  EXPECT_NEAR(candidates.at(0).goal_pose.position.y, 2.5, 1e-9);
  EXPECT_NEAR(candidates.at(1).goal_pose.position.y, 1.5, 1e-9);
  EXPECT_NEAR(candidates.at(0).distance_from_original_goal, 2.0, 1e-9);
  EXPECT_NEAR(candidates.at(2).distance_from_original_goal, 0.0, 1e-9);
}

TEST(GoalCandidateSearchCore, RejectsInvalidBoundarySearchConfiguration)
{
  autoware::behavior_path_planner::GoalCandidateSearchParameters parameters;
  parameters.longitudinal_interval = 0.0;
  autoware::behavior_path_planner::GoalCandidateAreaConstraints areas;
  areas.departure_check_area = rectangle(-10.0, -10.0, 30.0, 10.0);
  EXPECT_TRUE(autoware::behavior_path_planner::searchGoalCandidatesAlongBoundary(
    boundary(), poseAt(10.0), parameters, footprint(), areas).empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
