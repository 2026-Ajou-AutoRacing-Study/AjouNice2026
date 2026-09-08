#include <autoware/behavior_path_goal_planner_module/goal_candidate_update_core.hpp>

#include <autoware_perception_msgs/PredictedObject.h>

#include <gtest/gtest.h>

namespace
{
lanelet::Lanelet lane()
{
  lanelet::LineString3d left(1, {lanelet::Point3d(2, 0.0, 2.0, 0.0), lanelet::Point3d(3, 20.0, 2.0, 0.0)});
  lanelet::LineString3d right(4, {lanelet::Point3d(5, 0.0, -2.0, 0.0), lanelet::Point3d(6, 20.0, -2.0, 0.0)});
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
  return {{1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}};
}

autoware_perception_msgs::PredictedObjects objects()
{
  autoware_perception_msgs::PredictedObject object;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.kinematics.initial_pose_with_covariance.pose = poseAt(5.0);
  autoware_perception_msgs::PredictedObjects output;
  output.objects = {object};
  return output;
}

autoware_internal_planning_msgs::PathWithLaneId centerline()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const double x : {0.0, 5.0, 10.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose = poseAt(x);
    point.lane_ids = {101};
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

TEST(GoalCandidateUpdateCore, CountsRanksThenEvaluatesSafetyInSourceOrder)
{
  autoware::behavior_path_planner::GoalCandidates candidates(2);
  candidates.at(0).id = 1;
  candidates.at(0).goal_pose = poseAt(8.0);
  candidates.at(0).distance_from_original_goal = 1.0;
  candidates.at(1).id = 2;
  candidates.at(1).goal_pose = poseAt(2.0);
  candidates.at(1).distance_from_original_goal = 2.0;
  autoware::behavior_path_planner::GoalCandidateUpdateParameters parameters;
  parameters.prioritize_goals_before_objects = true;
  parameters.goal_priority = "minimum_longitudinal_distance";
  parameters.object_collision_margin = 0.1;
  parameters.safety.vehicle_width = 2.0;
  parameters.safety.object_recognition_collision_check_hard_margin = 0.1;

  autoware::behavior_path_planner::updateGoalCandidates(
    &candidates, objects(), {lane()}, centerline(), footprint(), parameters, nullptr);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates.at(0).id, 2U);
  EXPECT_EQ(candidates.at(0).num_objects_to_avoid, 0U);
  EXPECT_TRUE(candidates.at(0).is_safe);
  EXPECT_EQ(candidates.at(1).id, 1U);
  EXPECT_EQ(candidates.at(1).num_objects_to_avoid, 1U);
  EXPECT_TRUE(candidates.at(1).is_safe);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
