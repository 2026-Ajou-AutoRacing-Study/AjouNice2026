#include <autoware/behavior_path_goal_planner_module/goal_candidate_object_filter.hpp>

#include <autoware_perception_msgs/PredictedObject.h>

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

autoware_perception_msgs::PredictedObject boxAt(const double x, const double y)
{
  autoware_perception_msgs::PredictedObject object;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.kinematics.initial_pose_with_covariance.pose = poseAt(x, y);
  return object;
}
}  // namespace

TEST(GoalCandidateObjectFilter, SelectsOnlyObjectsWithinLateralMargin)
{
  autoware_perception_msgs::PredictedObjects objects;
  objects.objects = {boxAt(4.0, 1.3), boxAt(4.0, 4.0)};
  const auto inside = autoware::behavior_path_planner::filterGoalCandidateObjectsByLateralDistance(
    poseAt(0.0, 0.0), 2.0, objects, 0.4, true);
  ASSERT_EQ(inside.objects.size(), 1U);
  EXPECT_NEAR(inside.objects.front().kinematics.initial_pose_with_covariance.pose.position.y, 1.3, 1e-9);
}

TEST(GoalCandidateObjectFilter, KeepsOnlyObjectsOutsideWhenRequested)
{
  autoware_perception_msgs::PredictedObjects objects;
  objects.objects = {boxAt(4.0, 1.3), boxAt(4.0, 4.0)};
  const auto outside = autoware::behavior_path_planner::filterGoalCandidateObjectsByLateralDistance(
    poseAt(0.0, 0.0), 2.0, objects, 0.4, false);
  ASSERT_EQ(outside.objects.size(), 1U);
  EXPECT_NEAR(outside.objects.front().kinematics.initial_pose_with_covariance.pose.position.y, 4.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
