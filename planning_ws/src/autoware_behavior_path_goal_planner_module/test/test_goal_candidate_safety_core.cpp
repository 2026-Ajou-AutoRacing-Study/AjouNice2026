#include <autoware/behavior_path_goal_planner_module/goal_candidate_safety_core.hpp>

#include <autoware_perception_msgs/PredictedObject.h>

#include <gtest/gtest.h>

namespace
{
using autoware::behavior_path_planner::GoalCandidate;
using autoware::behavior_path_planner::GoalCandidateSafetyParameters;
using autoware::behavior_path_planner::GoalCandidates;
using autoware::behavior_path_planner::OccupancyGridBasedCollisionDetector;
using autoware::behavior_path_planner::OccupancyGridMapParam;
using autoware::behavior_path_planner::VehicleShape;

geometry_msgs::Pose poseAt(const double x, const double y = 0.0)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

autoware_perception_msgs::PredictedObject boxAt(const double x, const double y = 0.0)
{
  autoware_perception_msgs::PredictedObject object;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.kinematics.initial_pose_with_covariance.pose = poseAt(x, y);
  return object;
}

autoware_utils_geometry::LinearRing2d footprint()
{
  return {{2.0, 1.0}, {2.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}, {2.0, 1.0}};
}

GoalCandidateSafetyParameters safetyParameters()
{
  GoalCandidateSafetyParameters parameters;
  parameters.object_recognition_collision_check_hard_margin = 0.1;
  parameters.vehicle_width = 2.0;
  parameters.longitudinal_margin = 2.0;
  parameters.base_link2front = 2.0;
  parameters.base_link2rear = -1.0;
  return parameters;
}

std::shared_ptr<OccupancyGridBasedCollisionDetector> detectorWithObstacleAt(
  const int obstacle_x, const int obstacle_y)
{
  nav_msgs::OccupancyGrid map;
  map.info.resolution = 1.0;
  map.info.width = 40;
  map.info.height = 40;
  map.info.origin = poseAt(-20.0, -20.0);
  map.data.assign(map.info.width * map.info.height, 0);
  map.data.at(obstacle_y * map.info.width + obstacle_x) = 100;
  auto detector = std::make_shared<OccupancyGridBasedCollisionDetector>();
  detector->setParam(OccupancyGridMapParam{VehicleShape{4.0, 2.0, 1.0}, 72, 50});
  detector->setMap(map);
  return detector;
}
}  // namespace

TEST(GoalCandidateSafetyCore, RejectsFootprintAndLongitudinalObjectConflicts)
{
  auto parameters = safetyParameters();
  autoware_perception_msgs::PredictedObjects objects;
  objects.objects = {boxAt(1.0), boxAt(3.0)};
  GoalCandidates candidates(2);
  candidates.at(0).goal_pose = poseAt(0.0);
  candidates.at(1).goal_pose = poseAt(20.0);

  autoware::behavior_path_planner::updateGoalCandidateSafety(
    &candidates, footprint(), objects, parameters, nullptr);

  EXPECT_FALSE(candidates.at(0).is_safe);
  EXPECT_TRUE(candidates.at(1).is_safe);
}

TEST(GoalCandidateSafetyCore, RejectsClearFootprintWhenObjectViolatesLongitudinalMargin)
{
  const auto parameters = safetyParameters();
  autoware_perception_msgs::PredictedObjects objects;
  // The closest box face is 3 m ahead: clear of the 2 m front footprint, but
  // inside the 2 m source longitudinal margin measured from the front axle.
  objects.objects = {boxAt(3.5)};

  EXPECT_FALSE(autoware::behavior_path_planner::checkGoalCandidateCollision(
    footprint(), poseAt(0.0), objects, parameters, nullptr));
  EXPECT_TRUE(autoware::behavior_path_planner::checkGoalCandidateLongitudinalMargin(
    poseAt(0.0), objects, parameters, nullptr));
}

TEST(GoalCandidateSafetyCore, RejectsOccupancyCollisionAndPreservesClearCandidate)
{
  auto parameters = safetyParameters();
  parameters.use_occupancy_grid_for_goal_search = true;
  parameters.use_occupancy_grid_for_goal_longitudinal_margin = true;
  parameters.occupancy_grid_collision_check_margin = 0.5;
  parameters.longitudinal_margin = 3.0;
  const auto detector = detectorWithObstacleAt(20, 20);
  autoware_perception_msgs::PredictedObjects no_objects;
  GoalCandidates candidates(2);
  candidates.at(0).goal_pose = poseAt(0.0);
  candidates.at(1).goal_pose = poseAt(10.0);

  autoware::behavior_path_planner::updateGoalCandidateSafety(
    &candidates, footprint(), no_objects, parameters, detector);

  EXPECT_FALSE(candidates.at(0).is_safe);
  EXPECT_TRUE(candidates.at(1).is_safe);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
