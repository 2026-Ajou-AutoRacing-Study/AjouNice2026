#include <autoware/behavior_velocity_no_stopping_area_module/ros1_support.hpp>
#include "experimental/scene_no_stopping_area.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace
{
using autoware::behavior_velocity_planner::experimental::NoStoppingAreaModule;
using autoware::behavior_velocity_planner::experimental::Trajectory;

autoware_internal_planning_msgs::PathPointWithLaneId point(const double x)
{
  autoware_internal_planning_msgs::PathPointWithLaneId result;
  result.point.pose.position.x = x;
  result.point.pose.orientation.w = 1.0;
  result.point.longitudinal_velocity_mps = 8.0F;
  result.lane_ids = {42};
  return result;
}

autoware_perception_msgs::PredictedObjects stoppedVehicle()
{
  autoware_perception_msgs::PredictedObjects objects;
  autoware_perception_msgs::PredictedObject object;
  object.classification.resize(1);
  object.classification.front().label = autoware_perception_msgs::ObjectClassification::CAR;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  objects.objects.push_back(object);
  return objects;
}

std::shared_ptr<autoware::behavior_velocity_planner::PlannerData> plannerData()
{
  auto data = std::make_shared<autoware::behavior_velocity_planner::PlannerData>();
  auto pose = std::make_shared<geometry_msgs::PoseStamped>();
  pose->pose.orientation.w = 1.0;
  data->current_odometry = pose;
  auto velocity = std::make_shared<geometry_msgs::TwistStamped>();
  data->current_velocity = velocity;
  auto acceleration = std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  data->current_acceleration = acceleration;
  data->predicted_objects = std::make_shared<autoware_perception_msgs::PredictedObjects>(stoppedVehicle());
  data->vehicle_info_.max_longitudinal_offset_m = 0.0;
  data->vehicle_info_.vehicle_width_m=2.0;
  data->vehicle_info_.vehicle_length_m=4.0;
  data->max_stop_acceleration_threshold = -2.0;
  data->max_stop_jerk_threshold = -1.0;
  return data;
}

TEST(NoStoppingAreaScene, StopsForStoppedVehicleAndResumesWhenRtcActivated)
{
  ros::NodeHandle node;
  const lanelet::Polygon3d area(
    100, {lanelet::Point3d(101, 4.0, -1.0, 0.0), lanelet::Point3d(102, 6.0, -1.0, 0.0),
          lanelet::Point3d(103, 6.0, 1.0, 0.0), lanelet::Point3d(104, 4.0, 1.0, 0.0)});
  const lanelet::LineString3d stop_line(
    110, {lanelet::Point3d(111, 3.0, -3.0, 0.0), lanelet::Point3d(112, 3.0, 3.0, 0.0)});
  const auto reg_elem = lanelet::autoware::NoStoppingArea::make(200, {}, {area}, stop_line);
  const NoStoppingAreaModule::PlannerParam param{2.0,0.833,0.0,1.0,1.0,200.0,6.0,1.0};
  auto factor = std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
    &node, "test_no_stopping_area");
  NoStoppingAreaModule module(
    200, 42, *reg_elem, param, node, "no_stopping_test", std::make_shared<autoware::behavior_velocity_planner::NoStoppingAreaRos1Clock>(), std::make_shared<autoware_utils_debug::TimeKeeper>(), factor);
  const auto data = plannerData();
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points;
  for (int x = 0; x <= 10; ++x) points.push_back(point(x));
  const std::vector<geometry_msgs::Point> left_bound = [] {
    geometry_msgs::Point a; a.y = 3.0; geometry_msgs::Point b; b.x = 10.0; b.y = 3.0; return std::vector<geometry_msgs::Point>{a, b};
  }();
  const std::vector<geometry_msgs::Point> right_bound = [] {
    geometry_msgs::Point a; a.y = -3.0; geometry_msgs::Point b; b.x = 10.0; b.y = -3.0; return std::vector<geometry_msgs::Point>{a, b};
  }();

  auto trajectory = *Trajectory::Builder{}.build(points);
  ASSERT_TRUE(module.modifyPathVelocity(trajectory, left_bound, right_bound, *data));
  const auto stopped = trajectory.restore();
  EXPECT_FALSE(module.isSafe());
  EXPECT_FALSE(module.createDebugMarkerArray().markers.empty());
  EXPECT_TRUE(std::any_of(stopped.begin(), stopped.end(), [](const auto & p) {
    return std::abs(p.point.longitudinal_velocity_mps) < 1e-4F;
  }));

  module.setActivation(true);
  auto approved_trajectory = *Trajectory::Builder{}.build(points);
  ASSERT_TRUE(module.modifyPathVelocity(approved_trajectory, left_bound, right_bound, *data));
  const auto approved = approved_trajectory.restore();
  EXPECT_FALSE(module.isSafe());
  EXPECT_TRUE(std::all_of(approved.begin(), approved.end(), [](const auto & p) {
    return std::abs(p.point.longitudinal_velocity_mps - 8.0F) < 1e-4F;
  }));
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "no_stopping_area_scene_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
