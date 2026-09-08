#include <autoware/motion_velocity_planner_common/planner_data.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <gtest/gtest.h>

namespace
{
std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory()
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> points(3);
  for (size_t i = 0; i < points.size(); ++i) {
    points.at(i).pose.position.x = static_cast<double>(i * 5U);
    points.at(i).pose.orientation.w = 1.0;
  }
  return points;
}

autoware_perception_msgs::PredictedObject objectAt(const double x, const double y)
{
  autoware_perception_msgs::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 2.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.y = 1.0;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  return object;
}
}  // namespace

TEST(MotionVelocityPlannerDataObject, CachesSpatialAndRelativeVelocityQueries)
{
  autoware::motion_velocity_planner::PlannerData::Object object(objectAt(5.0, 3.0));
  auto trajectory_polygon = autoware_utils_geometry::Polygon2d{};
  trajectory_polygon.outer() = {{0.0, -1.0}, {10.0, -1.0}, {10.0, 1.0}, {0.0, 1.0}, {0.0, -1.0}};
  boost::geometry::correct(trajectory_polygon);
  geometry_msgs::Point ego_position;

  EXPECT_NEAR(object.get_dist_to_traj_poly({trajectory_polygon}), 1.0, 1e-6);
  EXPECT_NEAR(object.get_dist_to_traj_lateral(trajectory()), 3.0, 1e-6);
  EXPECT_NEAR(object.get_dist_from_ego_longitudinal(trajectory(), ego_position), 5.0, 1e-6);
  EXPECT_NEAR(object.get_lon_vel_relative_to_traj(trajectory()), 2.0, 1e-6);
  EXPECT_NEAR(object.get_lat_vel_relative_to_traj(trajectory()), -1.0, 1e-6);
}

TEST(MotionVelocityPlannerDataObject, ReplacesObjectCacheOnPredictedObjectUpdate)
{
  autoware::motion_velocity_planner::PlannerData data;
  autoware_perception_msgs::PredictedObjects input;
  input.header.frame_id = "map";
  input.objects.push_back(objectAt(1.0, 0.0));
  input.objects.push_back(objectAt(2.0, 0.0));
  data.process_predicted_objects(input);

  ASSERT_EQ(data.predicted_objects_header.frame_id, "map");
  ASSERT_EQ(data.objects.size(), 2U);
  EXPECT_DOUBLE_EQ(data.objects.at(1)->predicted_object.kinematics.initial_pose_with_covariance.pose.position.x, 2.0);
}

TEST(MotionVelocityPlannerDataObject, UsesHighestConfidencePredictedPathAndFallsBackSafely)
{
  auto input = objectAt(5.0, 0.0);
  autoware_perception_msgs::PredictedPath low_confidence;
  low_confidence.confidence = 0.1F;
  low_confidence.time_step.sec = 1;
  low_confidence.path.resize(2);
  low_confidence.path.at(0).position.x = 0.0;
  low_confidence.path.at(1).position.x = 2.0;
  low_confidence.path.at(0).orientation.w = 1.0;
  low_confidence.path.at(1).orientation.w = 1.0;
  autoware_perception_msgs::PredictedPath high_confidence = low_confidence;
  high_confidence.confidence = 0.9F;
  high_confidence.path.at(0).position.x = 10.0;
  high_confidence.path.at(1).position.x = 20.0;
  input.kinematics.predicted_paths = {low_confidence, high_confidence};
  autoware::motion_velocity_planner::PlannerData::Object object(input);

  EXPECT_NEAR(
    object.get_predicted_current_pose(ros::Time(100.5), ros::Time(100.0)).position.x, 15.0, 1e-6);
  EXPECT_DOUBLE_EQ(object.calc_predicted_pose(ros::Time(99.0), ros::Time(100.0)).position.x, 5.0);
}

TEST(MotionVelocityPlannerDataObject, ExposesObstacleStopVehicleStateAndNearestContracts)
{
  autoware::motion_velocity_planner::PlannerData data;
  data.ego_nearest_dist_threshold = 2.0;
  data.ego_nearest_yaw_threshold = 1.0;
  data.current_odometry.twist.twist.linear.x = 4.0;
  data.current_acceleration.accel.accel.linear.x = -1.0;
  data.vehicle_info_.max_longitudinal_offset_m = 2.0;
  data.trajectory_polygon_collision_check.decimate_trajectory_step_length = 2.0;
  geometry_msgs::Pose pose;
  pose.position.x = 5.1;
  pose.orientation.w = 1.0;

  EXPECT_EQ(data.find_index(trajectory(), pose), 1U);
  EXPECT_EQ(data.find_segment_index(trajectory(), pose), 1U);
  EXPECT_DOUBLE_EQ(data.current_odometry.twist.twist.linear.x, 4.0);
  EXPECT_DOUBLE_EQ(data.current_acceleration.accel.accel.linear.x, -1.0);
  EXPECT_DOUBLE_EQ(data.trajectory_polygon_collision_check.decimate_trajectory_step_length, 2.0);
}

TEST(MotionVelocityPlannerDataObject, PreservesTrafficSignalLookupAndFailsClosedWithoutProviders)
{
  autoware::motion_velocity_planner::PlannerData data;
  autoware::motion_velocity_planner::TrafficSignalStamped raw;
  raw.stamp = ros::Time(12.5);
  raw.signal.traffic_light_group_id = 101;
  data.traffic_light_id_map_raw_.emplace(42, raw);

  autoware::motion_velocity_planner::TrafficSignalStamped retained;
  retained.stamp = ros::Time(13.0);
  retained.signal.traffic_light_group_id = 202;
  data.traffic_light_id_map_last_observed_.emplace(42, retained);

  ASSERT_TRUE(data.get_traffic_signal(42).has_value());
  EXPECT_EQ(data.get_traffic_signal(42)->signal.traffic_light_group_id, 101);
  ASSERT_TRUE(data.get_traffic_signal(42, true).has_value());
  EXPECT_EQ(data.get_traffic_signal(42, true)->signal.traffic_light_group_id, 202);
  EXPECT_FALSE(data.get_traffic_signal(999).has_value());
  EXPECT_TRUE(data.calculate_map_stop_points(trajectory()).empty());
  EXPECT_FALSE(data.calculate_min_deceleration_distance(0.0).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
