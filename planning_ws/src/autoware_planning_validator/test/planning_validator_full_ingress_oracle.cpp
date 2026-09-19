#include "autoware/planning_validator/planning_validator_manager.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace
{
autoware_planning_msgs::LaneletRoute route_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  primitive.primitive_type = "lanelet";
  autoware_planning_msgs::LaneletSegment segment;
  segment.preferred_primitive = primitive;
  segment.primitives.push_back(primitive);
  autoware_planning_msgs::LaneletRoute route;
  route.segments.push_back(segment);
  return route;
}

autoware_planning_msgs::Trajectory trajectory_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::Trajectory result;
  result.header.stamp = ros::Time::now();
  for (const auto & center : lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = center.x();
    point.pose.position.y = center.y();
    point.pose.position.z = center.z();
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 0.0F;
    result.points.push_back(point);
  }
  return result;
}

TEST(PlanningValidatorFullIngress, AllFourProvidersAcceptCompleteInactiveRouteContext)
{
  ros::NodeHandle private_nh("/aw149_full_ingress");
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  const auto route = route_for(lane);
  const auto trajectory = trajectory_for(lane);
  ASSERT_GE(trajectory.points.size(), 2U);
  autoware::route_handler::RouteHandler route_handler(map);
  route_handler.setRoute(route);
  ASSERT_TRUE(route_handler.isHandlerReady());

  nav_msgs::Odometry odometry;
  const auto center = lane.centerline()[lane.centerline().size() / 2U];
  odometry.pose.pose.position.x = center.x();
  odometry.pose.pose.position.y = center.y();
  odometry.pose.pose.position.z = center.z();
  odometry.pose.pose.orientation.w = 1.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  sensor_msgs::PointCloud2 pointcloud;
  pointcloud.header.frame_id = "unused_lidar";
  autoware_map_msgs::LaneletMapBin lanelet_map;
  autoware_perception_msgs::TrafficLightGroupArray signals;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.wheel_base_m = 2.7;
  vehicle.front_overhang_m = 1.0;
  vehicle.rear_overhang_m = 1.0;
  vehicle.vehicle_height_m = 2.0;
  // Complete-input acceptance requires a physically valid vehicle footprint.
  vehicle.vehicle_width_m = 2.0;
  tf2_ros::Buffer tf_buffer;
  tf_buffer.setUsingDedicatedThread(true);

  autoware::planning_validator::PlanningValidatorManager manager;
  for (const auto & name : {
      "autoware::planning_validator::TrajectoryCheckerRos1",
      "autoware::planning_validator::LatencyCheckerRos1",
      "autoware::planning_validator::IntersectionCollisionCheckerRos1",
      "autoware::planning_validator::RearCollisionCheckerRos1"}) {
    ASSERT_TRUE(manager.load_plugin(private_nh, name)) << name;
  }
  const auto status = manager.validate({
    trajectory, &odometry, &pointcloud, &acceleration, &route, &lanelet_map, &signals,
    &trajectory, &vehicle, &tf_buffer, &route_handler});
  EXPECT_TRUE(status.is_valid_size);
  EXPECT_TRUE(status.is_valid_finite_value);
  EXPECT_TRUE(status.is_valid_interval);
  EXPECT_TRUE(status.is_valid_relative_angle);
  EXPECT_TRUE(status.is_valid_curvature);
  EXPECT_TRUE(status.is_valid_lateral_acc);
  EXPECT_TRUE(status.is_valid_lateral_jerk);
  EXPECT_TRUE(status.is_valid_longitudinal_max_acc);
  EXPECT_TRUE(status.is_valid_longitudinal_min_acc);
  EXPECT_TRUE(status.is_valid_steering);
  EXPECT_TRUE(status.is_valid_steering_rate);
  EXPECT_TRUE(status.is_valid_velocity_deviation);
  EXPECT_TRUE(status.is_valid_distance_deviation);
  EXPECT_TRUE(status.is_valid_longitudinal_distance_deviation);
  EXPECT_TRUE(status.is_valid_forward_trajectory_length);
  EXPECT_TRUE(status.is_valid_latency);
  EXPECT_TRUE(status.is_valid_yaw_deviation);
  EXPECT_TRUE(status.is_valid_trajectory_shift);
  EXPECT_TRUE(status.is_valid_intersection_collision_check);
  EXPECT_TRUE(status.is_valid_rear_collision_check);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_validator_full_ingress_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
