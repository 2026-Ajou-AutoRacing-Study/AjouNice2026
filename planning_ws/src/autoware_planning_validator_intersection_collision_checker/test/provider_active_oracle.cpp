#include "autoware/planning_validator_intersection_collision_checker/intersection_collision_checker_plugin.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/planning_validator/planning_validator_manager.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>

#include <cstdlib>

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
  int time = 0;
  for (const auto & center : lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = center.x();
    point.pose.position.y = center.y();
    point.pose.position.z = center.z();
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0F;
    point.time_from_start.sec = time++;
    result.points.push_back(point);
  }
  return result;
}

sensor_msgs::PointCloud2 empty_cloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 result;
  pcl::toROSMsg(cloud, result);
  result.header.frame_id = "lidar";
  result.header.stamp = ros::Time(10.0);
  return result;
}

lanelet::Lanelet make_lane(
  lanelet::Id id, const lanelet::Points3d & left, const lanelet::Points3d & right,
  const std::string & turn = "")
{
  lanelet::Lanelet result(
    id, lanelet::LineString3d(id + 100, left), lanelet::LineString3d(id + 200, right));
  result.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  if (!turn.empty()) result.attributes()["turn_direction"] = turn;
  return result;
}

autoware_planning_msgs::Trajectory synthetic_ego_trajectory()
{
  autoware_planning_msgs::Trajectory result;
  for (int x = 0; x <= 20; x += 4) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0F;
    point.time_from_start.sec = x / 2;
    result.points.push_back(point);
  }
  return result;
}

sensor_msgs::PointCloud2 moving_cluster(const float y, const ros::Time & stamp)
{
  sensor_msgs::PointCloud2 result;
  result.header.frame_id = "lidar";
  result.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  // Source processing first retains voxels only at min_size=3, then requires
  // a 10-point Euclidean cluster after voxelization.  Supply thirty adjacent
  // voxels with three samples each; this leaves ample margin at voxel-boundary
  // rounding while still representing one connected moving cluster.
  modifier.resize(90U);
  sensor_msgs::PointCloud2Iterator<float> x(result, "x"), yy(result, "y"), z(result, "z");
  for (int voxel = 0; voxel < 30; ++voxel) {
    const float dx = static_cast<float>(voxel % 6) * 0.21F - 0.525F;
    const float dy = static_cast<float>(voxel / 6) * 0.21F - 0.42F;
    for (int sample = 0; sample < 3; ++sample) {
      const float epsilon = static_cast<float>(sample) * 0.01F;
      *x = 10.0F + dx + epsilon; *yy = y + dy + epsilon; *z = 1.0F; ++x; ++yy; ++z;
    }
  }
  return result;
}

autoware::planning_validator::TrajectoryCheckerInput complete_input(
  const autoware_planning_msgs::Trajectory & trajectory, const nav_msgs::Odometry & odometry,
  const sensor_msgs::PointCloud2 & cloud, const geometry_msgs::AccelWithCovarianceStamped & acceleration,
  const autoware_planning_msgs::LaneletRoute & route, const autoware_map_msgs::LaneletMapBin & lanelet_map,
  const autoware_perception_msgs::TrafficLightGroupArray & signals,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle, tf2_ros::Buffer & tf_buffer,
  const autoware::route_handler::RouteHandler & route_handler)
{
  return {trajectory, &odometry, &cloud, &acceleration, &route, &lanelet_map, &signals,
    &trajectory, &vehicle, &tf_buffer, &route_handler};
}
}  // namespace

TEST(IntersectionCollisionCheckerProviderActive, AcceptsCompleteActiveTurnInput)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_INTERSECTION_FIXTURE_PATH);
  ASSERT_TRUE(map);
  lanelet::ConstLanelet turn;
  for (const auto & lane : map->laneletLayer) {
    if (lane.hasAttribute("turn_direction") &&
      (lane.attribute("turn_direction").value() == "right" ||
       lane.attribute("turn_direction").value() == "left")) {
      turn = lane;
      break;
    }
  }
  ASSERT_NE(turn.id(), lanelet::InvalId);
  const auto trajectory = trajectory_for(turn);
  ASSERT_GE(trajectory.points.size(), 2U);

  autoware::route_handler::RouteHandler route_handler(map);
  const auto route = route_for(turn);
  route_handler.setRoute(route);
  ASSERT_TRUE(route_handler.isHandlerReady());
  nav_msgs::Odometry odometry;
  odometry.pose.pose = trajectory.points.front().pose;
  odometry.twist.twist.linear.x = 10.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware_map_msgs::LaneletMapBin lanelet_map;
  autoware_perception_msgs::TrafficLightGroupArray signals;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.wheel_base_m = 2.0;
  vehicle.front_overhang_m = 1.0;
  vehicle.rear_overhang_m = 1.0;
  vehicle.vehicle_height_m = 2.0;
  auto cloud = empty_cloud();
  tf2_ros::Buffer tf_buffer;
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = cloud.header.stamp;
  transform.header.frame_id = "map";
  transform.child_frame_id = "lidar";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "icc-r004-active-oracle", true));

  ros::NodeHandle private_nh("~");
  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(
    private_nh, "autoware::planning_validator::IntersectionCollisionCheckerRos1"));
  ASSERT_EQ(manager.loaded_plugin_count(), 1U);
  const auto input = complete_input(
    trajectory, odometry, cloud, acceleration, route, lanelet_map, signals, vehicle, tf_buffer, route_handler);
  const auto status = manager.validate(input);
  EXPECT_TRUE(status.is_valid_intersection_collision_check);
}

TEST(IntersectionCollisionCheckerProviderActive, DebouncesAClusterInsideAnActiveCrossingLane)
{
  const auto ego_turn = make_lane(
    1000,
    {lanelet::Point3d(1001, 0.0, 1.0, 0.0), lanelet::Point3d(1002, 20.0, 1.0, 0.0)},
    {lanelet::Point3d(1003, 0.0, -1.0, 0.0), lanelet::Point3d(1004, 20.0, -1.0, 0.0)}, "right");
  const auto crossing = make_lane(
    2000,
    {lanelet::Point3d(2001, 9.0, -10.0, 0.0), lanelet::Point3d(2002, 9.0, 10.0, 0.0)},
    {lanelet::Point3d(2003, 11.0, -10.0, 0.0), lanelet::Point3d(2004, 11.0, 10.0, 0.0)});
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(ego_turn);
  map->add(crossing);
  autoware::route_handler::RouteHandler route_handler(map);
  const auto route = route_for(ego_turn);
  route_handler.setRoute(route);
  ASSERT_TRUE(route_handler.isHandlerReady());
  const auto trajectory = synthetic_ego_trajectory();
  nav_msgs::Odometry odometry;
  odometry.pose.pose = trajectory.points.front().pose;
  odometry.twist.twist.linear.x = 10.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware_map_msgs::LaneletMapBin lanelet_map;
  autoware_perception_msgs::TrafficLightGroupArray signals;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.wheel_base_m = 2.0;
  vehicle.front_overhang_m = 1.0;
  vehicle.rear_overhang_m = 1.0;
  vehicle.vehicle_height_m = 2.0;
  tf2_ros::Buffer tf_buffer;
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "lidar";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "icc-r004-cluster-oracle", true));
  ros::NodeHandle private_nh("~");
  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(
    private_nh, "autoware::planning_validator::IntersectionCollisionCheckerRos1"));

  // The source off-buffer starts at zero, so use t=2 for the initial safe
  // observation rather than treating its equality boundary as a pass.
  ros::Time::setNow(ros::Time(2.0));
  const auto first_cloud = moving_cluster(-3.0F, ros::Time(2.0));
  EXPECT_TRUE(manager.validate(complete_input(
    trajectory, odometry, first_cloud, acceleration, route, lanelet_map, signals, vehicle, tf_buffer,
    route_handler)).is_valid_intersection_collision_check);
  ros::Time::setNow(ros::Time(3.0));
  const auto second_cloud = moving_cluster(-2.0F, ros::Time(3.0));
  EXPECT_TRUE(manager.validate(complete_input(
    trajectory, odometry, second_cloud, acceleration, route, lanelet_map, signals, vehicle, tf_buffer,
    route_handler)).is_valid_intersection_collision_check);
  ros::Time::setNow(ros::Time(4.0));
  const auto third_cloud = moving_cluster(-1.0F, ros::Time(4.0));
  EXPECT_FALSE(manager.validate(complete_input(
    trajectory, odometry, third_cloud, acceleration, route, lanelet_map, signals, vehicle, tf_buffer,
    route_handler)).is_valid_intersection_collision_check);
  ros::Time::setNow(ros::Time());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "icc_r004_provider_active_oracle");
  if (std::system(
        "rosparam load \"$(rospack find autoware_planning_validator_intersection_collision_checker)/"
        "config/ros1/intersection_collision_checker.yaml\" /icc_r004_provider_active_oracle") != 0) {
    return 1;
  }
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
