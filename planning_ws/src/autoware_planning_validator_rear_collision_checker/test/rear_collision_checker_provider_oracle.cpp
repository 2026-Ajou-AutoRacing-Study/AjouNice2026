#include "autoware/planning_validator_rear_collision_checker/rear_collision_checker_plugin.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/planning_validator/rear_collision_decision_core.hpp>
#include <autoware/planning_validator/rear_lanelet_detection_core.hpp>
#include <autoware/planning_validator/rear_object_assembly_core.hpp>
#include <autoware/planning_validator/rear_pointcloud_core.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2_ros/buffer.h>

#include <array>
#include <limits>
#include <Eigen/Geometry>

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
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & point : lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = point.x();
    trajectory_point.pose.position.y = point.y();
    trajectory_point.pose.position.z = point.z();
    trajectory_point.pose.orientation.w = 1.0;
    trajectory.points.push_back(trajectory_point);
  }
  return trajectory;
}

void set_adjacent_lane_cluster(sensor_msgs::PointCloud2 & pointcloud, const ros::Time & stamp)
{
  pointcloud.header.frame_id = "lidar";
  pointcloud.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(pointcloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(6U);
  sensor_msgs::PointCloud2Iterator<float> x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(pointcloud, "z");
  for (const auto & point : {std::array<float, 3>{0.40F, 2.90F, 0.80F},
                             {0.50F, 2.90F, 0.80F}, {0.60F, 2.90F, 0.80F},
                             {0.40F, 3.00F, 0.80F}, {0.50F, 3.00F, 0.80F},
                             {0.60F, 3.00F, 0.80F}}) {
    *x = point[0]; *y = point[1]; *z = point[2];
    ++x; ++y; ++z;
  }
}
}  // namespace

TEST(RearCollisionCheckerProvider, IsInstalledButFailsClosedBeforeFullSourceRender)
{
  ros::NodeHandle private_nh("~rear_collision_checker_provider_oracle");
  autoware::planning_validator::RearCollisionCheckerRos1 checker;
  checker.initialize(private_nh, "autoware::planning_validator::RearCollisionCheckerRos1");
  EXPECT_EQ(checker.module_name(), "autoware::planning_validator::RearCollisionCheckerRos1");

  autoware_planning_msgs::Trajectory trajectory;
  autoware_planning_validator::PlanningValidatorStatus status;
  status.is_valid_rear_collision_check = true;
  checker.validate({trajectory}, status);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
}

TEST(RearCollisionCheckerProvider, AcceptsCompleteInactiveSourceBranchWithoutTf)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  const auto route = route_for(lane);
  const auto trajectory = trajectory_for(lane);
  ASSERT_GE(trajectory.points.size(), 2U);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  route_handler->setRoute(route);
  ASSERT_TRUE(route_handler->isHandlerReady());

  nav_msgs::Odometry odometry;
  const auto center = lane.centerline()[lane.centerline().size() / 2U];
  odometry.pose.pose.position.x = center.x();
  odometry.pose.pose.position.y = center.y();
  odometry.pose.pose.position.z = center.z();
  odometry.pose.pose.orientation.w = 1.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  sensor_msgs::PointCloud2 pointcloud;
  pointcloud.header.frame_id = "unavailable_lidar";
  autoware_map_msgs::LaneletMapBin lanelet_map;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  vehicle.vehicle_height_m = 2.0;
  tf2_ros::Buffer tf_buffer;
  tf_buffer.setUsingDedicatedThread(true);

  ros::NodeHandle private_nh("~rear_collision_checker_complete_inactive_oracle");
  autoware::planning_validator::RearCollisionCheckerRos1 checker;
  checker.initialize(private_nh, "autoware::planning_validator::RearCollisionCheckerRos1");
  autoware_planning_validator::PlanningValidatorStatus status;
  checker.validate(
    {trajectory, &odometry, &pointcloud, &acceleration, &route, &lanelet_map, nullptr,
      &trajectory, &vehicle, &tf_buffer, route_handler.get()},
    status);
  EXPECT_TRUE(status.is_valid_rear_collision_check);
}

TEST(RearCollisionCheckerProvider, ShortFiniteTerminalPathIsSourceInactiveButMalformedInputsFail)
{
  lanelet::Points3d left{lanelet::Point3d(301, 0, 2, 0), lanelet::Point3d(302, 20, 2, 0)};
  lanelet::Points3d right{lanelet::Point3d(303, 0, -2, 0), lanelet::Point3d(304, 20, -2, 0)};
  lanelet::Lanelet lane(300, lanelet::LineString3d(305, left), lanelet::LineString3d(306, right));
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  const auto route = route_for(map->laneletLayer.get(lane.id()));
  autoware::route_handler::RouteHandler handler(map);
  handler.setRoute(route);
  ASSERT_TRUE(handler.isHandlerReady());
  autoware_planning_msgs::Trajectory trajectory;
  for (int i = 0; i <= 2; ++i) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = i;
    point.pose.orientation.w = 1;
    point.longitudinal_velocity_mps = i == 2 ? 0 : 0.25;
    trajectory.points.push_back(point);
  }
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  sensor_msgs::PointCloud2 pointcloud;
  pointcloud.header.frame_id = "unavailable_lidar";
  autoware_map_msgs::LaneletMapBin map_message;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 1.896;
  vehicle.vehicle_height_m = 2.5;
  vehicle.max_longitudinal_offset_m = 3.79;
  tf2_ros::Buffer buffer;
  ros::NodeHandle node("~rear_short_path_source_inactive");
  autoware::planning_validator::RearCollisionCheckerRos1 checker;
  checker.initialize(node, "autoware::planning_validator::RearCollisionCheckerRos1");
  autoware::planning_validator::TrajectoryCheckerInput input{
    trajectory, &odometry, &pointcloud, &acceleration, &route, &map_message, nullptr,
    &trajectory, &vehicle, &buffer, &handler};
  autoware_planning_validator::PlanningValidatorStatus status;
  checker.validate(input, status);
  EXPECT_TRUE(status.is_valid_rear_collision_check);
  trajectory.points.back().pose.position.x = std::numeric_limits<double>::quiet_NaN();
  checker.validate(input, status);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
  trajectory.points.back().pose.position.x = 2;
  input.obstacle_pointcloud = nullptr;
  checker.validate(input, status);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
  input.obstacle_pointcloud = &pointcloud;
  trajectory.points.resize(1);
  checker.validate(input, status);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
}

TEST(RearCollisionCheckerProvider, FailsClosedForActiveShiftWithoutPointcloudTransform)
{
  lanelet::Points3d left{lanelet::Point3d(101, 0.0, 2.0, 0.0),
    lanelet::Point3d(102, 20.0, 2.0, 0.0)};
  lanelet::Points3d right{lanelet::Point3d(103, 0.0, -2.0, 0.0),
    lanelet::Point3d(104, 20.0, -2.0, 0.0)};
  lanelet::Lanelet lane(
    100, lanelet::LineString3d(105, left), lanelet::LineString3d(106, right));
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  const lanelet::ConstLanelet route_lane = map->laneletLayer.get(lane.id());
  const auto route = route_for(route_lane);
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & point_xy : {std::pair{0.0, 0.0}, std::pair{10.0, 3.0}}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = point_xy.first;
    point.pose.position.y = point_xy.second;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    trajectory.points.push_back(point);
  }
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  route_handler->setRoute(route);
  ASSERT_TRUE(route_handler->isHandlerReady());

  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 2.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  sensor_msgs::PointCloud2 pointcloud;
  pointcloud.header.frame_id = "unavailable_lidar";
  autoware_map_msgs::LaneletMapBin lanelet_map;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  vehicle.vehicle_height_m = 2.0;
  tf2_ros::Buffer tf_buffer;
  tf_buffer.setUsingDedicatedThread(true);

  ros::NodeHandle private_nh("~rear_collision_checker_active_no_tf_oracle");
  autoware::planning_validator::RearCollisionCheckerRos1 checker;
  checker.initialize(private_nh, "autoware::planning_validator::RearCollisionCheckerRos1");
  autoware_planning_validator::PlanningValidatorStatus status;
  status.is_valid_rear_collision_check = true;
  checker.validate(
    {trajectory, &odometry, &pointcloud, &acceleration, &route, &lanelet_map, nullptr,
      &trajectory, &vehicle, &tf_buffer, route_handler.get()},
    status);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
}

TEST(RearCollisionCheckerProvider, TracksAdjacentClusterBeforeDeclaringUnsafeShift)
{
  lanelet::Points3d left{lanelet::Point3d(201, 0.0, 2.0, 0.0),
    lanelet::Point3d(202, 20.0, 2.0, 0.0)};
  lanelet::Points3d right{lanelet::Point3d(203, 0.0, -2.0, 0.0),
    lanelet::Point3d(204, 20.0, -2.0, 0.0)};
  lanelet::Points3d outer_left{lanelet::Point3d(207, 0.0, 6.0, 0.0),
    lanelet::Point3d(208, 20.0, 6.0, 0.0)};
  lanelet::LineString3d shared_boundary(205, left);
  shared_boundary.attributes()["type"] = "line_thin";
  shared_boundary.attributes()["subtype"] = "dashed";
  lanelet::Lanelet lane(200, shared_boundary, lanelet::LineString3d(206, right));
  lane.attributes()["subtype"] = "road";
  lane.attributes()["type"] = "lanelet";
  lane.attributes()["location"] = "urban";
  lane.attributes()["one_way"] = "yes";
  // The adjacent lane shares the current lane's left bound exactly, so this
  // exercises RouteHandler's real routing-graph adjacency rather than a
  // hand-injected detection lane.
  lanelet::Lanelet adjacent_left_lane(209, lanelet::LineString3d(210, outer_left), shared_boundary);
  adjacent_left_lane.attributes()["subtype"] = "road";
  adjacent_left_lane.attributes()["type"] = "lanelet";
  adjacent_left_lane.attributes()["location"] = "urban";
  adjacent_left_lane.attributes()["one_way"] = "yes";
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  map->add(adjacent_left_lane);
  const lanelet::ConstLanelet route_lane = map->laneletLayer.get(lane.id());
  const auto route = route_for(route_lane);
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & point_xy : {std::pair{0.0, 0.0}, std::pair{10.0, 3.0}}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = point_xy.first;
    point.pose.position.y = point_xy.second;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    trajectory.points.push_back(point);
  }
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  route_handler->setRoute(route);
  ASSERT_TRUE(route_handler->isHandlerReady());
  const auto adjacent = route_handler->getLeftLanelet(route_lane, true, true);
  ASSERT_TRUE(adjacent);
  EXPECT_EQ(adjacent->id(), adjacent_left_lane.id());

  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 2.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  sensor_msgs::PointCloud2 pointcloud;
  pointcloud.header.frame_id = "lidar";
  pointcloud.header.stamp = ros::Time(10.0);
  autoware_map_msgs::LaneletMapBin lanelet_map;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  vehicle.vehicle_height_m = 2.0;
  tf2_ros::Buffer tf_buffer;
  tf_buffer.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "lidar";
  transform.header.stamp = pointcloud.header.stamp;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "rear_collision_checker_provider_oracle", true));

  ros::NodeHandle private_nh("~rear_collision_checker_active_empty_oracle");
  autoware::planning_validator::RearCollisionCheckerRos1 checker;
  checker.initialize(private_nh, "autoware::planning_validator::RearCollisionCheckerRos1");
  autoware_planning_validator::PlanningValidatorStatus status;
  checker.validate(
    {trajectory, &odometry, &pointcloud, &acceleration, &route, &lanelet_map, nullptr,
      &trajectory, &vehicle, &tf_buffer, route_handler.get()},
    status);
  EXPECT_TRUE(status.is_valid_rear_collision_check);

  // A first source-shaped observation remains safe until tracking matures.
  set_adjacent_lane_cluster(pointcloud, ros::Time(10.0));
  const autoware::planning_validator::RearPointCloudCoreParameters pointcloud_parameters;
  const auto filtered = autoware::planning_validator::preprocess_rear_pointcloud(
    pointcloud, pointcloud_parameters, vehicle.vehicle_height_m, Eigen::Affine3f::Identity());
  const auto clustered = autoware::planning_validator::cluster_rear_pointcloud(
    filtered, pointcloud_parameters, odometry.pose.pose.position.z);
  ASSERT_FALSE(clustered->empty());
  autoware::planning_validator::RearRangeInput range_input;
  range_input.range_buffer = 1.0;
  range_input.ego_velocity = odometry.twist.twist.linear.x;
  range_input.ego_max_deceleration = -4.0;
  range_input.ego_min_velocity = 1.38;
  range_input.participant_reaction_time = 1.2;
  range_input.participant_max_deceleration = -2.0;
  range_input.participant_max_velocity = 16.6;
  const auto range = autoware::planning_validator::rear_range_for_rss(range_input);
  const lanelet::ConstLanelets current_lanes{route_lane};
  const lanelet::ConstLanelets target_lanes{*adjacent};
  const auto areas = autoware::planning_validator::make_rear_previous_detection_areas(
    current_lanes, target_lanes, 20.0 - range.forward,
    20.0 - range.forward + range.forward + range.backward, route_handler.get(), -0.5, -0.5);
  ASSERT_FALSE(areas.empty());
  ASSERT_TRUE(autoware::planning_validator::assemble_rear_object_from_detection_areas(
    *clustered, areas, route_handler.get()));
  checker.validate(
    {trajectory, &odometry, &pointcloud, &acceleration, &route, &lanelet_map, nullptr,
      &trajectory, &vehicle, &tf_buffer, route_handler.get()},
    status);
  EXPECT_TRUE(status.is_valid_rear_collision_check);

  // The same detected object must satisfy both the 0.3 s observation interval
  // and the source 0.5 s moving-time filter before RSS can mark it unsafe.
  // The on-time debounce is measured on ROS wall time, not cloud timestamp.
  ros::WallDuration(0.55).sleep();
  pointcloud.header.stamp = ros::Time(10.6);
  checker.validate(
    {trajectory, &odometry, &pointcloud, &acceleration, &route, &lanelet_map, nullptr,
      &trajectory, &vehicle, &tf_buffer, route_handler.get()},
    status);
  // Low-pass tracking produces exactly 1.0 m/s at the second observation;
  // the source uses a strict greater-than filter, so it is still ignored.
  EXPECT_TRUE(status.is_valid_rear_collision_check);

  ros::WallDuration(0.55).sleep();
  pointcloud.header.stamp = ros::Time(11.2);
  checker.validate(
    {trajectory, &odometry, &pointcloud, &acceleration, &route, &lanelet_map, nullptr,
      &trajectory, &vehicle, &tf_buffer, route_handler.get()},
    status);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rear_collision_checker_provider_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
