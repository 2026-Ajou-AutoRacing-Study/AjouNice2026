#include "autoware/behavior_velocity_planner/node.hpp"
#include "autoware/behavior_velocity_planner/experimental/node.hpp"
#include "autoware/behavior_velocity_planner/pointcloud_transform.hpp"

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <lanelet2_core/LaneletMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>
#include <string>
#include <vector>

namespace
{
class Receiver
{
public:
  void callback(const autoware_planning_msgs::Path::ConstPtr & message) { message_ = message; }
  autoware_planning_msgs::Path::ConstPtr message_;
};

void setVehicleInfoParameters(ros::NodeHandle & node)
{
  const std::string prefix = "/test_behavior_velocity_planner_node/";
  node.setParam(prefix + "wheel_radius", 0.5);
  node.setParam(prefix + "wheel_width", 0.3);
  node.setParam(prefix + "wheel_base", 2.7);
  node.setParam(prefix + "wheel_tread", 1.6);
  node.setParam(prefix + "front_overhang", 1.0);
  node.setParam(prefix + "rear_overhang", 1.1);
  node.setParam(prefix + "left_overhang", 0.2);
  node.setParam(prefix + "right_overhang", 0.2);
  node.setParam(prefix + "vehicle_height", 1.8);
  node.setParam(prefix + "max_steer_angle", 0.6);
  node.setParam(prefix + "max_accel", 1.2);
  node.setParam(prefix + "max_jerk", 0.3);
  node.setParam(prefix + "system_delay", 0.4);
  node.setParam(prefix + "delay_response_time", 0.5);
}

TEST(BehaviorVelocityPlannerNode, PublishesManagerOutputOnTheRos1PrivateTopics)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto input_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);

  for (int i = 0; i < 100 && (input_pub.getNumSubscribers() == 0 || output_sub.getNumPublishers() == 0);
       ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(input_pub.getNumSubscribers(), 0U);
  ASSERT_GT(output_sub.getNumPublishers(), 0U);

  autoware_internal_planning_msgs::PathWithLaneId input;
  input.header.frame_id = "map";
  input.header.stamp = ros::Time(42, 0);
  input.points.resize(2);
  input.points.at(0).point.pose.position.x = 1.0;
  input.points.at(0).lane_ids.push_back(10);
  input.points.at(1).point.pose.position.x = 2.0;
  input.points.at(1).lane_ids.push_back(11);
  input_pub.publish(input);

  for (int i = 0; i < 100 && !receiver.message_; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_TRUE(receiver.message_);
  EXPECT_EQ(receiver.message_->header.frame_id, "map");
  EXPECT_EQ(receiver.message_->header.stamp, ros::Time(42, 0));
  ASSERT_EQ(receiver.message_->points.size(), 2U);
  EXPECT_DOUBLE_EQ(receiver.message_->points.at(0).pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(receiver.message_->points.at(1).pose.position.x, 2.0);
}

TEST(BehaviorVelocityPlannerNode, ExperimentalNodeSuppressesOutputUntilRequiredStateIsReady)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto input_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);
  for (int i = 0; i < 100 && (input_pub.getNumSubscribers() == 0 || output_sub.getNumPublishers() == 0);
       ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(input_pub.getNumSubscribers(), 0U);
  autoware_internal_planning_msgs::PathWithLaneId input;
  input.header.frame_id = "map";
  input.header.stamp = ros::Time(43, 0);
  input.points.resize(2);
  input.points.at(0).point.pose.orientation.w = 1.0;
  input.points.at(1).point.pose.position.x = 2.0;
  input.points.at(1).point.pose.orientation.w = 1.0;
  input.points.at(0).lane_ids = {10};
  input.points.at(1).lane_ids = {11};
  input_pub.publish(input);
  for (int i = 0; i < 100 && !receiver.message_; ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  EXPECT_FALSE(receiver.message_);
}

TEST(BehaviorVelocityPlannerNode, ExperimentalNodePublishesOnlyAfterMapOdometryAndAcceleration)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/test_behavior_velocity_planner_node/input/vector_map", 1, true);
  const auto odometry_pub = node.advertise<nav_msgs::Odometry>(
    "/test_behavior_velocity_planner_node/input/vehicle_odometry", 1, true);
  const auto acceleration_pub = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/test_behavior_velocity_planner_node/input/accel", 1, true);
  const auto path_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);
  for (int i = 0; i < 100 &&
       (map_pub.getNumSubscribers() == 0 || odometry_pub.getNumSubscribers() == 0 ||
        acceleration_pub.getNumSubscribers() == 0 || path_pub.getNumSubscribers() == 0);
       ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_BVP_TRANSPORT_TEST_MAP);
  ASSERT_TRUE(map);
  map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry_pub.publish(odometry);
  acceleration_pub.publish(geometry_msgs::AccelWithCovarianceStamped{});
  for (int i = 0; i < 30; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  autoware_internal_planning_msgs::PathWithLaneId input;
  input.header.frame_id = "map";
  input.header.stamp = ros::Time(44, 0);
  input.points.resize(2);
  input.points.at(0).point.pose.orientation.w = 1.0;
  input.points.at(1).point.pose.position.x = 2.0;
  input.points.at(1).point.pose.orientation.w = 1.0;
  input.points.at(0).lane_ids = {34675};
  input.points.at(1).lane_ids = {34675};
  path_pub.publish(input);
  for (int i = 0; i < 100 && !receiver.message_; ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_TRUE(receiver.message_);
  EXPECT_EQ(receiver.message_->header.stamp, ros::Time(44, 0));
  ASSERT_EQ(receiver.message_->points.size(), 2U);
  EXPECT_DOUBLE_EQ(receiver.message_->points.at(1).pose.position.x, 2.0);
}

TEST(BehaviorVelocityPlannerNode, ExperimentalNodeLoadsNoDrivableLanePluginAndPublishesAfterReadiness)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  node.setParam(
    "/test_behavior_velocity_planner_node/launch_modules",
    std::vector<std::string>{"autoware::behavior_velocity_planner::NoDrivableLaneModulePlugin"});
  autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/test_behavior_velocity_planner_node/input/vector_map", 1, true);
  const auto odometry_pub = node.advertise<nav_msgs::Odometry>(
    "/test_behavior_velocity_planner_node/input/vehicle_odometry", 1, true);
  const auto acceleration_pub = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/test_behavior_velocity_planner_node/input/accel", 1, true);
  const auto path_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);
  for (int i = 0; i < 100 &&
       (map_pub.getNumSubscribers() == 0 || odometry_pub.getNumSubscribers() == 0 ||
        acceleration_pub.getNumSubscribers() == 0 || path_pub.getNumSubscribers() == 0);
       ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_BVP_TRANSPORT_TEST_MAP);
  ASSERT_TRUE(map);
  map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry_pub.publish(odometry);
  acceleration_pub.publish(geometry_msgs::AccelWithCovarianceStamped{});
  for (int i = 0; i < 30; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  autoware_internal_planning_msgs::PathWithLaneId input;
  input.header.frame_id = "map";
  input.header.stamp = ros::Time(46, 0);
  input.points.resize(2);
  input.points.at(0).point.pose.orientation.w = 1.0;
  input.points.at(1).point.pose.position.x = 2.0;
  input.points.at(1).point.pose.orientation.w = 1.0;
  input.points.at(0).lane_ids = {34675};
  input.points.at(1).lane_ids = {34675};
  path_pub.publish(input);
  for (int i = 0; i < 100 && !receiver.message_; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_TRUE(receiver.message_);
  EXPECT_EQ(receiver.message_->header.stamp, ros::Time(46, 0));
  ASSERT_EQ(receiver.message_->points.size(), 2U);
  node.deleteParam("/test_behavior_velocity_planner_node/launch_modules");
}

TEST(BehaviorVelocityPlannerNode, ExperimentalDetectionAreaRequiresObjectsAndTransformedPointcloud)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  node.setParam(
    "/test_behavior_velocity_planner_node/launch_modules",
    std::vector<std::string>{"autoware::behavior_velocity_planner::experimental::DetectionAreaModulePlugin"});
  autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>("/test_behavior_velocity_planner_node/input/vector_map", 1, true);
  const auto odometry_pub = node.advertise<nav_msgs::Odometry>("/test_behavior_velocity_planner_node/input/vehicle_odometry", 1, true);
  const auto acceleration_pub = node.advertise<geometry_msgs::AccelWithCovarianceStamped>("/test_behavior_velocity_planner_node/input/accel", 1, true);
  const auto objects_pub = node.advertise<autoware_perception_msgs::PredictedObjects>("/test_behavior_velocity_planner_node/input/dynamic_objects", 1, true);
  const auto cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/test_behavior_velocity_planner_node/input/no_ground_pointcloud", 1, true);
  const auto path_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>("/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now(); tf.header.frame_id = "map"; tf.child_frame_id = "sensor"; tf.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(tf);
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_BVP_TRANSPORT_TEST_MAP);
  ASSERT_TRUE(map);
  map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  nav_msgs::Odometry odometry; odometry.pose.pose.orientation.w = 1.0; odometry_pub.publish(odometry);
  acceleration_pub.publish(geometry_msgs::AccelWithCovarianceStamped{});
  objects_pub.publish(autoware_perception_msgs::PredictedObjects{});
  pcl::PointCloud<pcl::PointXYZ> cloud; cloud.push_back(pcl::PointXYZ{1.0F, 0.0F, 0.0F});
  sensor_msgs::PointCloud2 cloud_message; pcl::toROSMsg(cloud, cloud_message); cloud_message.header.frame_id = "sensor";
  cloud_pub.publish(cloud_message);
  for (int i = 0; i < 40; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  autoware_internal_planning_msgs::PathWithLaneId input;
  input.header.frame_id = "map"; input.header.stamp = ros::Time(45, 0); input.points.resize(2);
  input.points.at(0).point.pose.orientation.w = 1.0; input.points.at(1).point.pose.position.x = 2.0;
  input.points.at(1).point.pose.orientation.w = 1.0; input.points.at(0).lane_ids = {34675}; input.points.at(1).lane_ids = {34675};
  path_pub.publish(input);
  for (int i = 0; i < 100 && !receiver.message_; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  ASSERT_TRUE(receiver.message_);
  EXPECT_EQ(receiver.message_->header.stamp, ros::Time(45, 0));
  node.deleteParam("/test_behavior_velocity_planner_node/launch_modules");
}

TEST(BehaviorVelocityPlannerNode, ExperimentalNoStoppingAreaRequiresObjectsAndPublishesAfterReadiness)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  node.setParam(
    "/test_behavior_velocity_planner_node/launch_modules",
    std::vector<std::string>{"autoware::behavior_velocity_planner::experimental::NoStoppingAreaModulePlugin"});
  autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/test_behavior_velocity_planner_node/input/vector_map", 1, true);
  const auto odometry_pub = node.advertise<nav_msgs::Odometry>(
    "/test_behavior_velocity_planner_node/input/vehicle_odometry", 1, true);
  const auto acceleration_pub = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/test_behavior_velocity_planner_node/input/accel", 1, true);
  const auto objects_pub = node.advertise<autoware_perception_msgs::PredictedObjects>(
    "/test_behavior_velocity_planner_node/input/dynamic_objects", 1, true);
  const auto path_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);
  for (int i = 0; i < 100 &&
       (map_pub.getNumSubscribers() == 0 || odometry_pub.getNumSubscribers() == 0 ||
        acceleration_pub.getNumSubscribers() == 0 || objects_pub.getNumSubscribers() == 0 ||
        path_pub.getNumSubscribers() == 0);
       ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_BVP_TRANSPORT_TEST_MAP);
  ASSERT_TRUE(map);
  map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry_pub.publish(odometry);
  acceleration_pub.publish(geometry_msgs::AccelWithCovarianceStamped{});
  objects_pub.publish(autoware_perception_msgs::PredictedObjects{});
  for (int i = 0; i < 30; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  autoware_internal_planning_msgs::PathWithLaneId input;
  input.header.frame_id = "map";
  input.header.stamp = ros::Time(47, 0);
  input.points.resize(2);
  input.points.at(0).point.pose.orientation.w = 1.0;
  input.points.at(1).point.pose.position.x = 2.0;
  input.points.at(1).point.pose.orientation.w = 1.0;
  input.points.at(0).lane_ids = {34675};
  input.points.at(1).lane_ids = {34675};
  path_pub.publish(input);
  for (int i = 0; i < 100 && !receiver.message_; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_TRUE(receiver.message_);
  EXPECT_EQ(receiver.message_->header.stamp, ros::Time(47, 0));
  ASSERT_EQ(receiver.message_->points.size(), 2U);
  node.deleteParam("/test_behavior_velocity_planner_node/launch_modules");
}

TEST(BehaviorVelocityPlannerNode, ConvertsNoGroundPointCloudWithTheTfRigidTransform)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  input.push_back(pcl::PointXYZ{1.0F, 0.0F, 0.0F});
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(input, message);

  geometry_msgs::TransformStamped transform;
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = 3.0;
  transform.transform.rotation.w = std::sqrt(0.5);
  transform.transform.rotation.z = std::sqrt(0.5);

  const auto output =
    autoware::behavior_velocity_planner::transformNoGroundPointCloud(message, transform);
  ASSERT_EQ(output->size(), 1U);
  EXPECT_NEAR(output->at(0).x, 2.0, 1e-5);
  EXPECT_NEAR(output->at(0).y, 4.0, 1e-5);
  EXPECT_NEAR(output->at(0).z, 0.0, 1e-5);
}

TEST(BehaviorVelocityPlannerNode, RegistersAllPlannerDataProviderTransports)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode planner(node);
  const auto acceleration_pub = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/test_behavior_velocity_planner_node/input/accel", 1);
  const auto objects_pub = node.advertise<autoware_perception_msgs::PredictedObjects>(
    "/test_behavior_velocity_planner_node/input/dynamic_objects", 1);
  const auto occupancy_pub = node.advertise<nav_msgs::OccupancyGrid>(
    "/test_behavior_velocity_planner_node/input/occupancy_grid", 1);
  const auto velocity_limit_pub = node.advertise<autoware_internal_planning_msgs::VelocityLimit>(
    "/test_behavior_velocity_planner_node/input/external_velocity_limit_mps", 1);

  for (int i = 0; i < 100 &&
       (acceleration_pub.getNumSubscribers() == 0 || objects_pub.getNumSubscribers() == 0 ||
        occupancy_pub.getNumSubscribers() == 0 || velocity_limit_pub.getNumSubscribers() == 0);
       ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_GT(acceleration_pub.getNumSubscribers(), 0U);
  EXPECT_GT(objects_pub.getNumSubscribers(), 0U);
  EXPECT_GT(occupancy_pub.getNumSubscribers(), 0U);
  EXPECT_GT(velocity_limit_pub.getNumSubscribers(), 0U);

  acceleration_pub.publish(geometry_msgs::AccelWithCovarianceStamped{});
  objects_pub.publish(autoware_perception_msgs::PredictedObjects{});
  occupancy_pub.publish(nav_msgs::OccupancyGrid{});
  velocity_limit_pub.publish(autoware_internal_planning_msgs::VelocityLimit{});
  ros::spinOnce();
}

TEST(BehaviorVelocityPlannerNode, LoadsInstalledStopLinePluginFromLaunchModules)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  node.setParam(
    "/test_behavior_velocity_planner_node/launch_modules",
    std::vector<std::string>{"autoware::behavior_velocity_planner::StopLineModulePlugin"});

  EXPECT_NO_THROW({
    autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode planner(node);
    ros::spinOnce();
  });
  node.deleteParam("/test_behavior_velocity_planner_node/launch_modules");
}

TEST(BehaviorVelocityPlannerNode, AcceptsConvertedStopSignLaneletMapProvider)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode planner(node);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/test_behavior_velocity_planner_node/input/vector_map", 1, true);
  for (int i = 0; i < 100 && map_pub.getNumSubscribers() == 0; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(map_pub.getNumSubscribers(), 0U);
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_BVP_TRANSPORT_TEST_MAP);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  for (int i = 0; i < 20; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
}

TEST(BehaviorVelocityPlannerNode, ExperimentalStopLineAdapterAppliesToMappedLaneletPath)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  node.setParam(
    "/test_behavior_velocity_planner_node/launch_modules",
    std::vector<std::string>{
      "autoware::behavior_velocity_planner::experimental::StopLineModulePlugin"});
  autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/test_behavior_velocity_planner_node/input/vector_map", 1, true);
  const auto odometry_pub = node.advertise<nav_msgs::Odometry>(
    "/test_behavior_velocity_planner_node/input/vehicle_odometry", 1, true);
  const auto acceleration_pub = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/test_behavior_velocity_planner_node/input/accel", 1, true);
  const auto path_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);
  for (int i = 0; i < 100 &&
       (map_pub.getNumSubscribers() == 0 || odometry_pub.getNumSubscribers() == 0 ||
        acceleration_pub.getNumSubscribers() == 0 || path_pub.getNumSubscribers() == 0 ||
        output_sub.getNumPublishers() == 0); ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_BVP_TRANSPORT_TEST_MAP);
  ASSERT_TRUE(map);
  const auto lanelet = map->laneletLayer.get(34675);
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.header.frame_id = "map";
  for (const auto & p : lanelet.centerline()) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = p.x(); point.point.pose.position.y = p.y(); point.point.pose.position.z = p.z();
    point.point.pose.orientation.w = 1.0; point.point.longitudinal_velocity_mps = 8.0F;
    point.lane_ids = {34675}; path.points.push_back(point);
  }
  for (const auto & p : lanelet.leftBound()) { geometry_msgs::Point q; q.x = p.x(); q.y = p.y(); q.z = p.z(); path.left_bound.push_back(q); }
  for (const auto & p : lanelet.rightBound()) { geometry_msgs::Point q; q.x = p.x(); q.y = p.y(); q.z = p.z(); path.right_bound.push_back(q); }
  nav_msgs::Odometry odometry;
  odometry.pose.pose = path.points.front().point.pose;
  map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  odometry_pub.publish(odometry);
  acceleration_pub.publish(geometry_msgs::AccelWithCovarianceStamped{});
  for (int i = 0; i < 30; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  path_pub.publish(path);
  for (int i = 0; i < 100 && !receiver.message_; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  ASSERT_TRUE(receiver.message_);
  ASSERT_FALSE(receiver.message_->points.empty());
  EXPECT_EQ(receiver.message_->points.back().longitudinal_velocity_mps, 0.0F);
  node.deleteParam("/test_behavior_velocity_planner_node/launch_modules");
}

TEST(BehaviorVelocityPlannerNode, AppliesStopLinePluginToTheMappedLaneletPath)
{
  ros::NodeHandle node;
  setVehicleInfoParameters(node);
  node.setParam(
    "/test_behavior_velocity_planner_node/launch_modules",
    std::vector<std::string>{"autoware::behavior_velocity_planner::StopLineModulePlugin"});
  autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode planner(node);
  Receiver receiver;
  const auto output_sub = node.subscribe(
    "/test_behavior_velocity_planner_node/output/path", 1, &Receiver::callback, &receiver);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/test_behavior_velocity_planner_node/input/vector_map", 1, true);
  const auto odometry_pub = node.advertise<nav_msgs::Odometry>(
    "/test_behavior_velocity_planner_node/input/vehicle_odometry", 1, true);
  const auto path_pub = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/test_behavior_velocity_planner_node/input/path_with_lane_id", 1);
  for (int i = 0; i < 100 &&
       (map_pub.getNumSubscribers() == 0 || odometry_pub.getNumSubscribers() == 0 ||
        path_pub.getNumSubscribers() == 0 || output_sub.getNumPublishers() == 0); ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(map_pub.getNumSubscribers(), 0U);
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_BVP_TRANSPORT_TEST_MAP);
  ASSERT_TRUE(map);
  const auto lanelet = map->laneletLayer.get(34675);
  const auto centerline = lanelet.centerline();
  ASSERT_GT(centerline.size(), 1U);
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.header.frame_id = "map";
  for (const auto & p : centerline) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = p.x(); point.point.pose.position.y = p.y(); point.point.pose.position.z = p.z();
    point.point.pose.orientation.w = 1.0; point.point.longitudinal_velocity_mps = 8.0F; point.lane_ids = {34675};
    path.points.push_back(point);
  }
  for (const auto & p : lanelet.leftBound()) { geometry_msgs::Point q; q.x=p.x(); q.y=p.y(); q.z=p.z(); path.left_bound.push_back(q); }
  for (const auto & p : lanelet.rightBound()) { geometry_msgs::Point q; q.x=p.x(); q.y=p.y(); q.z=p.z(); path.right_bound.push_back(q); }
  nav_msgs::Odometry odometry;
  odometry.pose.pose = path.points.front().point.pose;
  odometry.twist.twist.linear.x = 0.0;
  map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  odometry_pub.publish(odometry);
  for (int i = 0; i < 20; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  path_pub.publish(path);
  for (int i = 0; i < 100 && !receiver.message_; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  ASSERT_TRUE(receiver.message_);
  ASSERT_FALSE(receiver.message_->points.empty());
  EXPECT_EQ(receiver.message_->points.back().longitudinal_velocity_mps, 0.0F);
  node.deleteParam("/test_behavior_velocity_planner_node/launch_modules");
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_behavior_velocity_planner_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
