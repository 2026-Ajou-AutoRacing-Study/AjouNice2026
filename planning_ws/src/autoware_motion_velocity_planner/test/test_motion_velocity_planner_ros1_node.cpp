#include <autoware/motion_velocity_planner/motion_velocity_planner_ros1_node.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <pcl_conversions/pcl_conversions.h>

namespace
{
TEST(MotionVelocityPlannerRos1Node, PreservesTrajectoryThroughEmptyOrderedResultSequence)
{
  ros::NodeHandle node;
  ros::NodeHandle planner_node("/mvp_transport_oracle");
  autoware::motion_velocity_planner::MotionVelocityPlannerRos1Node planner(planner_node);
  std::atomic<int> output_count{0};
  autoware_planning_msgs::Trajectory received;
  const auto subscriber = node.subscribe<autoware_planning_msgs::Trajectory>(
    "/mvp_transport_oracle/output/trajectory", 1,
    [&output_count, &received](const autoware_planning_msgs::TrajectoryConstPtr & message) {
      received = *message;
      ++output_count;
    });
  const auto publisher = node.advertise<autoware_planning_msgs::Trajectory>(
    "/mvp_transport_oracle/input/trajectory", 1, true);
  const auto odometry_publisher = node.advertise<nav_msgs::Odometry>(
    "/mvp_transport_oracle/input/odometry", 1, true);
  const auto acceleration_publisher = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/mvp_transport_oracle/input/acceleration", 1, true);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry_publisher.publish(odometry);
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  acceleration_publisher.publish(acceleration);
  autoware_planning_msgs::Trajectory input;
  input.header.frame_id = "map";
  input.header.stamp = ros::Time(42.0);
  autoware_planning_msgs::TrajectoryPoint first;
  first.pose.position.x = 1.0;
  first.longitudinal_velocity_mps = 5.0F;
  autoware_planning_msgs::TrajectoryPoint second;
  second.pose.position.x = 2.0;
  second.longitudinal_velocity_mps = 2.0F;
  input.points = {first, second};
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::ok() && ros::WallTime::now() < deadline && output_count.load() == 0) {
    publisher.publish(input);
    ros::WallDuration(0.05).sleep();
  }
  if (output_count.load() != 1) {
    spinner.stop();
    ADD_FAILURE() << "MVP did not forward after odometry and acceleration became available";
    return;
  }
  EXPECT_EQ(received.header.frame_id, input.header.frame_id);
  EXPECT_EQ(received.header.stamp, input.header.stamp);
  if (received.points.size() != 2U) {
    spinner.stop();
    ADD_FAILURE() << "MVP changed the empty-module trajectory cardinality";
    return;
  }
  EXPECT_FLOAT_EQ(received.points[0].longitudinal_velocity_mps, 5.0F);
  EXPECT_FLOAT_EQ(received.points[1].longitudinal_velocity_mps, 2.0F);
  spinner.stop();
  (void)subscriber;
}

TEST(MotionVelocityPlannerRos1Node, RejectsUnavailableLaunchModuleWithoutRegisteringIt)
{
  ros::NodeHandle planner_node("/mvp_launch_module_oracle");
  XmlRpc::XmlRpcValue modules;
  modules.setSize(1);
  modules[0] = "autoware::motion_velocity_planner::MissingModule";
  planner_node.setParam("launch_modules", modules);
  autoware::motion_velocity_planner::MotionVelocityPlannerRos1Node planner(planner_node);
  EXPECT_TRUE(planner.hasLaunchModuleConfiguration());
  EXPECT_EQ(planner.loadedModuleCount(), 0U);
}

TEST(MotionVelocityPlannerRos1Node, WaitsForRequiredObjectsThenAppliesObstacleStopPlugin)
{
  ros::NodeHandle node;
  ros::NodeHandle planner_node("/mvp_obstacle_stop_oracle");
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find autoware_motion_velocity_planner)/config/ros1/"
      "obstacle_stop_default.yaml\" /mvp_obstacle_stop_oracle"),
    0);
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find autoware_velocity_smoother)/config/ros1/"
      "default_common.yaml\" /mvp_obstacle_stop_oracle"),
    0);
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find autoware_velocity_smoother)/config/ros1/"
      "Analytical.yaml\" /mvp_obstacle_stop_oracle"),
    0);
  XmlRpc::XmlRpcValue modules;
  modules.setSize(1);
  modules[0] = "autoware::motion_velocity_planner::ObstacleStopModule";
  planner_node.setParam("launch_modules", modules);
  planner_node.setParam("obstacle_stop/obstacle_filtering/lateral_margin/nominal", 0.0);
  planner_node.setParam("obstacle_stop/stop_planning/stop_margin", 1.0);
  planner_node.setParam("limit/max_acc", 2.0);
  planner_node.setParam("limit/min_acc", -2.0);
  planner_node.setParam("vehicle_info/max_longitudinal_offset_m", 2.0);
  planner_node.setParam("vehicle_info/rear_overhang_m", 1.0);
  planner_node.setParam("vehicle_info/vehicle_width_m", 2.0);
  autoware::motion_velocity_planner::MotionVelocityPlannerRos1Node planner(planner_node);
  ASSERT_EQ(planner.loadedModuleCount(), 1U);

  std::atomic<int> output_count{0};
  autoware_planning_msgs::Trajectory received;
  const auto subscriber = node.subscribe<autoware_planning_msgs::Trajectory>(
    "/mvp_obstacle_stop_oracle/output/trajectory", 1,
    [&output_count, &received](const autoware_planning_msgs::TrajectoryConstPtr & message) {
      received = *message;
      ++output_count;
    });
  const auto trajectory_publisher = node.advertise<autoware_planning_msgs::Trajectory>(
    "/mvp_obstacle_stop_oracle/input/trajectory", 1, true);
  const auto objects_publisher = node.advertise<autoware_perception_msgs::PredictedObjects>(
    "/mvp_obstacle_stop_oracle/input/predicted_objects", 1, true);
  const auto cloud_publisher = node.advertise<sensor_msgs::PointCloud2>(
    "/mvp_obstacle_stop_oracle/input/no_ground_pointcloud",1,true);
  const auto odometry_publisher = node.advertise<nav_msgs::Odometry>(
    "/mvp_obstacle_stop_oracle/input/odometry", 1, true);
  const auto acceleration_publisher = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/mvp_obstacle_stop_oracle/input/acceleration", 1, true);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 5.0;
  odometry_publisher.publish(odometry);
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  acceleration_publisher.publish(acceleration);

  autoware_planning_msgs::Trajectory input;
  input.header.frame_id = "map";
  for (size_t index = 0; index < 5; ++index) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(index) * 5.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0F;
    input.points.push_back(point);
  }
  trajectory_publisher.publish(input);
  ros::WallDuration(0.2).sleep();
  EXPECT_EQ(output_count.load(), 0);

  autoware_perception_msgs::PredictedObjects objects;
  objects.header.frame_id = "map";
  autoware_perception_msgs::PredictedObject object;
  object.classification.resize(1);
  object.classification.front().label=autoware_perception_msgs::ObjectClassification::UNKNOWN;
  object.classification.front().probability=1.0;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 10.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  objects.objects.push_back(object);
  objects_publisher.publish(objects);
  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  sensor_msgs::PointCloud2 cloud;pcl::toROSMsg(empty_cloud,cloud);
  cloud.header.frame_id="map";cloud_publisher.publish(cloud);
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::ok() && ros::WallTime::now() < deadline && output_count.load() == 0) {
    trajectory_publisher.publish(input);
    ros::WallDuration(0.05).sleep();
  }
  if (output_count.load() != 1) {
    spinner.stop();
    ADD_FAILURE() << "Obstacle-stop MVP did not forward after required objects arrived";
    return;
  }
  // The conservative stop result may insert an exact stop point between
  // source points; it must never discard the input trajectory wholesale.
  if (received.points.size() < input.points.size()) {
    spinner.stop();
    ADD_FAILURE() << "Obstacle-stop MVP discarded the source trajectory";
    return;
  }
  EXPECT_FLOAT_EQ(received.points.back().longitudinal_velocity_mps, 0.0F);
  spinner.stop();
  (void)subscriber;
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motion_velocity_planner_ros1_node_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
