#include "autoware/scenario_selector/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware/motion_velocity_planner/motion_velocity_planner_ros1_node.hpp>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <gtest/gtest.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <atomic>
#include <string>

namespace
{
TEST(ScenarioSelectorNode, SelectsLaneDrivingAndForwardsFreshTrajectoryOnlyAfterReadiness)
{
  ros::NodeHandle root;
  ros::NodeHandle selector_node("/scenario_selector_transport");
  autoware::scenario_selector::ScenarioSelectorNode selector(selector_node);
  ros::NodeHandle mvp_node("/scenario_selector_transport/mvp");
  autoware::motion_velocity_planner::MotionVelocityPlannerRos1Node mvp(mvp_node);
  std::atomic<int> scenarios{0};
  std::atomic<int> trajectories{0};
  std::string selected;
  std::string output_frame;
  const auto scenario_sub = root.subscribe<autoware_internal_planning_msgs::Scenario>(
    "/scenario_selector_transport/output/scenario", 1,
    [&](const autoware_internal_planning_msgs::Scenario::ConstPtr & message) {
      selected = message->current_scenario;
      ++scenarios;
    });
  const auto trajectory_sub = root.subscribe<autoware_planning_msgs::Trajectory>(
    "/scenario_selector_transport/output/trajectory", 1,
    [&](const autoware_planning_msgs::Trajectory::ConstPtr & message) {
      output_frame = message->header.frame_id;
      ++trajectories;
    });
  const auto map_pub = root.advertise<autoware_map_msgs::LaneletMapBin>(
    "/scenario_selector_transport/input/lanelet_map", 1, true);
  const auto route_pub = root.advertise<autoware_planning_msgs::LaneletRoute>(
    "/scenario_selector_transport/input/route", 1, true);
  const auto odom_pub = root.advertise<nav_msgs::Odometry>(
    "/scenario_selector_transport/input/odometry", 1, true);
  const auto mode_pub = root.advertise<autoware_adapi_v1_msgs::OperationModeState>(
    "/scenario_selector_transport/input/operation_mode_state", 1, true);
  const auto lane_trajectory_pub = root.advertise<autoware_planning_msgs::Trajectory>(
    "/scenario_selector_transport/input/lane_driving/trajectory", 1);
  const auto mvp_input_pub = root.advertise<autoware_planning_msgs::Trajectory>(
    "/scenario_selector_transport/mvp/input/trajectory", 1);
  // The direct MVP instance owns its own private readiness gate.  Supplying
  // only its trajectory input makes the intended fail-closed branch run and
  // cannot establish the selector-to-MVP transport contract.
  const auto mvp_odometry_pub = root.advertise<nav_msgs::Odometry>(
    "/scenario_selector_transport/mvp/input/odometry", 1, true);
  const auto mvp_acceleration_pub = root.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/scenario_selector_transport/mvp/input/acceleration", 1, true);
  const auto mvp_output_sub = root.subscribe<autoware_planning_msgs::Trajectory>(
    "/scenario_selector_transport/mvp/output/trajectory", 1,
    [&](const autoware_planning_msgs::Trajectory::ConstPtr & message) { lane_trajectory_pub.publish(*message); });
  // The frozen component is registered with a single-threaded executor; keep
  // callback and timer ordering deterministic in the ROS1 transport oracle.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  const auto centerline = lane.centerline();
  ASSERT_GE(centerline.size(), 2U);
  auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  map_message.header.frame_id = "map";
  map_message.version_map_format = "1";
  map_message.version_map = "5";
  geometry_msgs::AccelWithCovarianceStamped acceleration;

  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = centerline.front().x();
  route.start_pose.position.y = centerline.front().y();
  route.start_pose.orientation.w = 1.0;
  route.goal_pose.position.x = centerline.back().x();
  route.goal_pose.position.y = centerline.back().y();
  route.goal_pose.orientation.w = 1.0;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  route.segments.push_back(segment);

  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose = route.start_pose;
  odometry.twist.twist.linear.x = 0.0;
  autoware_adapi_v1_msgs::OperationModeState mode;
  mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;

  const auto deadline = ros::WallTime::now() + ros::WallDuration(4.0);
  while (ros::ok() && ros::WallTime::now() < deadline &&
         (scenarios.load() == 0 || trajectories.load() == 0)) {
    autoware_planning_msgs::Trajectory trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = ros::Time::now();
    trajectory.points.resize(2);
    trajectory.points.front().pose = route.start_pose;
    trajectory.points.back().pose = route.goal_pose;
    trajectory.points.front().pose.orientation.w = 1.0;
    trajectory.points.back().pose.orientation.w = 1.0;
    map_pub.publish(map_message);
    route_pub.publish(route);
    odom_pub.publish(odometry);
    mode_pub.publish(mode);
    mvp_odometry_pub.publish(odometry);
    mvp_acceleration_pub.publish(acceleration);
    mvp_input_pub.publish(trajectory);
    ros::WallDuration(0.05).sleep();
  }

  EXPECT_GT(scenarios.load(), 0);
  EXPECT_EQ(selected, autoware_internal_planning_msgs::Scenario::LANEDRIVING);
  EXPECT_GT(trajectories.load(), 0);
  EXPECT_EQ(output_frame, "map");

  ros::WallDuration(0.2).sleep();
  const int fresh_output_count = trajectories.load();
  autoware_planning_msgs::Trajectory stale_trajectory;
  stale_trajectory.header.frame_id = "map";
  stale_trajectory.header.stamp = ros::Time::now() - ros::Duration(2.0);
  stale_trajectory.points.resize(2);
  stale_trajectory.points.front().pose = route.start_pose;
  stale_trajectory.points.back().pose = route.goal_pose;
  mvp_input_pub.publish(stale_trajectory);
  ros::WallDuration(0.3).sleep();
  EXPECT_EQ(trajectories.load(), fresh_output_count);
  spinner.stop();
  (void)scenario_sub;
  (void)trajectory_sub;
  (void)mvp_output_sub;
}

TEST(ScenarioSelectorNode, ResetsScenarioImmediatelyForRouteWithoutReadyHandler)
{
  ros::NodeHandle node("/scenario_selector_uuid_reset");
  autoware::scenario_selector::ScenarioSelectorNode selector(node);
  selector.testSetScenario(autoware_internal_planning_msgs::Scenario::LANEDRIVING);
  auto route = boost::make_shared<autoware_planning_msgs::LaneletRoute>();
  selector.testOnRoute(route);
  EXPECT_EQ(selector.testScenario(), autoware_internal_planning_msgs::Scenario::EMPTY);
}

TEST(ScenarioSelectorNode, SelectsParkingForPoseInsideFixtureParkingLot)
{
  ros::NodeHandle root;
  ros::NodeHandle selector_node("/scenario_selector_parking_transport");
  autoware::scenario_selector::ScenarioSelectorNode selector(selector_node);
  std::atomic<int> scenarios{0};
  std::string selected;
  const auto scenario_sub = root.subscribe<autoware_internal_planning_msgs::Scenario>(
    "/scenario_selector_parking_transport/output/scenario", 1,
    [&](const autoware_internal_planning_msgs::Scenario::ConstPtr & message) {
      selected = message->current_scenario;
      ++scenarios;
    });
  const auto map_pub = root.advertise<autoware_map_msgs::LaneletMapBin>(
    "/scenario_selector_parking_transport/input/lanelet_map", 1, true);
  const auto route_pub = root.advertise<autoware_planning_msgs::LaneletRoute>(
    "/scenario_selector_parking_transport/input/route", 1, true);
  const auto odom_pub = root.advertise<nav_msgs::Odometry>(
    "/scenario_selector_parking_transport/input/odometry", 1, true);
  const auto mode_pub = root.advertise<autoware_adapi_v1_msgs::OperationModeState>(
    "/scenario_selector_parking_transport/input/operation_mode_state", 1, true);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(map);
  ASSERT_FALSE(parking_lots.empty());
  const auto & parking_lot = parking_lots.front();
  ASSERT_GE(parking_lot.size(), 3U);
  geometry_msgs::Pose parking_pose;
  for (const auto & point : parking_lot) {
    parking_pose.position.x += point.x();
    parking_pose.position.y += point.y();
  }
  parking_pose.position.x /= static_cast<double>(parking_lot.size());
  parking_pose.position.y /= static_cast<double>(parking_lot.size());
  parking_pose.orientation.w = 1.0;

  const auto lane = *map->laneletLayer.begin();
  const auto centerline = lane.centerline();
  ASSERT_GE(centerline.size(), 2U);
  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = centerline.front().x();
  route.start_pose.position.y = centerline.front().y();
  route.start_pose.orientation.w = 1.0;
  route.goal_pose = parking_pose;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  route.segments.push_back(segment);

  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose = parking_pose;
  odometry.twist.twist.linear.x = 0.0;
  autoware_adapi_v1_msgs::OperationModeState mode;
  mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  const auto deadline = ros::WallTime::now() + ros::WallDuration(4.0);
  while (ros::ok() && ros::WallTime::now() < deadline &&
         (scenarios.load() == 0 || selected != autoware_internal_planning_msgs::Scenario::PARKING)) {
    map_pub.publish(map_message);
    route_pub.publish(route);
    odom_pub.publish(odometry);
    mode_pub.publish(mode);
    ros::WallDuration(0.05).sleep();
  }

  EXPECT_GT(scenarios.load(), 0);
  EXPECT_EQ(selected, autoware_internal_planning_msgs::Scenario::PARKING);
  spinner.stop();
  (void)scenario_sub;
}

TEST(ScenarioSelectorNode, SwitchesFromLaneDrivingToParkingAfterContinuousAutonomousStop)
{
  ros::NodeHandle root;
  ros::NodeHandle selector_node("/scenario_selector_mode_switch_transport");
  autoware::scenario_selector::ScenarioSelectorNode selector(selector_node);
  std::atomic<int> scenarios{0};
  std::string selected;
  const auto scenario_sub = root.subscribe<autoware_internal_planning_msgs::Scenario>(
    "/scenario_selector_mode_switch_transport/output/scenario", 1,
    [&](const autoware_internal_planning_msgs::Scenario::ConstPtr & message) {
      selected = message->current_scenario;
      ++scenarios;
    });
  const auto map_pub = root.advertise<autoware_map_msgs::LaneletMapBin>(
    "/scenario_selector_mode_switch_transport/input/lanelet_map", 1, true);
  const auto route_pub = root.advertise<autoware_planning_msgs::LaneletRoute>(
    "/scenario_selector_mode_switch_transport/input/route", 1, true);
  const auto odom_pub = root.advertise<nav_msgs::Odometry>(
    "/scenario_selector_mode_switch_transport/input/odometry", 1, true);
  const auto mode_pub = root.advertise<autoware_adapi_v1_msgs::OperationModeState>(
    "/scenario_selector_mode_switch_transport/input/operation_mode_state", 1, true);
  const auto lane_trajectory_pub = root.advertise<autoware_planning_msgs::Trajectory>(
    "/scenario_selector_mode_switch_transport/input/lane_driving/trajectory", 1);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(map);
  ASSERT_FALSE(parking_lots.empty());
  const auto & parking_lot = parking_lots.front();
  ASSERT_GE(parking_lot.size(), 3U);
  geometry_msgs::Pose parking_pose;
  for (const auto & point : parking_lot) {
    parking_pose.position.x += point.x();
    parking_pose.position.y += point.y();
  }
  parking_pose.position.x /= static_cast<double>(parking_lot.size());
  parking_pose.position.y /= static_cast<double>(parking_lot.size());
  parking_pose.orientation.w = 1.0;

  const auto lane = *map->laneletLayer.begin();
  const auto centerline = lane.centerline();
  ASSERT_GE(centerline.size(), 2U);
  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = centerline.front().x();
  route.start_pose.position.y = centerline.front().y();
  route.start_pose.orientation.w = 1.0;
  route.goal_pose = parking_pose;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  route.segments.push_back(segment);
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);

  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose = route.start_pose;
  odometry.twist.twist.linear.x = 0.0;
  autoware_adapi_v1_msgs::OperationModeState local_mode;
  local_mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  autoware_planning_msgs::Trajectory lane_trajectory;
  lane_trajectory.header.frame_id = "map";
  lane_trajectory.points.resize(2);
  lane_trajectory.points.front().pose = route.start_pose;
  lane_trajectory.points.back().pose = route.start_pose;

  const auto lane_deadline = ros::WallTime::now() + ros::WallDuration(4.0);
  while (ros::ok() && ros::WallTime::now() < lane_deadline &&
         (scenarios.load() == 0 || selected != autoware_internal_planning_msgs::Scenario::LANEDRIVING)) {
    odometry.header.stamp = ros::Time::now();
    lane_trajectory.header.stamp = odometry.header.stamp;
    map_pub.publish(map_message);
    route_pub.publish(route);
    odom_pub.publish(odometry);
    mode_pub.publish(local_mode);
    lane_trajectory_pub.publish(lane_trajectory);
    ros::WallDuration(0.05).sleep();
  }
  ASSERT_EQ(selected, autoware_internal_planning_msgs::Scenario::LANEDRIVING);

  autoware_adapi_v1_msgs::OperationModeState autonomous_mode;
  autonomous_mode.mode = autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS;
  autonomous_mode.is_autoware_control_enabled = true;
  odometry.pose.pose = parking_pose;
  const auto switch_deadline = ros::WallTime::now() + ros::WallDuration(7.0);
  while (ros::ok() && ros::WallTime::now() < switch_deadline &&
         selected != autoware_internal_planning_msgs::Scenario::PARKING) {
    odometry.header.stamp = ros::Time::now();
    lane_trajectory.header.stamp = odometry.header.stamp;
    odom_pub.publish(odometry);
    mode_pub.publish(autonomous_mode);
    lane_trajectory_pub.publish(lane_trajectory);
    ros::WallDuration(0.05).sleep();
  }

  EXPECT_EQ(selected, autoware_internal_planning_msgs::Scenario::PARKING);
  spinner.stop();
  (void)scenario_sub;
}

TEST(ScenarioSelectorNode, SwitchesFromParkingToLaneDrivingAfterContinuousEmptyTrajectory)
{
  ros::NodeHandle root;
  ros::NodeHandle selector_node("/scenario_selector_return_switch_transport");
  autoware::scenario_selector::ScenarioSelectorNode selector(selector_node);
  std::atomic<int> scenarios{0};
  std::string selected;
  const auto scenario_sub = root.subscribe<autoware_internal_planning_msgs::Scenario>(
    "/scenario_selector_return_switch_transport/output/scenario", 1,
    [&](const autoware_internal_planning_msgs::Scenario::ConstPtr & message) {
      selected = message->current_scenario;
      ++scenarios;
    });
  const auto map_pub = root.advertise<autoware_map_msgs::LaneletMapBin>(
    "/scenario_selector_return_switch_transport/input/lanelet_map", 1, true);
  const auto route_pub = root.advertise<autoware_planning_msgs::LaneletRoute>(
    "/scenario_selector_return_switch_transport/input/route", 1, true);
  const auto odom_pub = root.advertise<nav_msgs::Odometry>(
    "/scenario_selector_return_switch_transport/input/odometry", 1, true);
  const auto mode_pub = root.advertise<autoware_adapi_v1_msgs::OperationModeState>(
    "/scenario_selector_return_switch_transport/input/operation_mode_state", 1, true);
  const auto parking_trajectory_pub = root.advertise<autoware_planning_msgs::Trajectory>(
    "/scenario_selector_return_switch_transport/input/parking/trajectory", 1);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(map);
  ASSERT_FALSE(parking_lots.empty());
  const auto & parking_lot = parking_lots.front();
  ASSERT_GE(parking_lot.size(), 3U);
  geometry_msgs::Pose parking_pose;
  for (const auto & point : parking_lot) {
    parking_pose.position.x += point.x();
    parking_pose.position.y += point.y();
  }
  parking_pose.position.x /= static_cast<double>(parking_lot.size());
  parking_pose.position.y /= static_cast<double>(parking_lot.size());
  parking_pose.orientation.w = 1.0;

  const auto lane = *map->laneletLayer.begin();
  const auto centerline = lane.centerline();
  ASSERT_GE(centerline.size(), 2U);
  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = centerline.front().x();
  route.start_pose.position.y = centerline.front().y();
  route.start_pose.orientation.w = 1.0;
  route.goal_pose = parking_pose;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  route.segments.push_back(segment);
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);

  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose = parking_pose;
  odometry.twist.twist.linear.x = 0.0;
  autoware_adapi_v1_msgs::OperationModeState mode;
  mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  const auto parking_deadline = ros::WallTime::now() + ros::WallDuration(4.0);
  while (ros::ok() && ros::WallTime::now() < parking_deadline &&
         (scenarios.load() == 0 || selected != autoware_internal_planning_msgs::Scenario::PARKING)) {
    odometry.header.stamp = ros::Time::now();
    map_pub.publish(map_message);
    route_pub.publish(route);
    odom_pub.publish(odometry);
    mode_pub.publish(mode);
    ros::WallDuration(0.05).sleep();
  }
  ASSERT_EQ(selected, autoware_internal_planning_msgs::Scenario::PARKING);

  odometry.pose.pose = route.start_pose;
  autoware_planning_msgs::Trajectory empty_parking_trajectory;
  empty_parking_trajectory.header.frame_id = "map";
  empty_parking_trajectory.points.resize(1);
  empty_parking_trajectory.points.front().pose = route.start_pose;
  const auto switch_deadline = ros::WallTime::now() + ros::WallDuration(5.0);
  while (ros::ok() && ros::WallTime::now() < switch_deadline &&
         selected != autoware_internal_planning_msgs::Scenario::LANEDRIVING) {
    odometry.header.stamp = ros::Time::now();
    empty_parking_trajectory.header.stamp = odometry.header.stamp;
    odom_pub.publish(odometry);
    mode_pub.publish(mode);
    parking_trajectory_pub.publish(empty_parking_trajectory);
    ros::WallDuration(0.05).sleep();
  }

  EXPECT_EQ(selected, autoware_internal_planning_msgs::Scenario::LANEDRIVING);
  spinner.stop();
  (void)scenario_sub;
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "scenario_selector_node_transport_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
