#include "autoware/remaining_distance_time_calculator/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_internal_msgs/MissionRemainingDistanceTime.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <atomic>
#include <cmath>

namespace
{
TEST(RemainingDistanceTimeCalculator, ParkingPublishesImmediateZeroWithoutMap)
{
  ros::NodeHandle root, node("/rdt_parking");
  autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode calculator(node);
  std::atomic<int> received{0};
  autoware_internal_msgs::MissionRemainingDistanceTime latest;
  const auto out = root.subscribe<autoware_internal_msgs::MissionRemainingDistanceTime>(
    "/rdt_parking/output/mission_remaining_distance_time", 1,
    [&](const autoware_internal_msgs::MissionRemainingDistanceTime::ConstPtr & message) { latest = *message; ++received; });
  const auto scenario = root.advertise<autoware_internal_planning_msgs::Scenario>("/rdt_parking/input/scenario", 1, true);
  ros::AsyncSpinner spinner(1); spinner.start();
  autoware_internal_planning_msgs::Scenario parking;
  parking.current_scenario = autoware_internal_planning_msgs::Scenario::PARKING;
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::WallTime::now() < deadline && received.load() == 0) { scenario.publish(parking); ros::WallDuration(0.02).sleep(); }
  ASSERT_GT(received.load(), 0);
  EXPECT_DOUBLE_EQ(latest.remaining_distance, 0.0);
  EXPECT_DOUBLE_EQ(latest.remaining_time, 0.0);
}

TEST(RemainingDistanceTimeCalculator, LaneDrivingUsesRouteDistanceAndVelocityLimit)
{
  ros::NodeHandle root, node("/rdt_lane");
  autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode calculator(node);
  std::atomic<int> received{0};
  autoware_internal_msgs::MissionRemainingDistanceTime latest;
  const auto out = root.subscribe<autoware_internal_msgs::MissionRemainingDistanceTime>(
    "/rdt_lane/output/mission_remaining_distance_time", 1,
    [&](const autoware_internal_msgs::MissionRemainingDistanceTime::ConstPtr & message) { latest = *message; ++received; });
  const auto map_pub = root.advertise<autoware_map_msgs::LaneletMapBin>("/rdt_lane/input/map", 1, true);
  const auto route_pub = root.advertise<autoware_planning_msgs::LaneletRoute>("/rdt_lane/input/route", 1, true);
  const auto odom_pub = root.advertise<nav_msgs::Odometry>("/rdt_lane/input/odometry", 1, true);
  const auto scenario_pub = root.advertise<autoware_internal_planning_msgs::Scenario>("/rdt_lane/input/scenario", 1, true);
  const auto limit_pub = root.advertise<autoware_internal_planning_msgs::VelocityLimit>("/rdt_lane/input/velocity_limit", 1, true);
  ros::AsyncSpinner spinner(1); spinner.start();

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map); ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin(); const auto centerline = lane.centerline(); ASSERT_GE(centerline.size(), 2U);
  auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  map_message.header.frame_id = "map"; map_message.version_map_format = "1"; map_message.version_map = "5";
  autoware_planning_msgs::LaneletRoute route; route.header.frame_id = "map";
  route.start_pose.position.x = centerline.front().x(); route.start_pose.position.y = centerline.front().y(); route.start_pose.orientation.w = 1.0;
  route.goal_pose.position.x = centerline.back().x(); route.goal_pose.position.y = centerline.back().y(); route.goal_pose.orientation.w = 1.0;
  autoware_planning_msgs::LaneletPrimitive primitive; primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment; segment.primitives.push_back(primitive); segment.preferred_primitive = primitive; route.segments.push_back(segment);
  nav_msgs::Odometry odometry; odometry.pose.pose = route.start_pose;
  autoware_internal_planning_msgs::Scenario lane_driving; lane_driving.current_scenario = autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  autoware_internal_planning_msgs::VelocityLimit limit; limit.max_velocity = 5.0F;
  const auto deadline = ros::WallTime::now() + ros::WallDuration(3.0);
  while (ros::WallTime::now() < deadline && received.load() == 0) {
    map_pub.publish(map_message); route_pub.publish(route); odom_pub.publish(odometry); scenario_pub.publish(lane_driving); limit_pub.publish(limit); ros::WallDuration(0.025).sleep();
  }
  ASSERT_GT(received.load(), 0);
  EXPECT_GT(latest.remaining_distance, 0.1);
  EXPECT_NEAR(latest.remaining_time, latest.remaining_distance / 5.0, 1e-4);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_remaining_distance_time_transport");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
