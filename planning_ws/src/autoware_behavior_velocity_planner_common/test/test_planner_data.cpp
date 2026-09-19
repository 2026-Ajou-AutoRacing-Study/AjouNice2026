#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <gtest/gtest.h>
#include <ros/init.h>

namespace autoware::behavior_velocity_planner
{
namespace
{
geometry_msgs::TwistStamped velocityAt(const ros::Time & stamp, const double velocity)
{
  geometry_msgs::TwistStamped message;
  message.header.stamp = stamp;
  message.twist.linear.x = velocity;
  return message;
}

TEST(PlannerData, StopsOnlyWhenTheRequiredRecentWindowHasNoMovingSample)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(10, 0));
  PlannerData data;
  EXPECT_FALSE(data.isVehicleStopped());

  data.velocity_buffer.push_back(velocityAt(ros::Time(9, 500000000), 0.0));
  EXPECT_TRUE(data.isVehicleStopped(0.4));

  data.velocity_buffer.push_front(velocityAt(ros::Time(9, 900000000), 0.01));
  EXPECT_FALSE(data.isVehicleStopped(0.4));

  data.velocity_buffer.front().twist.linear.x = 0.0;
  EXPECT_TRUE(data.isVehicleStopped(0.4));
}

TEST(PlannerData, SelectsRawOrLastTrafficSignalWithoutFallback)
{
  PlannerData data;
  TrafficSignalStamped raw;
  raw.stamp = ros::Time(1, 0);
  TrafficSignalStamped last;
  last.stamp = ros::Time(2, 0);
  data.traffic_light_id_map_raw_.emplace(42, raw);
  data.traffic_light_id_map_last_observed_.emplace(42, last);

  ASSERT_TRUE(data.getTrafficSignal(42));
  EXPECT_EQ(data.getTrafficSignal(42)->stamp, ros::Time(1, 0));
  ASSERT_TRUE(data.getTrafficSignal(42, true));
  EXPECT_EQ(data.getTrafficSignal(42, true)->stamp, ros::Time(2, 0));
  EXPECT_FALSE(data.getTrafficSignal(43));
}

TEST(PlannerData, ReadsVehicleGeometryAndPlannerThresholdsFromItsPrivateNamespace)
{
  ros::NodeHandle node("planner_data_provider_test");
  node.setParam("wheel_radius", 0.5);
  node.setParam("wheel_width", 0.3);
  node.setParam("wheel_base", 2.7);
  node.setParam("wheel_tread", 1.6);
  node.setParam("front_overhang", 1.0);
  node.setParam("rear_overhang", 1.1);
  node.setParam("left_overhang", 0.2);
  node.setParam("right_overhang", 0.2);
  node.setParam("vehicle_height", 1.8);
  node.setParam("max_steer_angle", 0.6);
  node.setParam("max_accel", 1.2);
  node.setParam("max_jerk", 0.3);
  node.setParam("system_delay", 0.4);
  node.setParam("delay_response_time", 0.5);

  const PlannerData data(node);
  EXPECT_DOUBLE_EQ(data.vehicle_info_.wheel_base_m, 2.7);
  EXPECT_DOUBLE_EQ(data.max_stop_acceleration_threshold, 1.2);
  EXPECT_DOUBLE_EQ(data.max_stop_jerk_threshold, 0.3);
  EXPECT_DOUBLE_EQ(data.system_delay, 0.4);
  EXPECT_DOUBLE_EQ(data.delay_response_time, 0.5);
}

TEST(PlannerData, UpdatesOdometryAndDropsOnlyVelocitySamplesOlderThanTheBufferWindow)
{
  ros::Time::setNow(ros::Time(20, 0));
  PlannerData data;
  nav_msgs::Odometry old_message;
  old_message.header.stamp = ros::Time(9, 0);
  old_message.twist.twist.linear.x = 1.0;
  data.updateOdometry(old_message);

  nav_msgs::Odometry current_message;
  current_message.header.stamp = ros::Time(19, 500000000);
  current_message.pose.pose.position.x = 3.0;
  current_message.twist.twist.linear.x = 2.0;
  data.updateOdometry(current_message);

  ASSERT_TRUE(data.current_odometry);
  ASSERT_TRUE(data.current_velocity);
  EXPECT_DOUBLE_EQ(data.current_odometry->pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(data.current_velocity->twist.linear.x, 2.0);
  ASSERT_EQ(data.velocity_buffer.size(), 1U);
  EXPECT_DOUBLE_EQ(data.velocity_buffer.front().twist.linear.x, 2.0);
}

TEST(PlannerData, KeepsLastKnownTrafficSignalBodyWhenTheCurrentObservationIsUnknown)
{
  PlannerData data;
  autoware_perception_msgs::TrafficLightGroupArray observed;
  observed.stamp.sec = 1;
  observed.stamp.nanosec = 0;
  observed.traffic_light_groups.resize(1);
  observed.traffic_light_groups.front().traffic_light_group_id = 7;
  observed.traffic_light_groups.front().elements.resize(1);
  observed.traffic_light_groups.front().elements.front().color =
    autoware_perception_msgs::TrafficLightElement::GREEN;
  data.updateTrafficSignals(observed);

  auto unknown = observed;
  unknown.stamp.sec = 2;
  unknown.stamp.nanosec = 0;
  unknown.traffic_light_groups.front().elements.front().color =
    autoware_perception_msgs::TrafficLightElement::UNKNOWN;
  data.updateTrafficSignals(unknown);

  ASSERT_TRUE(data.getTrafficSignal(7));
  EXPECT_EQ(
    data.getTrafficSignal(7)->signal.elements.front().color,
    autoware_perception_msgs::TrafficLightElement::UNKNOWN);
  ASSERT_TRUE(data.getTrafficSignal(7, true));
  EXPECT_EQ(data.getTrafficSignal(7, true)->stamp, ros::Time(2, 0));
  EXPECT_EQ(
    data.getTrafficSignal(7, true)->signal.elements.front().color,
    autoware_perception_msgs::TrafficLightElement::GREEN);
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_behavior_velocity_planner_common_planner_data");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
