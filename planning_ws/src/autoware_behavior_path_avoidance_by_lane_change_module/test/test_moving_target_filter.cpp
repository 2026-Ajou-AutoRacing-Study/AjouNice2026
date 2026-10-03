#include "moving_target_filter.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace bpp = autoware::behavior_path_planner;
namespace sa = bpp::utils::static_obstacle_avoidance;
namespace
{
auto parameters()
{
  auto p = std::make_shared<bpp::AvoidanceParameters>();
  p->object_parameters[1].moving_speed_threshold = 1.0;
  p->object_parameters[1].moving_time_threshold = 1.0;
  return p;
}
bpp::ObjectData object(double speed, double time)
{
  bpp::ObjectData o;
  o.object.classification.resize(1);
  o.object.classification[0].label = 1;
  o.object.classification[0].probability = 1.0;
  o.object.object_id.uuid[0] = 25;
  o.object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
  o.move_time = time;
  return o;
}
}

TEST(AvlcMovingFilter, NewMovingUsesExistingMovingTimeInitializer)
{
  ros::Time::init();
  auto o = object(40.0 / 3.6, 0);
  bpp::ObjectDataArray stopped, other;
  sa::fillObjectMovingTime(o, stopped, parameters());
  EXPECT_TRUE(std::isinf(o.move_time));
  EXPECT_TRUE(bpp::avoidance_lane_change_detail::excludeMovingTarget(o, parameters(), other));
  ASSERT_EQ(other.size(), 1U);
  EXPECT_EQ(other.front().info, bpp::ObjectInfo::MOVING_OBJECT);
}

TEST(AvlcMovingFilter, NewStoppedRemainsTarget)
{
  ros::Time::init();
  auto o = object(0, 999);
  bpp::ObjectDataArray stopped, other;
  sa::fillObjectMovingTime(o, stopped, parameters());
  EXPECT_EQ(o.move_time, 0);
  EXPECT_FALSE(bpp::avoidance_lane_change_detail::excludeMovingTarget(o, parameters(), other));
}

TEST(AvlcMovingFilter, RestartTimeBoundaryPreserved)
{
  for (double t : {0.0, .999, 1.0, 1.000001}) {
    bpp::ObjectDataArray other;
    EXPECT_EQ(bpp::avoidance_lane_change_detail::excludeMovingTarget(
      object(10, t), parameters(), other), t > 1.0);
  }
}

TEST(AvlcMovingFilter, OtherObjectsPreventsLostTargetResurrection)
{
  auto o = object(10, std::numeric_limits<double>::infinity());
  bpp::AvoidancePlanningData data;
  ASSERT_TRUE(bpp::avoidance_lane_change_detail::excludeMovingTarget(
    o, parameters(), data.other_objects));
  // The actual compensation function must skip the cached target before
  // dereferencing PlannerData (deliberately absent in this negative fixture).
  sa::compensateLostTargetObjects(data, {o}, nullptr);
  EXPECT_TRUE(data.target_objects.empty());
  EXPECT_EQ(data.other_objects.front().object, o.object);
}

TEST(AvlcMovingFilter, InputAndRawObjectUnchanged)
{
  const auto raw = std::make_shared<const bpp::PredictedObject>(object(10, 2).object);
  const auto * identity = raw.get();
  auto o = object(10, 2);
  const auto original = o.object;
  const auto original_info = o.info;
  bpp::ObjectDataArray other;
  ASSERT_TRUE(bpp::avoidance_lane_change_detail::excludeMovingTarget(o, parameters(), other));
  EXPECT_EQ(o.object, original);
  EXPECT_EQ(o.info, original_info);
  EXPECT_EQ(raw.get(), identity);
  EXPECT_EQ(*raw, original);
}

TEST(AvlcMovingFilter, CachedTargetMovesToCenterCannotResurrect)
{
  ros::Time::init();
  const auto old_target = object(0, 0);
  auto raw = old_target.object;
  raw.kinematics.initial_twist_with_covariance.twist.linear.x = 40.0 / 3.6;
  bpp::ObjectDataArray stopped;
  bpp::AvoidancePlanningData data;
  ASSERT_TRUE(bpp::avoidance_lane_change_detail::excludeMovingCenterFilteredObject(
    raw, stopped, parameters(), data.other_objects));
  sa::compensateLostTargetObjects(data, {old_target}, nullptr);
  EXPECT_TRUE(data.target_objects.empty());
  ASSERT_EQ(data.other_objects.size(), 1U);
  EXPECT_EQ(data.other_objects.front().object, raw);
}

TEST(AvlcMovingFilter, ActualRestartHistoryPreservesGraceAndExpires)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(100));
  for (const double elapsed : {.5, 1.0, 2.0}) {
    auto stopped_object = object(0, 0);
    stopped_object.last_stop = ros::Time(100) - ros::Duration(elapsed);
    bpp::ObjectDataArray stopped{stopped_object}, other;
    const auto moving = object(10, 0);
    EXPECT_EQ(bpp::avoidance_lane_change_detail::excludeMovingCenterFilteredObject(
      moving.object, stopped, parameters(), other), elapsed > 1.0);
    EXPECT_EQ(stopped.empty(), elapsed > 1.0);
    EXPECT_EQ(other.empty(), elapsed <= 1.0);
  }
}

TEST(AvlcMovingFilter, CenterStationaryHistoryUnchanged)
{
  for (const double speed : {0.0, 1.0}) {
    bpp::ObjectDataArray stopped, other;
    EXPECT_FALSE(bpp::avoidance_lane_change_detail::excludeMovingCenterFilteredObject(
      object(speed, 0).object, stopped, parameters(), other));
    EXPECT_TRUE(stopped.empty());
    EXPECT_TRUE(other.empty());
  }
}
