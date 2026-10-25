// Copyright 2026 AjouNice contributors
// Licensed under the Apache License, Version 2.0.
#include "pid_based_planner/pid_based_planner.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

namespace autoware::motion_velocity_planner
{
// Exercise the actual selection, filtering and output code without live planner publishers.
class PIDBasedPlannerTestPeer
{
public:
  static auto select(
    PIDBasedPlanner & planner, const std::vector<TrajectoryPoint> & trajectory,
    const std::shared_ptr<PlannerData> & data, const std::vector<CruiseObstacle> & obstacles)
  {
    return planner.calc_obstacle_to_cruise(trajectory, data, obstacles);
  }
  static VelocityLimit output(
    PIDBasedPlanner & planner, const std::shared_ptr<PlannerData> & data,
    const PIDBasedPlanner::CruiseObstacleInfo & obstacle)
  {
    return planner.plan_cruise_with_velocity_limit(data, obstacle);
  }
  static std::vector<float> debug(PIDBasedPlanner & planner)
  {
    return planner.get_cruise_planning_debug_message(ros::Time(10)).data;
  }
};

namespace
{
class PIDTargetSwitch : public ::testing::Test
{
protected:
  ros::NodeHandle node_{"~"};
  CruiseCommonParam common_;
  CruisePlanningParam rss_;
  std::shared_ptr<PlannerData> data_{std::make_shared<PlannerData>()};
  std::vector<TrajectoryPoint> trajectory_;

  void SetUp() override
  {
    const std::string prefix = "obstacle_cruise/cruise_planning/pid_based_planner/";
    node_.setParam(prefix + "min_accel_during_cruise", -6.0);
    node_.setParam(prefix + "use_velocity_limit_based_planner", true);
    node_.setParam(prefix + "min_cruise_target_vel", 0.0);
    node_.setParam(prefix + "time_to_evaluate_rss", 0.0);
    node_.setParam(prefix + "error_function_type", "quadratic");
    node_.setParam(prefix + "lpf_normalized_error_cruise_dist_gain", 0.2);
    const std::string velocity = prefix + "velocity_limit_based_planner/";
    node_.setParam(velocity + "kp", 10.0);
    node_.setParam(velocity + "ki", 1.0);  // Nonzero to expose integral carryover.
    node_.setParam(velocity + "kd", 2.0);
    node_.setParam(velocity + "output_ratio_during_accel", 0.6);
    node_.setParam(velocity + "vel_to_acc_weight", 12.0);
    node_.setParam(velocity + "enable_jerk_limit_to_output_acc", false);
    node_.setParam(velocity + "disable_target_acceleration", true);
    const std::string insertion = prefix + "velocity_insertion_based_planner/";
    for (const auto & key : {"kp_acc", "ki_acc", "kd_acc", "kp_jerk", "ki_jerk", "kd_jerk"}) {
      node_.setParam(insertion + key, 1.0);
    }
    node_.setParam(insertion + "output_acc_ratio_during_accel", 0.6);
    node_.setParam(insertion + "output_jerk_ratio_during_accel", 1.0);
    node_.setParam(insertion + "enable_jerk_limit_to_output_acc", true);
    common_.max_accel = 3.7;
    common_.min_accel = -6;
    common_.max_jerk = 100;
    common_.min_jerk = -10;
    rss_.idling_time = 2;
    rss_.min_ego_accel_for_rss = -6;
    rss_.min_object_accel_for_rss = -3;
    rss_.safe_distance_margin = 5;
    data_->current_odometry.pose.pose.orientation.w = 1;
    data_->current_odometry.twist.twist.linear.x = 20;
    data_->vehicle_info_.max_longitudinal_offset_m = 3.845;
    for (int x = 0; x <= 300; ++x) {
      TrajectoryPoint point;
      point.pose.position.x = x;
      point.pose.orientation.w = 1;
      point.longitudinal_velocity_mps = 30;
      trajectory_.push_back(point);
    }
  }

  CruiseObstacle obstacle(unsigned char id, double distance, double speed = 10)
  {
    UUID uuid;
    uuid.uuid[0] = 0x41;
    uuid.uuid[15] = id;  // Same shared prefix, different full UUID.
    polygon_utils::PointWithStamp point;
    point.point.x = distance + data_->vehicle_info_.max_longitudinal_offset_m;
    point.stamp = ros::Time(10);
    geometry_msgs::Pose pose;
    pose.position.x = point.point.x;
    pose.orientation.w = 1;
    return CruiseObstacle(uuid, ros::Time(10), pose, speed, 0, {point});
  }

  double cycle(PIDBasedPlanner & planner, const CruiseObstacle & target)
  {
    const auto selected = PIDBasedPlannerTestPeer::select(planner, trajectory_, data_, {target});
    EXPECT_TRUE(selected);
    if (!selected) return NAN;
    return PIDBasedPlannerTestPeer::output(planner, data_, *selected).max_velocity;
  }
};

TEST_F(PIDTargetSwitch, LaneChangeHandoffDropsHistoryEvenForTheSameReacquiredUuid)
{
  PIDBasedPlanner planner(node_,common_,rss_);
  PIDBasedPlanner fresh(node_,common_,rss_);
  for(int i=0;i<5;++i) cycle(planner,obstacle(1,150));
  planner.resetForLongitudinalHandoff();
  EXPECT_NEAR(cycle(planner,obstacle(1,20)),cycle(fresh,obstacle(1,20)),1e-9);
}

TEST_F(PIDTargetSwitch, NearTargetDoesNotInheritFarTargetFiltersOrPidHistory)
{
  PIDBasedPlanner planner(node_, common_, rss_);
  PIDBasedPlanner fresh(node_, common_, rss_);
  for (int i = 0; i < 5; ++i) EXPECT_GT(cycle(planner, obstacle(1, 150)), 20);
  const double switched = cycle(planner, obstacle(2, 20));
  const double clean = cycle(fresh, obstacle(2, 20));
  const auto debug = PIDBasedPlannerTestPeer::debug(planner);
  EXPECT_FLOAT_EQ(debug.at(3), debug.at(4));  // Distance raw == filtered.
  EXPECT_FLOAT_EQ(debug.at(8), debug.at(9));  // Unsafe error is not masked.
  EXPECT_LT(debug.at(9), 0);
  EXPECT_NEAR(switched, clean, 1e-9);
  EXPECT_LT(switched, 20);
}

TEST_F(PIDTargetSwitch, ReturningTargetDoesNotReuseItsEarlierHistory)
{
  PIDBasedPlanner planner(node_, common_, rss_);
  PIDBasedPlanner fresh(node_, common_, rss_);
  cycle(planner, obstacle(1, 150));
  cycle(planner, obstacle(2, 20));
  EXPECT_NEAR(cycle(planner, obstacle(1, 100)), cycle(fresh, obstacle(1, 100)), 1e-9);
}

TEST_F(PIDTargetSwitch, SameTargetKeepsFilteringBetweenCycles)
{
  PIDBasedPlanner planner(node_, common_, rss_);
  cycle(planner, obstacle(1, 100));
  cycle(planner, obstacle(1, 98));
  const auto debug = PIDBasedPlannerTestPeer::debug(planner);
  EXPECT_NEAR(debug.at(3), 98, 1e-5);
  EXPECT_NEAR(debug.at(4), 99, 1e-5);
}

TEST_F(PIDTargetSwitch, MissingTargetClearsHistoryBeforeSameUuidReturns)
{
  PIDBasedPlanner planner(node_, common_, rss_);
  PIDBasedPlanner fresh(node_, common_, rss_);
  for (int i = 0; i < 5; ++i) cycle(planner, obstacle(1, 150));
  EXPECT_FALSE(PIDBasedPlannerTestPeer::select(planner, trajectory_, data_, {}));
  EXPECT_NEAR(cycle(planner, obstacle(1, 20)), cycle(fresh, obstacle(1, 20)), 1e-9);
}

TEST(PIDControllerState, ResetClearsIntegralAndDerivativeWithoutChangingGains)
{
  PIDController pid(10, 1, 2);
  pid.calc(5);
  pid.calc(10);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.calc(-1), -11);
  EXPECT_DOUBLE_EQ(pid.getKp(), 10);
  EXPECT_DOUBLE_EQ(pid.getKi(), 1);
  EXPECT_DOUBLE_EQ(pid.getKd(), 2);
}
}  // namespace
}  // namespace autoware::motion_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_pid_target_switch");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
