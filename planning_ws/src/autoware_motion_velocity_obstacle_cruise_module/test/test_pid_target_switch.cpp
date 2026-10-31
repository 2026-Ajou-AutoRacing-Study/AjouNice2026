// Copyright 2026 AjouNice contributors
// Licensed under the Apache License, Version 2.0.
#include "pid_based_planner/pid_based_planner.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <stdexcept>
#include <utility>

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
    node_.deleteParam(prefix + "target_distance_scale");
    node_.setParam(prefix + "min_accel_during_cruise", -6.0);
    node_.setParam(prefix + "use_velocity_limit_based_planner", true);
    node_.setParam(prefix + "min_cruise_target_vel", 0.0);
    node_.setParam(prefix + "time_to_evaluate_rss", 0.0);
    node_.deleteParam(prefix + "rss_prediction");
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

TEST_F(PIDTargetSwitch, ScaledDistanceKeepsMinimumMarginAndDetectsTooCloseLead)
{
  for (const double scale : {1.0, 0.6}) {
    node_.setParam("obstacle_cruise/cruise_planning/pid_based_planner/target_distance_scale", scale);
    PIDBasedPlanner planner(node_, common_, rss_);
    // Closing lead, equal speeds, post-scale sub-5 m, negative RSS, and zero speed.
    for (const auto & speeds : std::vector<std::pair<double, double>>{
           {80, 40}, {40, 40}, {80, 80}, {90, 90}, {100, 100}, {0, 0}}) {
      const double ego_vel = speeds.first / 3.6;
      const double lead_vel = speeds.second / 3.6;
      data_->current_odometry.twist.twist.linear.x = ego_vel;
      const double raw_target = 5 + 2 * ego_vel + ego_vel * ego_vel / 12 -
                                lead_vel * lead_vel / 6;
      const double expected = std::max(5.0, scale * raw_target);
      for (const double gap : {50.0, 4.0}) {
        planner.resetForLongitudinalHandoff();
        const auto result = PIDBasedPlannerTestPeer::select(
          planner, trajectory_, data_, {obstacle(1, gap, lead_vel)});
        ASSERT_TRUE(result);
        EXPECT_NEAR(result->target_dist_to_obstacle, expected, 1e-9);
        EXPECT_GE(result->target_dist_to_obstacle, 5.0);
        EXPECT_NEAR(result->error_cruise_dist, gap - expected, 1e-9);
        if (gap < 5.0) EXPECT_LT(result->error_cruise_dist, 0.0);
      }
    }
  }
}

TEST_F(PIDTargetSwitch, InvalidDistanceScaleIsRejected)
{
  for (const double scale : std::vector<double>{0.0, -0.6, NAN, INFINITY}) {
    node_.setParam("obstacle_cruise/cruise_planning/pid_based_planner/target_distance_scale", scale);
    EXPECT_THROW((PIDBasedPlanner(node_, common_, rss_)), std::invalid_argument);
  }
}

TEST_F(PIDTargetSwitch, AdaptivePredictionSlowsHighClosingLeadBeforeBaselineRss)
{
  const std::string prefix = "obstacle_cruise/cruise_planning/pid_based_planner/";
  node_.setParam(prefix + "target_distance_scale", 0.4);
  PIDBasedPlanner baseline(node_, common_, rss_);
  node_.setParam(prefix + "rss_prediction/enabled", true);
  PIDBasedPlanner adaptive(node_, common_, rss_);
  data_->current_odometry.twist.twist.linear.x = 130.0 / 3.6;
  const auto lead = obstacle(1, 75, 50.0 / 3.6);
  const auto old = PIDBasedPlannerTestPeer::select(baseline, trajectory_, data_, {lead});
  const auto anticipated = PIDBasedPlannerTestPeer::select(adaptive, trajectory_, data_, {lead});
  ASSERT_TRUE(old);
  ASSERT_TRUE(anticipated);
  EXPECT_GT(old->error_cruise_dist, 0);
  EXPECT_LT(anticipated->error_cruise_dist, 0);
  EXPECT_NEAR(anticipated->dist_to_obstacle, 75, 1e-9);  // Real gap stays real.
  EXPECT_NEAR(anticipated->target_dist_to_obstacle, old->target_dist_to_obstacle, 1e-9);
  EXPECT_NEAR(anticipated->prediction.time, 1, 1e-9);
  EXPECT_NEAR(anticipated->prediction.correction, 80.0 / 3.6, 1e-9);
  EXPECT_LT(PIDBasedPlannerTestPeer::output(adaptive, data_, *anticipated).max_velocity,
    data_->current_odometry.twist.twist.linear.x);
  EXPECT_GT(PIDBasedPlannerTestPeer::output(baseline, data_, *old).max_velocity,
    data_->current_odometry.twist.twist.linear.x);
}

TEST_F(PIDTargetSwitch, CloseLeadFallsBackToBaselineWithoutLosingBrakingSign)
{
  const std::string prefix = "obstacle_cruise/cruise_planning/pid_based_planner/";
  node_.setParam(prefix + "target_distance_scale", 0.4);
  PIDBasedPlanner baseline(node_, common_, rss_);
  node_.setParam(prefix + "rss_prediction/enabled", true);
  PIDBasedPlanner adaptive(node_, common_, rss_);
  // Includes a small relative speed: a 0.5 s fixed prediction could still
  // become negative at a sufficiently narrow gap.
  for (const auto & speeds : std::vector<std::pair<double, double>>{{130, 50}, {50, 40}}) {
    data_->current_odometry.twist.twist.linear.x = speeds.first / 3.6;
    for (double gap : {0.0, -0.2, 0.05, 1.0, 2.0}) {
      baseline.resetForLongitudinalHandoff();
      adaptive.resetForLongitudinalHandoff();
      const auto lead = obstacle(1, gap, speeds.second / 3.6);
      const auto a = PIDBasedPlannerTestPeer::select(adaptive, trajectory_, data_, {lead});
      const auto b = PIDBasedPlannerTestPeer::select(baseline, trajectory_, data_, {lead});
      ASSERT_TRUE(a);
      ASSERT_TRUE(b);
      EXPECT_DOUBLE_EQ(a->prediction.correction, 0);
      EXPECT_NEAR(a->error_cruise_dist, b->error_cruise_dist, 1e-9);
      EXPECT_LT(a->error_cruise_dist, 0);
      const auto output = PIDBasedPlannerTestPeer::output(adaptive, data_, *a);
      EXPECT_TRUE(std::isfinite(output.max_velocity));
      EXPECT_GE(output.max_velocity, 0);
      EXPECT_LT(output.max_velocity, speeds.first / 3.6);
    }
  }
}

TEST_F(PIDTargetSwitch, PredictionUsesMostRestrictiveCandidateAndResetsOnTargetSwitch)
{
  node_.setParam("obstacle_cruise/cruise_planning/pid_based_planner/rss_prediction/enabled", true);
  PIDBasedPlanner planner(node_, common_, rss_);
  PIDBasedPlanner fresh(node_, common_, rss_);
  data_->current_odometry.twist.twist.linear.x = 130.0 / 3.6;
  for (int i = 0; i < 5; ++i) cycle(planner, obstacle(1, 200, 130.0 / 3.6));
  const auto close = obstacle(1, 60, 130.0 / 3.6);
  const auto closing = obstacle(2, 75, 50.0 / 3.6);
  const auto selected = PIDBasedPlannerTestPeer::select(
    planner, trajectory_, data_, {close, closing});
  ASSERT_TRUE(selected);
  EXPECT_EQ(selected->obstacle.uuid.uuid[15], 2);
  EXPECT_NEAR(selected->dist_to_obstacle, 75, 1e-9);
  const auto clean = PIDBasedPlannerTestPeer::select(
    fresh, trajectory_, data_, {closing, close});
  ASSERT_TRUE(clean);
  EXPECT_EQ(clean->obstacle.uuid.uuid[15], 2);
  EXPECT_NEAR(PIDBasedPlannerTestPeer::output(planner, data_, *selected).max_velocity,
    PIDBasedPlannerTestPeer::output(fresh, data_, *clean).max_velocity, 1e-9);
}

TEST_F(PIDTargetSwitch, LowClosingAndOpeningTargetsRetainBaselineOutput)
{
  const std::string prefix = "obstacle_cruise/cruise_planning/pid_based_planner/";
  node_.setParam(prefix + "target_distance_scale", 0.4);
  PIDBasedPlanner baseline(node_, common_, rss_);
  node_.setParam(prefix + "rss_prediction/enabled", true);
  PIDBasedPlanner adaptive(node_, common_, rss_);
  data_->current_odometry.twist.twist.linear.x = 80.0 / 3.6;
  for (double closing : {-20.0, 0.0, 10.0, 29.9, 30.0}) {
    baseline.resetForLongitudinalHandoff();
    adaptive.resetForLongitudinalHandoff();
    const auto lead = obstacle(1, 40, (80.0 - closing) / 3.6);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(cycle(adaptive, lead), cycle(baseline, lead), 1e-9);
    }
  }
}

TEST_F(PIDTargetSwitch, RuntimeThreeMetreFloorStillBrakesBelowTarget)
{
  rss_.safe_distance_margin = 3.0;
  node_.setParam("obstacle_cruise/cruise_planning/pid_based_planner/target_distance_scale", 0.4);
  node_.setParam("obstacle_cruise/cruise_planning/pid_based_planner/rss_prediction/enabled", true);
  PIDBasedPlanner planner(node_, common_, rss_);
  data_->current_odometry.twist.twist.linear.x = 80.0 / 3.6;
  const auto selected = PIDBasedPlannerTestPeer::select(
    planner, trajectory_, data_, {obstacle(1, 2, 80.0 / 3.6)});
  ASSERT_TRUE(selected);
  EXPECT_DOUBLE_EQ(selected->target_dist_to_obstacle, 3.0);
  EXPECT_NEAR(selected->error_cruise_dist, -1.0, 1e-9);
  EXPECT_DOUBLE_EQ(selected->prediction.correction, 0);
  EXPECT_LT(PIDBasedPlannerTestPeer::output(planner, data_, *selected).max_velocity, 80.0 / 3.6);
}

TEST_F(PIDTargetSwitch, PredictionUpdateIsValidatedBeforeReplacingPolicy)
{
  node_.setParam("obstacle_cruise/cruise_planning/pid_based_planner/target_distance_scale", 0.4);
  PIDBasedPlanner planner(node_, common_, rss_);
  data_->current_odometry.twist.twist.linear.x = 130.0 / 3.6;
  const auto lead = obstacle(1, 75, 50.0 / 3.6);
  const std::string prefix = "obstacle_cruise.cruise_planning.pid_based_planner.rss_prediction.";
  ParameterMap update;
  update[prefix + "enabled"] = XmlRpc::XmlRpcValue(true);
  planner.updateParameters(update);
  auto result = PIDBasedPlannerTestPeer::select(planner, trajectory_, data_, {lead});
  ASSERT_TRUE(result);
  EXPECT_NEAR(result->prediction.time, 1, 1e-9);
  update[prefix + "full_relative_speed_kph"] = XmlRpc::XmlRpcValue(20.0);
  EXPECT_THROW(planner.updateParameters(update), std::invalid_argument);
  result = PIDBasedPlannerTestPeer::select(planner, trajectory_, data_, {lead});
  ASSERT_TRUE(result);
  EXPECT_NEAR(result->prediction.time, 1, 1e-9);  // Previous valid policy survived.
}

TEST(RSSPrediction, RelativeSpeedRampAndOpeningLead)
{
  RSSPredictionParameters p;
  p.enabled = true;
  for (const auto & example : std::vector<std::pair<double, double>>{
      {-20, 0}, {0, 0}, {10, 0}, {30, 0}, {40, 0.2}, {50, 0.4},
      {60, 0.6}, {70, 0.8}, {80, 1}, {100, 1}}) {
    const auto result = evaluate_rss_prediction(200, example.first / 3.6, 5, p);
    EXPECT_NEAR(result.time, example.second, 1e-9);
    EXPECT_NEAR(result.correction,
      std::max(0.0, example.first) / 3.6 * example.second, 1e-9);
  }
}

TEST(RSSPrediction, NearZeroProjectionFadesContinuouslyWithoutDivisionByProjection)
{
  RSSPredictionParameters p;
  p.enabled = true;
  constexpr double closing = 80.0 / 3.6;
  double previous = 0;
  for (double distance = -1; distance < 60; distance += 0.01) {
    const auto prediction = evaluate_rss_prediction(distance, closing, 0, p);
    EXPECT_GE(prediction.correction, 0);
    EXPECT_LE(prediction.correction, closing);
    EXPECT_LT(std::abs(prediction.correction - previous), 0.016);
    if (distance <= closing) EXPECT_DOUBLE_EQ(prediction.correction, 0);
    const double error = distance - 65 - prediction.correction;
    const double normalized = normalize_cruise_distance_error(error, distance);
    EXPECT_TRUE(std::isfinite(normalized));
    EXPECT_LT(normalized, 0);  // Below RSS remains braking, including signed gaps <= 0.
    previous = prediction.correction;
  }
  for (double boundary : {closing, 2 * closing}) {
    const auto lhs = evaluate_rss_prediction(boundary - 1e-5, closing, 0, p);
    const auto rhs = evaluate_rss_prediction(boundary + 1e-5, closing, 0, p);
    EXPECT_NEAR(lhs.correction, rhs.correction, 1e-4);
  }
}

TEST(RSSPrediction, FixedTimeAlsoRetainsPhysicalDistanceAndRejectsOpeningBoost)
{
  RSSPredictionParameters p;  // Adaptive feature disabled, fixed-time fallback.
  EXPECT_DOUBLE_EQ(evaluate_rss_prediction(100, -5, 0.5, p).correction, 0);
  EXPECT_DOUBLE_EQ(evaluate_rss_prediction(2, 10, 0.5, p).correction, 0);
  EXPECT_DOUBLE_EQ(evaluate_rss_prediction(100, 10, 0.5, p).correction, 5);
}

TEST_F(PIDTargetSwitch, InvalidPredictionParametersAreRejected)
{
  const std::string prefix = "obstacle_cruise/cruise_planning/pid_based_planner/";
  for (double value : std::vector<double>{-1.0, NAN, INFINITY}) {
    node_.setParam(prefix + "rss_prediction/max_time", value);
    EXPECT_THROW((PIDBasedPlanner(node_, common_, rss_)), std::invalid_argument);
  }
  node_.setParam(prefix + "rss_prediction/max_time", 1.0);
  node_.setParam(prefix + "rss_prediction/full_relative_speed_kph", 30.0);
  EXPECT_THROW((PIDBasedPlanner(node_, common_, rss_)), std::invalid_argument);
}

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
