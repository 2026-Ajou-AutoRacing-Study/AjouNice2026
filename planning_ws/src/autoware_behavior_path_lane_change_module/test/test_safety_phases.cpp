// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_lane_change_module/utils/safety_check.hpp>

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <gtest/gtest.h>

#include <limits>

namespace lc = autoware::behavior_path_planner::utils::lane_change;
namespace check = autoware::behavior_path_planner::utils::path_safety_checker;

namespace
{
geometry_msgs::Pose pose(double x, double y = 0.0)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation.w = 1.0;
  return p;
}

struct Fixture
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  std::vector<check::PoseWithVelocityStamped> prediction;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  check::RSSparams prepare, execution;
  check::CollisionCheckDebugMap debug;

  Fixture()
  {
    for (double x = 0; x <= 40; x += 1.0) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose = pose(x);
      path.points.push_back(point);
    }
    for (double t = 0; t <= 8; t += 1.0) prediction.emplace_back(t, pose(t * 2), 2.0);
    vehicle.wheel_base_m = 1.0;
    vehicle.front_overhang_m = vehicle.rear_overhang_m = 1.0;
    vehicle.wheel_tread_m = vehicle.vehicle_width_m = 2.0;
    vehicle.max_longitudinal_offset_m = 2.0;
    prepare.extended_polygon_policy = "along_path";
    prepare.front_vehicle_deceleration = prepare.rear_vehicle_deceleration = -6.0;
    prepare.rear_vehicle_reaction_time = 1.0;
    prepare.rear_vehicle_safety_time_margin = 0.8;
    prepare.lateral_distance_max_threshold = 0.5;
    prepare.longitudinal_distance_min_threshold = 1.0;
    execution = prepare;
    execution.extended_polygon_policy = "rectangle";
    execution.rear_vehicle_reaction_time = 2.0;
    execution.rear_vehicle_safety_time_margin = 1.0;
    execution.lateral_distance_max_threshold = 2.0;
    execution.longitudinal_distance_min_threshold = 3.0;
  }

  check::ExtendedPredictedObject object(double time, double lateral = 3.0, double ahead = 8.0)
  {
    check::ExtendedPredictedObject obj;
    obj.uuid.uuid[0] = 1;
    obj.initial_pose = pose(time * 2 + ahead, lateral);
    check::PredictedPathWithPolygon predicted;
    predicted.confidence = 1.0;
    predicted.path.emplace_back(time, obj.initial_pose, 2.0,
      autoware_utils::to_footprint(obj.initial_pose, 1.0, 1.0, 1.0));
    obj.predicted_paths.push_back(predicted);
    return obj;
  }

  lc::PhaseSafetyResult run(const check::ExtendedPredictedObjects & objects, double split = 4.0)
  {
    return lc::check_safety_by_phase(path, prediction, vehicle, objects, debug,
      prepare, execution, split, 3.1416, true);
  }
};
}

TEST(LaneChangeSafetyPhases, PreparationIsNotRejectedByExecutionMargin)
{
  Fixture f;
  const auto object = f.object(1.0);
  EXPECT_FALSE(check::check_safety_with_rss(
    f.path, f.prediction, f.vehicle, {object}, f.execution, true, 1.0, 3.1416));
  EXPECT_TRUE(f.run({object}).safe);
}

TEST(LaneChangeSafetyPhases, ExecutionIsNotRejectedByPreparationPolicy)
{
  Fixture f;
  f.prepare.lateral_distance_max_threshold = 2.0;
  f.prepare.longitudinal_distance_min_threshold = 6.0;
  f.execution.lateral_distance_max_threshold = 0.5;
  const auto object = f.object(5.0);
  EXPECT_FALSE(check::check_safety_with_rss(
    f.path, f.prediction, f.vehicle, {object}, f.prepare, true, 1.0, 3.1416));
  EXPECT_TRUE(f.run({object}).safe);
}

TEST(LaneChangeSafetyPhases, TransitionTimestampBelongsToExecution)
{
  Fixture f;
  const auto result = f.run({f.object(4.0)});
  EXPECT_FALSE(result.safe);
  EXPECT_STREQ(result.stage, "execution");
  EXPECT_DOUBLE_EQ(result.trace.time, 4.0);
  EXPECT_EQ(f.debug.begin()->second.unsafe_reason, "overlap_extended_polygon");
}

TEST(LaneChangeSafetyPhases, ZeroPreparationUsesExecutionImmediately)
{
  Fixture f;
  const auto result = f.run({f.object(0.0)}, 0.0);
  EXPECT_FALSE(result.safe);
  EXPECT_STREQ(result.stage, "execution");
  EXPECT_DOUBLE_EQ(result.trace.time, 0.0);
}

TEST(LaneChangeSafetyPhases, BodyOverlapRemainsUnsafeInBothPhases)
{
  for (double time : {1.0, 5.0}) {
    Fixture f;
    const auto result = f.run({f.object(time, 0.0, 0.0)});
    EXPECT_FALSE(result.safe);
    EXPECT_EQ(f.debug.begin()->second.unsafe_reason, "overlap_polygon");
    EXPECT_DOUBLE_EQ(result.trace.time, time);
    EXPECT_STREQ(result.stage, time < 4.0 ? "prepare" : "execution");
  }
}

TEST(LaneChangeSafetyPhases, ChecksEveryTargetObject)
{
  Fixture f;
  auto clear = f.object(1.0);
  auto unsafe = f.object(5.0);
  unsafe.uuid.uuid[0] = 2;
  const auto result = f.run({clear, unsafe});
  EXPECT_FALSE(result.safe);
  EXPECT_FALSE(f.debug.at(unsafe.uuid.uuid).is_safe);
}

TEST(LaneChangeSafetyPhases, HypothesisThatNeverReachesStartUsesPreparationThroughout)
{
  Fixture f;
  EXPECT_TRUE(f.run({f.object(5.0)}, std::numeric_limits<double>::infinity()).safe);
  EXPECT_FALSE(f.run({f.object(5.0, 0.0, 0.0)}, std::numeric_limits<double>::infinity()).safe);
}

TEST(LaneChangeSafetyPhases, InvalidPhaseTimeFailsClosed)
{
  Fixture f;
  EXPECT_FALSE(f.run({}, -1.0).safe);
  EXPECT_FALSE(f.run({}, std::numeric_limits<double>::quiet_NaN()).safe);
}

TEST(LaneChangeSafetyPhases, ApprovedManeuverUsesRemainingTimeInsteadOfOriginalDuration)
{
  Fixture f;
  const std::vector<check::PoseWithVelocityStamped> prediction{
    {0.0, pose(4.0), 2.0}, {1.0, pose(6.0), 2.0}, {2.0, pose(8.0), 2.0}};
  const auto remaining = lc::remaining_prepare_time(f.path, prediction, pose(8.0));
  ASSERT_TRUE(remaining);
  EXPECT_DOUBLE_EQ(*remaining, 2.0);
  const auto result = lc::check_safety_by_phase(f.path, prediction, f.vehicle,
    {f.object(2.0)}, f.debug, f.prepare, f.execution, *remaining, 3.1416, true);
  EXPECT_FALSE(result.safe);
  EXPECT_STREQ(result.stage, "execution");
}

TEST(LaneChangeSafetyPhases, ApprovedManeuverAlreadyPastStartHasNoPreparation)
{
  Fixture f;
  const auto remaining = lc::remaining_prepare_time(f.path, {{0.0, pose(9.0), 2.0}}, pose(8.0));
  ASSERT_TRUE(remaining);
  EXPECT_DOUBLE_EQ(*remaining, 0.0);
}

TEST(LaneChangeSafetyPhases, StoppedHypothesisDoesNotRestartOriginalPreparationClock)
{
  Fixture f;
  const auto remaining = lc::remaining_prepare_time(f.path,
    {{0.0, pose(4.0), 2.0}, {1.0, pose(5.0), 0.0}, {2.0, pose(5.0), 0.0}}, pose(8.0));
  ASSERT_TRUE(remaining);
  EXPECT_EQ(*remaining, std::numeric_limits<double>::infinity());
}

TEST(LaneChangeSafetyPhases, TransitionTimeFollowsCurvedPathArc)
{
  Fixture f;
  f.path.points.clear();
  for (const auto & p : {pose(0.0), pose(10.0), pose(10.0, 10.0)}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose = p;
    f.path.points.push_back(point);
  }
  const auto remaining = lc::remaining_prepare_time(f.path,
    {{0.0, pose(5.0), 5.0}, {1.0, pose(10.0), 5.0}, {2.0, pose(10.0, 10.0), 5.0}},
    pose(10.0, 5.0));
  ASSERT_TRUE(remaining);
  EXPECT_DOUBLE_EQ(*remaining, 1.5);
}

TEST(LaneChangeSafetyPhases, InvalidPredictionCannotProduceAPhaseBoundary)
{
  Fixture f;
  EXPECT_FALSE(lc::remaining_prepare_time(f.path, {}, pose(8.0)));
  EXPECT_FALSE(lc::remaining_prepare_time(f.path,
    {{0.0, pose(0.0), 2.0}, {0.0, pose(1.0), 2.0}}, pose(8.0)));
}
