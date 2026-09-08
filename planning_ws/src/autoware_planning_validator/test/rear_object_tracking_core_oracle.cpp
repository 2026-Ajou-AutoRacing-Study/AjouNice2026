#include "autoware/planning_validator/rear_object_tracking_core.hpp"

#include <gtest/gtest.h>

namespace
{
using autoware::planning_validator::RearObjectTrackingConfig;
using autoware::planning_validator::RearObjectTrackingCore;
using autoware::planning_validator::RearObjectTrackingState;

RearObjectTrackingConfig source_config()
{
  RearObjectTrackingConfig config;
  config.latency = 0.2;
  config.observation_time = 0.5;
  config.max_acceleration = 2.0;
  config.min_velocity = 1.0;
  return config;
}

RearObjectTrackingState observation(
  const int64_t lane_id, const double update_time, const double distance)
{
  RearObjectTrackingState state;
  state.lane_id = lane_id;
  state.last_update_time = update_time;
  state.last_stop_time = update_time;
  state.relative_distance = distance;
  return state;
}
}  // namespace

TEST(RearObjectTrackingCore, TracksVelocityDelayAndMovingTimeWithSourceLowpass)
{
  RearObjectTrackingCore core(source_config());
  auto first = observation(10, 0.0, 10.0);
  core.update(first, 3.0, 0.0, {});

  auto second = observation(10, 1.0, 8.0);
  core.update(second, 3.0, 1.0, {});
  EXPECT_DOUBLE_EQ(second.velocity, 2.5);  // raw=5, lowpass(raw, 0, 0.5)
  EXPECT_DOUBLE_EQ(second.tracking_duration, 1.0);
  EXPECT_DOUBLE_EQ(second.relative_distance_with_delay_compensation, 7.5);
  EXPECT_DOUBLE_EQ(second.moving_time, 1.0);
  EXPECT_DOUBLE_EQ(second.last_stop_time, 0.0);
}

TEST(RearObjectTrackingCore, PreservesSourceZeroDeltaAndOutlierBranches)
{
  RearObjectTrackingCore core(source_config());
  auto first = observation(10, 0.0, 10.0);
  first.velocity = 4.0;
  first.tracking_duration = 1.0;
  first.moving_time = 2.0;
  first.last_stop_time = 0.0;
  core.update(first, 0.0, 0.0, {});

  auto zero_dt = observation(10, 0.0, 9.0);
  core.update(zero_dt, 0.0, 7.0, {});
  EXPECT_DOUBLE_EQ(zero_dt.velocity, 4.0);
  EXPECT_DOUBLE_EQ(zero_dt.tracking_duration, 1.0);
  EXPECT_DOUBLE_EQ(zero_dt.moving_time, 2.0);

  auto outlier = observation(10, 1.0, -10.0);
  core.update(outlier, 0.0, 8.0, {});
  EXPECT_DOUBLE_EQ(outlier.velocity, 4.0);
  EXPECT_DOUBLE_EQ(outlier.tracking_duration, 1.0);
  EXPECT_EQ(outlier.detail, "the estimated velocity may be an outlier.");
}

TEST(RearObjectTrackingCore, AdoptsFirstKnownPredecessorAndResetsStoppedTime)
{
  RearObjectTrackingCore core(source_config());
  auto predecessor = observation(1, 0.0, 10.0);
  predecessor.velocity = 0.0;
  predecessor.tracking_duration = 1.0;
  predecessor.moving_time = 9.0;
  core.update(predecessor, 0.0, 0.0, {});

  auto successor = observation(2, 1.0, 10.0);
  core.update(successor, 0.0, 42.0, {999, 1});
  EXPECT_DOUBLE_EQ(successor.velocity, 0.0);
  EXPECT_DOUBLE_EQ(successor.tracking_duration, 2.0);
  EXPECT_DOUBLE_EQ(successor.moving_time, 0.0);
  EXPECT_DOUBLE_EQ(successor.last_stop_time, 42.0);
  ASSERT_EQ(core.history().count(2), 1U);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
