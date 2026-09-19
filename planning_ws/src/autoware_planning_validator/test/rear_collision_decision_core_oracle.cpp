#include "autoware/planning_validator/rear_collision_decision_core.hpp"

#include <gtest/gtest.h>

using autoware::planning_validator::RearDetectionRange;
using autoware::planning_validator::RearRangeInput;
using autoware::planning_validator::RearSafetyDebounceState;
using autoware::planning_validator::RearSafetyObject;

namespace
{
RearRangeInput source_range_fixture()
{
  RearRangeInput input;
  input.range_buffer = 5.0;
  input.ego_max_longitudinal_offset = 2.0;
  input.ego_min_longitudinal_offset = -1.0;
  input.ego_velocity = 10.0;
  input.ego_max_deceleration = -5.0;
  input.ego_min_velocity = 1.0;
  input.blind_spot_check_front = true;
  input.participant_reaction_time = 1.0;
  input.participant_max_deceleration = -4.0;
  input.participant_max_velocity = 20.0;
  input.distance_to_conflict_point = 30.0;
  input.ttc_margin = 3.0;
  return input;
}
}  // namespace

TEST(RearCollisionDecisionCore, MatchesSourceRssAndTtcDetectionRanges)
{
  const auto input = source_range_fixture();
  const RearDetectionRange rss = autoware::planning_validator::rear_range_for_rss(input);
  EXPECT_DOUBLE_EQ(rss.forward, 15.0);
  EXPECT_DOUBLE_EQ(rss.backward, 66.0);

  const RearDetectionRange ttc = autoware::planning_validator::rear_range_for_ttc(input);
  EXPECT_DOUBLE_EQ(ttc.forward, 7.0);
  EXPECT_DOUBLE_EQ(ttc.backward, 95.0);
}

TEST(RearCollisionDecisionCore, RequiresMatureNonIgnoredUnsafeObject)
{
  EXPECT_TRUE(autoware::planning_validator::rear_objects_are_safe({{0.4, false, false}}, 0.5));
  EXPECT_TRUE(autoware::planning_validator::rear_objects_are_safe({{0.5, true, false}}, 0.5));
  EXPECT_FALSE(autoware::planning_validator::rear_objects_are_safe({{0.5, false, false}}, 0.5));
}

TEST(RearCollisionDecisionCore, MatchesSourceTtcAndRssObjectFlags)
{
  autoware::planning_validator::RearObjectSafetyInput input;
  input.relative_distance_with_delay_compensation = 1.0;
  input.velocity = 4.0;
  input.moving_time = 1.0;
  input.ego_velocity = 5.0;
  input.ego_min_velocity = 1.0;
  input.ego_max_deceleration = -4.0;
  input.participant_max_deceleration = -2.0;
  input.participant_max_velocity = 5.5;
  input.participant_reaction_time = 1.2;
  input.distance_to_conflict_point = 10.0;
  input.min_velocity = 1.0;
  input.moving_time_threshold = 0.5;
  input.ttc_margin = 2.0;
  EXPECT_FALSE(autoware::planning_validator::evaluate_rear_ttc_safety(input).safe);
  EXPECT_FALSE(autoware::planning_validator::evaluate_rear_rss_safety(input).safe);
  input.moving_time = 0.0;
  EXPECT_TRUE(autoware::planning_validator::evaluate_rear_ttc_safety(input).ignore);
}

TEST(RearCollisionDecisionCore, MatchesSourceOnAndOffTimeDebounce)
{
  RearSafetyDebounceState state{10.0, 0.0};
  EXPECT_TRUE(autoware::planning_validator::apply_rear_safety_debounce(false, 11.9, 2.0, 1.0, state));
  EXPECT_FALSE(autoware::planning_validator::apply_rear_safety_debounce(false, 12.0, 2.0, 1.0, state));
  EXPECT_DOUBLE_EQ(state.last_unsafe_time, 12.0);
  EXPECT_TRUE(autoware::planning_validator::apply_rear_safety_debounce(true, 12.5, 2.0, 1.0, state));
  EXPECT_DOUBLE_EQ(state.last_safe_time, 10.0);  // strict > off buffer
  EXPECT_TRUE(autoware::planning_validator::apply_rear_safety_debounce(true, 13.1, 2.0, 1.0, state));
  EXPECT_DOUBLE_EQ(state.last_safe_time, 13.1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
