#include "autoware/planning_validator/rear_collision_metric_core.hpp"

#include <gtest/gtest.h>

namespace
{
autoware::planning_validator::RearCollisionMetricInput source_fixture()
{
  autoware::planning_validator::RearCollisionMetricInput input;
  input.object_velocity = 10.0;
  input.relative_distance_with_delay_compensation = 21.0;
  input.object_moving_time = 1.0;
  input.ego_velocity = 5.0;
  input.distance_to_conflict_point = 20.0;
  input.reaction_time = 1.0;
  input.object_max_deceleration = -4.0;
  input.object_max_velocity = 20.0;
  input.ego_max_deceleration = -5.0;
  input.ego_min_velocity = 1.0;
  input.filter_min_velocity = 1.0;
  input.moving_time_threshold = 0.5;
  input.ttc_margin = 3.0;
  return input;
}
}  // namespace

TEST(RearCollisionMetricCore, MatchesSourceRssSafetyAndIgnoreRules)
{
  auto input = source_fixture();
  const auto safe = autoware::planning_validator::evaluate_rear_rss(input);
  EXPECT_DOUBLE_EQ(safe.value, 20.0);
  EXPECT_TRUE(safe.safe);
  EXPECT_FALSE(safe.ignore);

  input.relative_distance_with_delay_compensation = 20.0;
  EXPECT_FALSE(autoware::planning_validator::evaluate_rear_rss(input).safe);
  input.object_moving_time = 0.2;
  EXPECT_TRUE(autoware::planning_validator::evaluate_rear_rss(input).ignore);
}

TEST(RearCollisionMetricCore, MatchesSourceTtcSafetyAndIgnoreRules)
{
  auto input = source_fixture();
  input.object_velocity = 5.0;
  input.relative_distance_with_delay_compensation = 10.0;
  input.ego_velocity = 10.0;
  const auto safe = autoware::planning_validator::evaluate_rear_ttc(input);
  EXPECT_DOUBLE_EQ(safe.value, 4.0);
  EXPECT_TRUE(safe.safe);

  input.relative_distance_with_delay_compensation = 0.0;
  EXPECT_FALSE(autoware::planning_validator::evaluate_rear_ttc(input).safe);
  input.object_velocity = 25.0;
  EXPECT_TRUE(autoware::planning_validator::evaluate_rear_ttc(input).ignore);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
