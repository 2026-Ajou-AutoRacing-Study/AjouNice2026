#include "autoware/planning_validator_intersection_collision_checker/collision_decision_core_ros1.hpp"

#include <gtest/gtest.h>

namespace icc = autoware::planning_validator::intersection_collision_checker_ros1;

icc::Params params_for_decision()
{
  icc::Params params{};
  params.filter.min_velocity = 1.0;
  params.close_time_th = 2.0;
  params.ego_deceleration = 2.0;
  params.ttc_threshold = 0.5;
  return params;
}

TEST(CollisionDecisionCore, UnreliableObjectCannotTriggerCollision)
{
  const auto params = params_for_decision();
  EXPECT_FALSE(icc::is_colliding({false, true, 0.1}, {0.0, 1.0}, 3.0, params));
}

TEST(CollisionDecisionCore, CloseObjectBeforeEgoArrivalTriggersCollision)
{
  const auto params = params_for_decision();
  EXPECT_TRUE(icc::is_colliding({true, false, 0.0}, {1.0, 2.0}, 3.0, params));
  EXPECT_FALSE(icc::is_colliding({true, false, 0.0}, {2.0, 3.0}, 3.0, params));
}

TEST(CollisionDecisionCore, MovingObjectRequiresOverlapWithinThreshold)
{
  const auto params = params_for_decision();
  EXPECT_TRUE(icc::is_colliding({true, true, 1.3}, {1.0, 2.0}, 3.0, params));
  EXPECT_FALSE(icc::is_colliding({true, true, 2.6}, {1.0, 2.0}, 3.0, params));
}
