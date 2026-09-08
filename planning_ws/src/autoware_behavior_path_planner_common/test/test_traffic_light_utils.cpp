#include <autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp>

#include <gtest/gtest.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <limits>

namespace traffic_light = autoware::behavior_path_planner::utils::traffic_light;

TEST(TrafficLightUtils, FailsClosedForMissingRouteAndPlannerState)
{
  const lanelet::ConstLanelets no_lanes;
  const geometry_msgs::Pose pose;
  const autoware_internal_planning_msgs::PathWithLaneId path;
  EXPECT_EQ(traffic_light::getDistanceToNextTrafficLight(pose, no_lanes),
            std::numeric_limits<double>::infinity());
  EXPECT_FALSE(traffic_light::calcDistanceToRedTrafficLight(no_lanes, path, nullptr));
  EXPECT_FALSE(traffic_light::isStoppedAtRedTrafficLightWithinDistance(no_lanes, path, nullptr));
  EXPECT_FALSE(traffic_light::isTrafficSignalStop(no_lanes, nullptr));
}
