#include <autoware/behavior_path_planner_common/turn_signal_decider.hpp>

#include <gtest/gtest.h>

namespace
{
autoware_internal_planning_msgs::PathWithLaneId makePath()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i = 0; i < 3; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i);
    point.point.pose.orientation.w = 1.0;
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

TEST(TurnSignalResolver, FailsClosedForEmptyPath)
{
  autoware::behavior_path_planner::TurnSignalDecider resolver;
  const autoware::behavior_path_planner::TurnSignalInfo no_signal;
  const auto command = resolver.resolve_turn_signal({}, geometry_msgs::Pose{}, 0, no_signal, no_signal, no_signal, 3.0, 1.0);
  EXPECT_EQ(command.command, autoware_vehicle_msgs::TurnIndicatorsCommand::DISABLE);
}

TEST(TurnSignalResolver, SelectsValidBehaviorSignal)
{
  autoware::behavior_path_planner::TurnSignalDecider resolver;
  const auto path = makePath();
  geometry_msgs::Pose current_pose;
  current_pose.orientation.w = 1.0;
  autoware::behavior_path_planner::TurnSignalInfo behavior;
  behavior.turn_signal.command = autoware_vehicle_msgs::TurnIndicatorsCommand::ENABLE_LEFT;
  behavior.desired_start_point = current_pose;
  behavior.required_start_point = current_pose;
  behavior.desired_end_point = path.points.back().point.pose;
  behavior.required_end_point = path.points.back().point.pose;
  const autoware::behavior_path_planner::TurnSignalInfo no_signal;
  resolver.setParameters(0.0, 0.0, 0.0, 0.0, "Disable", "Disable", false, 0.0, 0.0, 0);
  const auto command = resolver.resolve_turn_signal(path, current_pose, 0, no_signal, no_signal, behavior, 3.0, 1.0);
  EXPECT_EQ(command.command, autoware_vehicle_msgs::TurnIndicatorsCommand::ENABLE_LEFT);
}
