#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__TURN_SIGNAL_TYPES_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__TURN_SIGNAL_TYPES_HPP_

#include <autoware_vehicle_msgs/HazardLightsCommand.h>
#include <autoware_vehicle_msgs/TurnIndicatorsCommand.h>
#include <geometry_msgs/Pose.h>

namespace autoware::behavior_path_planner
{
struct TurnSignalInfo
{
  TurnSignalInfo()
  {
    turn_signal.command = autoware_vehicle_msgs::TurnIndicatorsCommand::NO_COMMAND;
    hazard_signal.command = autoware_vehicle_msgs::HazardLightsCommand::NO_COMMAND;
  }
  TurnSignalInfo(const geometry_msgs::Pose & start, const geometry_msgs::Pose & end)
  : TurnSignalInfo()
  {
    desired_start_point = start;
    desired_end_point = end;
    required_start_point = start;
    required_end_point = end;
  }

  autoware_vehicle_msgs::TurnIndicatorsCommand turn_signal;
  autoware_vehicle_msgs::HazardLightsCommand hazard_signal;
  geometry_msgs::Pose desired_start_point;
  geometry_msgs::Pose desired_end_point;
  geometry_msgs::Pose required_start_point;
  geometry_msgs::Pose required_end_point;
};

struct TurnSignalDebugData
{
  TurnSignalInfo intersection_turn_signal_info;
  TurnSignalInfo behavior_turn_signal_info;
  TurnSignalInfo roundabout_turn_signal_info;
};
}  // namespace autoware::behavior_path_planner

#endif
