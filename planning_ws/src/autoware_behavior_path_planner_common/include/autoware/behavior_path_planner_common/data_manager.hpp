#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_

// ROS1 data-only public surface. PlannerData message ownership and output
// types are implemented below; ROS2 Node parameter declaration and
// TurnSignalDecider provider methods remain intentionally outside this header.
#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/behavior_path_planner_common/planner_data.hpp>

#include <ros/time.h>

namespace autoware::behavior_path_planner
{
struct BoolStamped { explicit BoolStamped(bool value) : data(value) {} bool data{false}; ros::Time stamp{}; };
struct ModuleNameStamped { std::string module_name{"NONE"}; ros::Time stamp{}; };
}
#endif
