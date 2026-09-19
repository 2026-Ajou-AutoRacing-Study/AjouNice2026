// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_
#define AUTOWARE__PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_

#include <autoware/planning_topic_converter/converter_base.hpp>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>

namespace autoware
{
namespace planning_topic_converter
{

class PathToTrajectory
  : public ConverterBase<autoware_planning_msgs::Path, autoware_planning_msgs::Trajectory>
{
public:
  PathToTrajectory(ros::NodeHandle & node, ros::NodeHandle & private_node);

private:
  void process(const autoware_planning_msgs::Path::ConstPtr & msg) override;
};

}  // namespace planning_topic_converter
}  // namespace autoware

#endif
