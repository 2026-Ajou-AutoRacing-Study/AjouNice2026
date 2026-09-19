// Copyright 2025 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__PLANNING_EVALUATOR__MOTION_EVALUATOR_NODE_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__MOTION_EVALUATOR_NODE_HPP_

#include "autoware/planning_evaluator/metrics_calculator.hpp"
#include "autoware_utils/math/accumulator.hpp"
#include "autoware/planning_evaluator/ros1_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace planning_diagnostics
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;
using autoware_utils::Accumulator;

/**
 * @brief Node for planning evaluation
 */
class MotionEvaluatorNode : public Ros1EvaluatorNode
{
public:
  explicit MotionEvaluatorNode(const ros::NodeHandle & node);
  ~MotionEvaluatorNode();

  /**
   * @brief callback on vehicle twist message
   * @param [in] twist_msg twist message
   */
  void onOdom(const nav_msgs::Odometry::Ptr msg);

private:
  geometry_msgs::Pose getCurrentEgoPose() const;

  // ROS
  ros::Subscriber twist_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // Parameters
  bool output_metrics_;

  // Calculator
  MetricsCalculator metrics_calculator_;
  // Metrics
  std::vector<Metric> metrics_;
  VehicleInfo vehicle_info_;
  std::deque<ros::Time> stamps_;
  Trajectory accumulated_trajectory_;
};
}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__MOTION_EVALUATOR_NODE_HPP_
