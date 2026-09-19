// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include <ros/serialization.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
struct PlannerWaypointsVector
{
  std::vector<std::vector<double>> waypoints;
  double length;
};

// Python ROS1 messages serialize their payload on the ROS1 wire (without a TCPROS length
// prefix).  The ROS2 binding used CDR; this is deliberately a wire-format migration, not a
// byte-compatible claim across ROS generations.
template <typename MessageT>
MessageT deserialize_ros1(const std::string & bytes)
{
  if (bytes.empty()) {
    throw std::runtime_error("ROS1 serialized message is empty");
  }
  std::vector<uint8_t> buffer(bytes.begin(), bytes.end());
  ros::serialization::IStream stream(buffer.data(), static_cast<uint32_t>(buffer.size()));
  MessageT message;
  ros::serialization::deserialize(stream, message);
  return message;
}

class AstarSearchPython : public freespace_planning_algorithms::AstarSearch
{
  using AstarSearch::AstarSearch;

public:
  void setMapByte(const std::string & costmap_byte)
  {
    freespace_planning_algorithms::AstarSearch::setMap(
      deserialize_ros1<nav_msgs::OccupancyGrid>(costmap_byte));
  }

  bool makePlanByte(const std::string & start_pose_byte, const std::string & goal_pose_byte)
  {
    return freespace_planning_algorithms::AstarSearch::makePlan(
      deserialize_ros1<geometry_msgs::Pose>(start_pose_byte),
      deserialize_ros1<geometry_msgs::Pose>(goal_pose_byte));
  }

  PlannerWaypointsVector getWaypointsAsVector()
  {
    freespace_planning_algorithms::PlannerWaypoints waypoints =
      freespace_planning_algorithms::AstarSearch::getWaypoints();
    PlannerWaypointsVector waypoints_vector;
    for (const auto & waypoint : waypoints.waypoints) {
      // NOTE: it was difficult to return the deserialized pose_byte and serialize the pose_byte on
      // python-side. So this function returns [*position, *quaternion] as double array
      const auto & xyz = waypoint.pose.pose.position;
      const auto & quat = waypoint.pose.pose.orientation;
      const double & is_back = waypoint.is_back;
      waypoints_vector.waypoints.push_back(
        std::vector<double>({xyz.x, xyz.y, xyz.z, quat.x, quat.y, quat.z, quat.w, is_back}));
    }
    waypoints_vector.length = waypoints.compute_length();
    return waypoints_vector;
  }

  double getDistanceToObstacle(const std::string & pose_byte)
  {
    return freespace_planning_algorithms::AstarSearch::getDistanceToObstacle(
      deserialize_ros1<geometry_msgs::Pose>(pose_byte));
  }
};

namespace py = pybind11;

// cppcheck-suppress syntaxError
PYBIND11_MODULE(autoware_freespace_planning_algorithms_pybind, p)
{
  // The Python wrapper is usable without constructing a roscpp NodeHandle.
  // ROS1 requires its time subsystem to be initialized before ros::Time::now().
  ros::Time::init();
  auto pyPlannerWaypointsVector = py::class_<PlannerWaypointsVector>(p, "PlannerWaypointsVector")
                                    .def(py::init<>())
                                    .def_readwrite("waypoints", &PlannerWaypointsVector::waypoints)
                                    .def_readwrite("length", &PlannerWaypointsVector::length);
  auto pyAstarParam =
    py::class_<freespace_planning_algorithms::AstarParam>(p, "AstarParam")
      .def(py::init<>())
      .def_readwrite("search_method", &freespace_planning_algorithms::AstarParam::search_method)
      .def_readwrite(
        "only_behind_solutions", &freespace_planning_algorithms::AstarParam::only_behind_solutions)
      .def_readwrite("use_back", &freespace_planning_algorithms::AstarParam::use_back)
      .def_readwrite(
        "adapt_expansion_distance",
        &freespace_planning_algorithms::AstarParam::adapt_expansion_distance)
      .def_readwrite(
        "expansion_distance", &freespace_planning_algorithms::AstarParam::expansion_distance)
      .def_readwrite(
        "near_goal_distance", &freespace_planning_algorithms::AstarParam::near_goal_distance)
      .def_readwrite(
        "distance_heuristic_weight",
        &freespace_planning_algorithms::AstarParam::distance_heuristic_weight)
      .def_readwrite(
        "smoothness_weight", &freespace_planning_algorithms::AstarParam::smoothness_weight)
      .def_readwrite(
        "obstacle_distance_weight",
        &freespace_planning_algorithms::AstarParam::obstacle_distance_weight)
      .def_readwrite(
        "goal_lat_distance_weight",
        &freespace_planning_algorithms::AstarParam::goal_lat_distance_weight);
  auto pyPlannerCommonParam =
    py::class_<freespace_planning_algorithms::PlannerCommonParam>(p, "PlannerCommonParam")
      .def(py::init<>())
      .def_readwrite("time_limit", &freespace_planning_algorithms::PlannerCommonParam::time_limit)
      .def_readwrite("theta_size", &freespace_planning_algorithms::PlannerCommonParam::theta_size)
      .def_readwrite(
        "max_turning_ratio", &freespace_planning_algorithms::PlannerCommonParam::max_turning_ratio)
      .def_readwrite(
        "turning_steps", &freespace_planning_algorithms::PlannerCommonParam::turning_steps)
      .def_readwrite(
        "curve_weight", &freespace_planning_algorithms::PlannerCommonParam::curve_weight)
      .def_readwrite(
        "reverse_weight", &freespace_planning_algorithms::PlannerCommonParam::reverse_weight)
      .def_readwrite(
        "direction_change_weight",
        &freespace_planning_algorithms::PlannerCommonParam::direction_change_weight)
      .def_readwrite(
        "lateral_goal_range",
        &freespace_planning_algorithms::PlannerCommonParam::lateral_goal_range)
      .def_readwrite(
        "longitudinal_goal_range",
        &freespace_planning_algorithms::PlannerCommonParam::longitudinal_goal_range)
      .def_readwrite(
        "angle_goal_range", &freespace_planning_algorithms::PlannerCommonParam::angle_goal_range)
      .def_readwrite(
        "obstacle_threshold",
        &freespace_planning_algorithms::PlannerCommonParam::obstacle_threshold);
  auto pyVehicleShape =
    py::class_<freespace_planning_algorithms::VehicleShape>(p, "VehicleShape")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double>())
      .def("setMinMaxDimension", &freespace_planning_algorithms::VehicleShape::setMinMaxDimension)
      .def_readwrite("length", &freespace_planning_algorithms::VehicleShape::length)
      .def_readwrite("width", &freespace_planning_algorithms::VehicleShape::width)
      .def_readwrite("base_length", &freespace_planning_algorithms::VehicleShape::base_length)
      .def_readwrite("max_steering", &freespace_planning_algorithms::VehicleShape::max_steering)
      .def_readwrite("base2back", &freespace_planning_algorithms::VehicleShape::base2back)
      .def_readwrite("min_dimension", &freespace_planning_algorithms::VehicleShape::min_dimension);

  auto pyAbstractPlanningAlgorithm =
    py::class_<freespace_planning_algorithms::AbstractPlanningAlgorithm>(
      p, "AbstractPlanningAlgorithm");
  auto pyAstarSearchCpp = py::class_<
    freespace_planning_algorithms::AstarSearch,
    freespace_planning_algorithms::AbstractPlanningAlgorithm>(p, "AstarSearchCpp");
  py::class_<AstarSearchPython, freespace_planning_algorithms::AstarSearch>(p, "AstarSearch")
    .def(
      py::init<
        freespace_planning_algorithms::PlannerCommonParam &,
        freespace_planning_algorithms::VehicleShape &,
        freespace_planning_algorithms::AstarParam &>())
    .def("setMap", &AstarSearchPython::setMapByte)
    .def("makePlan", &AstarSearchPython::makePlanByte)
    .def("getWaypoints", &AstarSearchPython::getWaypointsAsVector)
    .def("getDistanceToObstacle", &AstarSearchPython::getDistanceToObstacle);
}
}  // namespace autoware::freespace_planning_algorithms
