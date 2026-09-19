// Copyright 2023 TIER IV, Inc.
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


#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <limits>
namespace autoware::behavior_path_planner::utils {
using Point = geometry_msgs::Point;
using Pose = geometry_msgs::Pose;
std::optional<double> getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const double vehicle_width, const double base_link2front,
  const double base_link2rear, const Pose & vehicle_pose, const bool left_side)
{
  // Depending on which side is selected, calculate the transformed coordinates of the front and
  // rear vehicle corners
  Point rear_corner_point, front_corner_point;
  if (left_side) {
    Point front_left, rear_left;
    rear_left.x = -base_link2rear;
    rear_left.y = vehicle_width / 2;
    front_left.x = base_link2front;
    front_left.y = vehicle_width / 2;
    rear_corner_point = autoware_utils::transform_point(rear_left, vehicle_pose);
    front_corner_point = autoware_utils::transform_point(front_left, vehicle_pose);
  } else {
    Point front_right, rear_right;
    rear_right.x = -base_link2rear;
    rear_right.y = -vehicle_width / 2;
    front_right.x = base_link2front;
    front_right.y = -vehicle_width / 2;
    rear_corner_point = autoware_utils::transform_point(rear_right, vehicle_pose);
    front_corner_point = autoware_utils::transform_point(front_right, vehicle_pose);
  }

  const auto combined_lane_opt =
    autoware::experimental::lanelet2_utils::combine_lanelets_shape(lanelets);
  if (!combined_lane_opt.has_value()) {
    // if lanelets is empty, return nullopt
    return std::nullopt;
  }
  const auto & combined_lane = combined_lane_opt.value();
  const auto & bound_line_2d = left_side ? lanelet::utils::to2D(combined_lane.leftBound3d())
                                         : lanelet::utils::to2D(combined_lane.rightBound3d());

  // Find the closest bound segment that contains the corner point in the X-direction
  // and calculate the lateral distance from that segment.
  const auto calcLateralDistanceFromBound =
    [&](const Point & vehicle_corner_point) -> std::optional<std::pair<double, size_t>> {
    Pose vehicle_corner_pose{};
    vehicle_corner_pose.position = vehicle_corner_point;
    vehicle_corner_pose.orientation = vehicle_pose.orientation;

    std::optional<std::pair<double, size_t>> lateral_distance_with_idx{};

    // Euclidean distance to find the closest segment containing the corner point.
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < bound_line_2d.size() - 1; i++) {
      const Point p1 = experimental::lanelet2_utils::to_ros(bound_line_2d[i]);
      const Point p2 = experimental::lanelet2_utils::to_ros(bound_line_2d[i + 1]);

      const Point inverse_p1 = autoware_utils::inverse_transform_point(p1, vehicle_corner_pose);
      const Point inverse_p2 = autoware_utils::inverse_transform_point(p2, vehicle_corner_pose);
      const double dx_p1 = inverse_p1.x;
      const double dx_p2 = inverse_p2.x;
      const double dy_p1 = inverse_p1.y;
      const double dy_p2 = inverse_p2.y;

      // Calculate the Euclidean distances between vehicle's corner and the current and next points.
      const double distance1 = autoware_utils::calc_distance2d(p1, vehicle_corner_point);
      const double distance2 = autoware_utils::calc_distance2d(p2, vehicle_corner_point);

      // If one of the bound points is behind and the other is in front of the vehicle corner point
      // and any of these points is closer than the current minimum distance,
      // then update minimum distance, lateral distance and the segment index.
      if (dx_p1 < 0 && dx_p2 > 0 && (distance1 < min_distance || distance2 < min_distance)) {
        min_distance = std::min(distance1, distance2);
        // Update lateral distance using the formula derived from similar triangles in the lateral
        // cross-section view.
        lateral_distance_with_idx =
          std::make_pair(-1.0 * (dy_p1 * dx_p2 + dy_p2 * -dx_p1) / (dx_p2 - dx_p1), i);
      }
    }
    if (lateral_distance_with_idx) {
      return lateral_distance_with_idx;
    }
    return std::nullopt;
  };

  // Calculate the lateral distance for both the rear and front corners of the vehicle.
  const std::optional<std::pair<double, size_t>> rear_lateral_distance_with_idx =
    calcLateralDistanceFromBound(rear_corner_point);
  const std::optional<std::pair<double, size_t>> front_lateral_distance_with_idx =
    calcLateralDistanceFromBound(front_corner_point);

  // If no closest bound segment was found for both corners, return an empty optional.
  if (!rear_lateral_distance_with_idx && !front_lateral_distance_with_idx) {
    return {};
  }
  // If only one of them found the closest bound, return the found lateral distance.
  if (!rear_lateral_distance_with_idx) {
    return front_lateral_distance_with_idx.value().first;
  } else if (!front_lateral_distance_with_idx) {
    return rear_lateral_distance_with_idx.value().first;
  }
  // If both corners found their closest bound, return the maximum (for left side) or the minimum
  // (for right side) lateral distance.
  double lateral_distance = left_side ? std::max(
                                          rear_lateral_distance_with_idx.value().first,
                                          front_lateral_distance_with_idx.value().first)
                                      : std::min(
                                          rear_lateral_distance_with_idx.value().first,
                                          front_lateral_distance_with_idx.value().first);

  // Iterate through all segments between the segments closest to the rear and front corners.
  // Update the lateral distance in case any of these inner segments are closer to the vehicle.
  for (size_t i = rear_lateral_distance_with_idx.value().second + 1;
       i < front_lateral_distance_with_idx.value().second; i++) {
    Pose bound_pose;
    bound_pose.position = experimental::lanelet2_utils::to_ros(bound_line_2d[i]);
    bound_pose.orientation = vehicle_pose.orientation;

    const Point inverse_rear_point =
      autoware_utils::inverse_transform_point(rear_corner_point, bound_pose);
    const Point inverse_front_point =
      autoware_utils::inverse_transform_point(front_corner_point, bound_pose);
    const double dx_rear = inverse_rear_point.x;
    const double dx_front = inverse_front_point.x;
    const double dy_rear = inverse_rear_point.y;
    const double dy_front = inverse_front_point.y;

    const double current_lateral_distance =
      (dy_rear * dx_front + dy_front * -dx_rear) / (dx_front - dx_rear);
    lateral_distance = left_side ? std::max(lateral_distance, current_lateral_distance)
                                 : std::min(lateral_distance, current_lateral_distance);
  }

  return lateral_distance;
}

lanelet::ConstLanelets getExtendedCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  const double backward_length, const double forward_length, const bool forward_only_in_route)
{
  auto lanes = getCurrentLanesFromPath(path, planner_data);

  if (lanes.empty()) return lanes;
  const auto start_lane = lanes.front();
  double forward_length_sum = 0.0;
  double backward_length_sum = 0.0;

  while (backward_length_sum < backward_length) {
    auto extended_lanes = extendPrevLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for backward_length,
    // the extending process will not finish.
    if (extended_lanes.front().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      backward_length_sum += lanelet::geometry::length2d(extended_lanes.front());
    } else {
      break;  // no more previous lanes to add
    }
    lanes = extended_lanes;
  }

  while (forward_length_sum < forward_length) {
    auto extended_lanes = extendNextLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for forward_length,
    // the extending process will not finish.
    if (extended_lanes.back().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      forward_length_sum += lanelet::geometry::length2d(extended_lanes.back());
    } else {
      break;  // no more next lanes to add
    }

    // stop extending when the lane outside of the route is reached
    // if forward_length is a very large value, set it to true,
    // as it may continue to extend forever.
    if (forward_only_in_route) {
      if (!planner_data->route_handler->isRouteLanelet(extended_lanes.back())) {
        return lanes;
      }
    }

    lanes = extended_lanes;
  }

  return lanes;
}
}
