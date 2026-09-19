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

#include <autoware/behavior_path_planner_common/utils/root_reference_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/dynamic_area_core.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <boost/geometry.hpp>
#include <functional>
#include <unordered_set>
namespace autoware::behavior_path_planner::utils {
using Pose = geometry_msgs::Pose;
using Point = geometry_msgs::Point;
using PoseWithUuidStamped = autoware_planning_msgs::PoseWithUuidStamped;
using PathWithLaneId = autoware_internal_planning_msgs::PathWithLaneId;
using RouteHandler = autoware::route_handler::RouteHandler;
bool isInLaneletWithYawThreshold(
  const Pose & current_pose, const lanelet::ConstLanelet & lanelet, const double yaw_threshold,
  const double radius)
{
  const double pose_yaw = tf2::getYaw(current_pose.orientation);
  const double lanelet_angle = autoware::experimental::lanelet2_utils::get_lanelet_angle(
    lanelet, autoware::experimental::lanelet2_utils::from_ros(current_pose.position).basicPoint());
  const double angle_diff = std::abs(autoware_utils::normalize_radian(lanelet_angle - pose_yaw));

  return (angle_diff < std::abs(yaw_threshold)) &&
         autoware::experimental::lanelet2_utils::is_in_lanelet(current_pose, lanelet, radius);
}

bool isEgoOutOfRoute(
  const Pose & self_pose, const lanelet::ConstLanelet & closest_road_lane,
  const std::optional<PoseWithUuidStamped> & modified_goal,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  const Pose & goal_pose = (modified_goal && modified_goal->uuid == route_handler->getRouteUuid())
                             ? modified_goal->pose
                             : route_handler->getGoalPose();

  lanelet::ConstLanelet goal_lane;
  const auto shoulder_goal_lanes = route_handler->getShoulderLaneletsAtPose(goal_pose);
  if (!shoulder_goal_lanes.empty()) goal_lane = shoulder_goal_lanes.front();
  const auto is_failed_getting_lanelet =
    shoulder_goal_lanes.empty() && !route_handler->getGoalLanelet(&goal_lane);
  if (is_failed_getting_lanelet) {
    ROS_WARN_STREAM_NAMED("behavior_path_planner.util", "cannot find goal lanelet");
    return true;
  }

  // If ego vehicle is over goal on goal lane, return true
  const double yaw_threshold = autoware_utils::deg2rad(90);
  if (
    closest_road_lane.id() == goal_lane.id() &&
    isInLaneletWithYawThreshold(self_pose, goal_lane, yaw_threshold)) {
    constexpr double buffer = 1.0;
    const auto ego_arc_coord =
      autoware::experimental::lanelet2_utils::get_arc_coordinates({goal_lane}, self_pose);
    const auto goal_arc_coord = autoware::experimental::lanelet2_utils::get_arc_coordinates(
      {goal_lane}, route_handler->getGoalPose());
    if (ego_arc_coord.length > goal_arc_coord.length + buffer) {
      ROS_WARN_STREAM_NAMED("behavior_path_planner.util", "ego pose is beyond goal");
      return true;
    }
    return false;
  }

  // If ego vehicle is out of the closest lanelet, return true
  // Check if ego vehicle is in shoulder lane
  const bool is_in_shoulder_lane = !route_handler->getShoulderLaneletsAtPose(self_pose).empty();
  // Check if ego vehicle is in road lane
  const bool is_in_road_lane = std::invoke([&]() {
    if (autoware::experimental::lanelet2_utils::is_in_lanelet(self_pose, closest_road_lane)) {
      return true;
    }

    // check previous lanes for backward driving (e.g. pull out)
    const auto prev_lanes = route_handler->getPreviousLanelets(closest_road_lane);
    for (const auto & lane : prev_lanes) {
      if (autoware::experimental::lanelet2_utils::is_in_lanelet(self_pose, lane)) {
        return true;
      }
    }

    return false;
  });

  return !is_in_shoulder_lane && !is_in_road_lane;
}

std::vector<lanelet::Id> get_lanelet_id_from_path(const PathWithLaneId & path)
{
  std::vector<lanelet::Id> unique_lanelet_ids;
  for (const auto & p : path.points) {
    const auto & lane_ids = p.lane_ids;
    for (const auto & lane_id : lane_ids) {
      if (
        std::find(unique_lanelet_ids.begin(), unique_lanelet_ids.end(), lane_id) ==
        unique_lanelet_ids.end()) {
        unique_lanelet_ids.push_back(lane_id);
      }
    }
  }
  return unique_lanelet_ids;
}

lanelet::ConstLanelets get_lanelet_sequence_from_path(
  const PathWithLaneId & path, const RouteHandler & route_handler)
{
  const auto unique_lanelet_ids = get_lanelet_id_from_path(path);

  lanelet::ConstLanelets lanelets;
  lanelets.reserve(unique_lanelet_ids.size());
  for (const auto & lane_id : unique_lanelet_ids) {
    lanelets.push_back(route_handler.getLaneletsFromId(lane_id));
  }

  return lanelets;
}

lanelet::ConstLanelets getLaneletsFromPath(
  const PathWithLaneId & path, const std::shared_ptr<RouteHandler> & route_handler)
{
  return get_lanelet_sequence_from_path(path, *route_handler);
}

BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto p = planner_data->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 10.0;
  const double backward_length = p.backward_path_length + extra_margin;
  const auto current_lanes_with_backward_margin =
    route_handler->getLaneletSequence(current_lane, backward_length, p.forward_path_length);
  const auto no_shift_pose = autoware::experimental::lanelet2_utils::get_closest_center_pose(
    current_lane, autoware::experimental::lanelet2_utils::from_ros(current_pose));
  reference_path = getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, no_shift_pose, backward_length,
    p.forward_path_length, p);

  if (reference_path.points.empty()) {
    ROS_WARN_THROTTLE(5.0, "Empty reference path detected.");
    BehaviorModuleOutput output;
    return output;
  }

  // clip backward length
  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      reference_path.points, no_shift_pose, p.ego_nearest_dist_threshold,
      p.ego_nearest_yaw_threshold);
  reference_path.points = autoware::motion_utils::cropPoints(
    reference_path.points, no_shift_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  BehaviorModuleOutput output;
  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}

BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data)
{
  BehaviorModuleOutput output;

  const auto & route_handler = planner_data->route_handler;
  const auto & modified_goal = planner_data->prev_modified_goal;

  const Pose goal_pose = modified_goal ? modified_goal->pose : route_handler->getGoalPose();

  lanelet::ConstLanelet goal_lane;
  const auto shoulder_goal_lanes = route_handler->getShoulderLaneletsAtPose(goal_pose);
  if (!shoulder_goal_lanes.empty()) goal_lane = shoulder_goal_lanes.front();
  const auto is_failed_getting_lanelet =
    shoulder_goal_lanes.empty() && !route_handler->getGoalLanelet(&goal_lane);
  if (is_failed_getting_lanelet) {
    return output;
  }

  constexpr double backward_length = 1.0;
  const auto arc_coord =
    autoware::experimental::lanelet2_utils::get_arc_coordinates({goal_lane}, goal_pose);
  const double s_start = std::max(arc_coord.length - backward_length, 0.0);
  const double s_end = arc_coord.length;

  auto reference_path = route_handler->getCenterLinePath({goal_lane}, s_start, s_end);

  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // Insert zero velocity to each point in the path.
  for (auto & point : reference_path.points) {
    point.point.longitudinal_velocity_mps = 0.0;
  }

  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}

void generateDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const bool enable_expanding_hatched_road_markings, const bool enable_expanding_intersection_areas,
  const bool enable_expanding_freespace_areas,
  const std::shared_ptr<const PlannerData> planner_data, const bool is_driving_forward)
{
  if (path.points.empty()) {
    return;
  }

  path.left_bound.clear();
  path.right_bound.clear();

  // Insert Position
  path.left_bound = calcBound(
    path, planner_data, lanes, enable_expanding_hatched_road_markings,
    enable_expanding_intersection_areas, enable_expanding_freespace_areas, true,
    is_driving_forward);
  path.right_bound = calcBound(
    path, planner_data, lanes, enable_expanding_hatched_road_markings,
    enable_expanding_intersection_areas, enable_expanding_freespace_areas, false,
    is_driving_forward);

  if (path.left_bound.empty() || path.right_bound.empty()) {
    ROS_ERROR_STREAM_THROTTLE(1.0,
      "The right or left bound of drivable area is empty");
    return;
  }

  const auto & expansion_params = planner_data->drivable_area_expansion_parameters;
  if (expansion_params.enabled) {
    autoware::behavior_path_planner::drivable_area_expansion::expand_drivable_area(
      path, planner_data);
  }
}

void generateDrivableArea(
  PathWithLaneId & path, const double vehicle_length, const double offset,
  const bool is_driving_forward)
{
  using autoware_utils::calc_offset_pose;

  // remove path points which is close to the previous point
  PathWithLaneId resampled_path{};
  const double resample_interval = 2.0;
  for (size_t i = 0; i < path.points.size(); ++i) {
    if (i == 0) {
      resampled_path.points.push_back(path.points.at(i));
    } else {
      const auto & prev_point = resampled_path.points.back().point.pose.position;
      const auto & curr_point = path.points.at(i).point.pose.position;
      const double signed_arc_length =
        autoware::motion_utils::calcSignedArcLength(path.points, prev_point, curr_point);
      if (signed_arc_length > resample_interval) {
        resampled_path.points.push_back(path.points.at(i));
      }
    }
  }
  if (resampled_path.points.empty()) {
    return;
  }
  // add last point of path if enough far from the one of resampled path
  constexpr double th_last_point_distance = 0.3;
  if (
    autoware_utils::calc_distance2d(
      resampled_path.points.back().point.pose.position, path.points.back().point.pose.position) >
    th_last_point_distance) {
    resampled_path.points.push_back(path.points.back());
  }

  // create bound point by calculating offset point
  std::vector<Point> left_bound;
  std::vector<Point> right_bound;
  for (const auto & point : resampled_path.points) {
    const auto & pose = point.point.pose;

    const auto left_point = calc_offset_pose(pose, 0, offset, 0);
    const auto right_point = calc_offset_pose(pose, 0, -offset, 0);

    left_bound.push_back(left_point.position);
    right_bound.push_back(right_point.position);
  }

  if (is_driving_forward) {
    // add backward offset point to bound
    const Pose first_point =
      calc_offset_pose(resampled_path.points.front().point.pose, -vehicle_length, 0, 0);
    const Pose left_first_point = calc_offset_pose(first_point, 0, offset, 0);
    const Pose right_first_point = calc_offset_pose(first_point, 0, -offset, 0);
    left_bound.insert(left_bound.begin(), left_first_point.position);
    right_bound.insert(right_bound.begin(), right_first_point.position);

    // add forward offset point to bound
    const Pose last_point =
      calc_offset_pose(resampled_path.points.back().point.pose, vehicle_length, 0, 0);
    const Pose left_last_point = calc_offset_pose(last_point, 0, offset, 0);
    const Pose right_last_point = calc_offset_pose(last_point, 0, -offset, 0);
    left_bound.push_back(left_last_point.position);
    right_bound.push_back(right_last_point.position);
  } else {
    // add forward offset point to bound
    const Pose first_point =
      calc_offset_pose(resampled_path.points.front().point.pose, vehicle_length, 0, 0);
    const Pose left_first_point = calc_offset_pose(first_point, 0, offset, 0);
    const Pose right_first_point = calc_offset_pose(first_point, 0, -offset, 0);
    left_bound.insert(left_bound.begin(), left_first_point.position);
    right_bound.insert(right_bound.begin(), right_first_point.position);

    // add backward offset point to bound
    const Pose last_point =
      calc_offset_pose(resampled_path.points.back().point.pose, -vehicle_length, 0, 0);
    const Pose left_last_point = calc_offset_pose(last_point, 0, offset, 0);
    const Pose right_last_point = calc_offset_pose(last_point, 0, -offset, 0);
    left_bound.push_back(left_last_point.position);
    right_bound.push_back(right_last_point.position);
  }

  if (left_bound.empty() || right_bound.empty()) {
    return;
  }

  // fix intersected bound
  // if bound is intersected, remove them and insert intersection point
  typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
  typedef boost::geometry::model::linestring<BoostPoint> LineString;
  auto modify_bound_intersection = [](const std::vector<Point> & bound) {
    const double intersection_check_distance = 10.0;
    std::vector<Point> modified_bound;
    size_t i = 0;
    while (i < bound.size() - 1) {
      BoostPoint p1(bound.at(i).x, bound.at(i).y);
      BoostPoint p2(bound.at(i + 1).x, bound.at(i + 1).y);
      LineString p_line;
      p_line.push_back(p1);
      p_line.push_back(p2);
      bool intersection_found = false;
      for (size_t j = i + 2; j < bound.size() - 1; j++) {
        const double distance = autoware_utils::calc_distance2d(bound.at(i), bound.at(j));
        if (distance > intersection_check_distance) {
          break;
        }
        LineString q_line;
        BoostPoint q1(bound.at(j).x, bound.at(j).y);
        BoostPoint q2(bound.at(j + 1).x, bound.at(j + 1).y);
        q_line.push_back(q1);
        q_line.push_back(q2);
        std::vector<BoostPoint> intersection_points;
        boost::geometry::intersection(p_line, q_line, intersection_points);
        if (intersection_points.size() > 0) {
          modified_bound.push_back(bound.at(i));
          Point intersection_point;
          intersection_point.x = intersection_points.at(0).x();
          intersection_point.y = intersection_points.at(0).y();
          modified_bound.push_back(intersection_point);
          i = j + 1;
          intersection_found = true;
          break;
        }
      }
      if (!intersection_found) {
        modified_bound.push_back(bound.at(i));
        i++;
      }
    }
    modified_bound.push_back(bound.back());
    return modified_bound;
  };
  std::vector<Point> modified_left_bound = modify_bound_intersection(left_bound);
  std::vector<Point> modified_right_bound = modify_bound_intersection(right_bound);

  // set bound to path
  path.left_bound = modified_left_bound;
  path.right_bound = modified_right_bound;
}
}
