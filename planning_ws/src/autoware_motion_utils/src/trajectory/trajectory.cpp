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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/motion_utils/ros1_time.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace autoware::motion_utils
{

//
template void validateNonEmpty<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> &);
template void
validateNonEmpty<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &);
template void validateNonEmpty<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &);

//
template std::optional<bool> isDrivingForward<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> &, const double);
template std::optional<bool>
isDrivingForward<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &, const double);
template std::optional<bool>
isDrivingForward<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &, const double);

//
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &);

//
template std::vector<autoware_planning_msgs::PathPoint>
removeOverlapPoints<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t start_idx);
template std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>
removeOverlapPoints<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t start_idx);
template std::vector<autoware_planning_msgs::TrajectoryPoint>
removeOverlapPoints<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t start_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const size_t src_idx, const size_t dst_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const size_t src_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist);

//
template size_t findNearestIndex<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & point);
template size_t
findNearestIndex<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & point);
template size_t findNearestIndex<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & point);

//
template std::optional<size_t>
findNearestIndex<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Pose & pose, const double max_dist, const double max_yaw);

//
template double
calcLongitudinalOffsetToSegment<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t seg_idx,
  const geometry_msgs::Point & p_target, const bool throw_exception);
template double calcLongitudinalOffsetToSegment<
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t seg_idx, const geometry_msgs::Point & p_target, const bool throw_exception);
template double
calcLongitudinalOffsetToSegment<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t seg_idx,
  const geometry_msgs::Point & p_target, const bool throw_exception);

//
template size_t findNearestSegmentIndex<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & point);
template size_t
findNearestSegmentIndex<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & point);
template size_t findNearestSegmentIndex<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & point);

//
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Pose & pose, const double max_dist, const double max_yaw);

//
template double calcLateralOffset<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & p_target, const size_t seg_idx, const bool throw_exception);
template double
calcLateralOffset<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & p_target, const size_t seg_idx, const bool throw_exception);
template double calcLateralOffset<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & p_target, const size_t seg_idx, const bool throw_exception);

//
template double calcLateralOffset<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & p_target, const bool throw_exception);
template double
calcLateralOffset<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & p_target, const bool throw_exception);
template double calcLateralOffset<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & p_target, const bool throw_exception);

//
template double calcSignedArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t src_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t src_idx,
  const size_t dst_idx);

//
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
template std::vector<double> calcSignedArcLengthPartialSum<
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t src_idx, const size_t dst_idx);
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t src_idx,
  const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & src_point, const size_t dst_idx);
template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &,
  const geometry_msgs::Point & src_point, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &,
  const geometry_msgs::Point & src_point, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t src_idx,
  const geometry_msgs::Point & dst_point);
template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t src_idx, const geometry_msgs::Point & dst_point);
template double calcSignedArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t src_idx,
  const geometry_msgs::Point & dst_point);

//
template double calcSignedArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & src_point, const geometry_msgs::Point & dst_point);
template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & src_point, const geometry_msgs::Point & dst_point);
template double calcSignedArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & src_point, const geometry_msgs::Point & dst_point);

//
template double calcArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points);
template double
calcArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points);
template double calcArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points);

//
template std::vector<double> calcCurvature<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points);
template std::vector<double>
calcCurvature<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points);
template std::vector<double>
calcCurvature<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points);

//
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points);
template std::vector<std::pair<double, std::pair<double, double>>> calcCurvatureAndSegmentLength<
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points);
template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points);

//
template std::vector<double>
calcElevationAngles<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points);
template std::vector<double>
calcElevationAngles<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points);
template std::vector<double>
calcElevationAngles<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const size_t src_idx);

//
template std::optional<geometry_msgs::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t src_idx,
  const double offset, const bool throw_exception);
template std::optional<geometry_msgs::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset, const bool throw_exception);
template std::optional<geometry_msgs::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t src_idx,
  const double offset, const bool throw_exception);

//
template std::optional<geometry_msgs::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & src_point, const double offset);
template std::optional<geometry_msgs::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & src_point, const double offset);
template std::optional<geometry_msgs::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & src_point, const double offset);

//
template std::optional<geometry_msgs::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<geometry_msgs::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<geometry_msgs::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);

//
template std::optional<geometry_msgs::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<geometry_msgs::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<geometry_msgs::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const size_t seg_idx, const geometry_msgs::Point & p_target,
  std::vector<autoware_planning_msgs::PathPoint> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const size_t seg_idx, const geometry_msgs::Point & p_target,
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const size_t seg_idx, const geometry_msgs::Point & p_target,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const double insert_point_length, const geometry_msgs::Point & p_target,
  std::vector<autoware_planning_msgs::PathPoint> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const double insert_point_length, const geometry_msgs::Point & p_target,
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const double insert_point_length, const geometry_msgs::Point & p_target,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_planning_msgs::PathPoint> & points, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const geometry_msgs::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_planning_msgs::PathPoint> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const geometry_msgs::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const geometry_msgs::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t> insertStopPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::PathPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);

//
template std::optional<size_t> insertStopPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::PathPoint> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const double distance_to_stop_point,
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t> insertStopPoint<std::vector<autoware_planning_msgs::PathPoint>>(
  const geometry_msgs::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::PathPoint> & points_with_twist, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const geometry_msgs::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const geometry_msgs::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const size_t stop_seg_idx, const geometry_msgs::Point & stop_point,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t>
insertDecelPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const geometry_msgs::Point & src_point, const double distance_to_decel_point,
  const double velocity,
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist);

//
template void insertOrientation<std::vector<autoware_planning_msgs::PathPoint>>(
  std::vector<autoware_planning_msgs::PathPoint> & points, const bool is_driving_forward);
template void
insertOrientation<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const bool is_driving_forward);
template void insertOrientation<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const bool is_driving_forward);

//
template void insertOrientationAsSpline<std::vector<autoware_planning_msgs::PathPoint>>(
  std::vector<autoware_planning_msgs::PathPoint> & points, const bool is_driving_forward);
template void
insertOrientationAsSpline<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const bool is_driving_forward);
template void insertOrientationAsSpline<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const bool is_driving_forward);

//
template double calcSignedArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::Point & dst_point, const size_t dst_seg_idx);
template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::Point & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::Point & dst_point, const size_t dst_seg_idx);

//
template double calcSignedArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const size_t src_idx,
  const geometry_msgs::Point & dst_point, const size_t dst_seg_idx);
template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const size_t src_idx, const geometry_msgs::Point & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const size_t src_idx,
  const geometry_msgs::Point & dst_point, const size_t dst_seg_idx);

//
template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestIndexWithSoftConstraints<
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Pose & pose, const double dist_threshold, const double yaw_threshold);

//
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Pose & pose, const double dist_threshold, const double yaw_threshold);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points_with_twist,
  const geometry_msgs::Pose & pose, const double max_dist, const double max_yaw);

//
template std::vector<autoware_planning_msgs::PathPoint>
cropForwardPoints<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>
cropForwardPoints<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<autoware_planning_msgs::TrajectoryPoint>
cropForwardPoints<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);

//
template std::vector<autoware_planning_msgs::PathPoint>
cropBackwardPoints<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>
cropBackwardPoints<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<autoware_planning_msgs::TrajectoryPoint>
cropBackwardPoints<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);

//
template std::vector<autoware_planning_msgs::PathPoint>
cropPoints<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>
cropPoints<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<autoware_planning_msgs::TrajectoryPoint>
cropPoints<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);

//
template double calcYawDeviation<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Pose & pose, const bool throw_exception);
template double
calcYawDeviation<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Pose & pose, const bool throw_exception);
template double calcYawDeviation<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Pose & pose, const bool throw_exception);

//
template bool isTargetPointFront<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> & points,
  const geometry_msgs::Point & base_point, const geometry_msgs::Point & target_point,
  const double threshold);
template bool
isTargetPointFront<std::vector<autoware_internal_planning_msgs::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & base_point, const geometry_msgs::Point & target_point,
  const double threshold);
template bool isTargetPointFront<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Point & base_point, const geometry_msgs::Point & target_point,
  const double threshold);

//
double calc_diff_angle_against_trajectory(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points,
  const geometry_msgs::Pose & target_pose)
{
  const size_t nearest_idx =
    autoware::motion_utils::findNearestIndex(traj_points, target_pose.position);
  const double traj_yaw = tf2::getYaw(traj_points.at(nearest_idx).pose.orientation);

  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double diff_yaw = autoware_utils_math::normalize_radian(target_yaw - traj_yaw);
  return diff_yaw;
}

void calculate_time_from_start(
  std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const geometry_msgs::Point & current_ego_point, const float min_velocity)
{
  const auto nearest_segment_idx = findNearestSegmentIndex(trajectory, current_ego_point);
  if (nearest_segment_idx + 1 == trajectory.size()) {
    return;
  }
  for (auto & p : trajectory) {
    p.time_from_start = duration_from_seconds(0.0);
  }
  // TODO(Maxime): some points can have very low velocities which introduce huge time errors
  // Temporary solution: use a minimum velocity
  for (auto idx = nearest_segment_idx + 1; idx < trajectory.size(); ++idx) {
    const auto & from = trajectory[idx - 1];
    const auto velocity = std::max(min_velocity, from.longitudinal_velocity_mps);
    if (velocity != 0.0) {
      auto & to = trajectory[idx];
      const auto t = autoware_utils_geometry::calc_distance2d(from, to) / velocity;
      to.time_from_start = duration_from_seconds(t + duration_to_seconds(from.time_from_start));
    }
  }
}
}  // namespace autoware::motion_utils
