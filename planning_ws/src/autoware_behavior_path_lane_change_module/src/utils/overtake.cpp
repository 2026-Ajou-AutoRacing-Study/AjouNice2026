// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/utils/overtake.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner::utils::lane_change
{
double rear_body_gap(
  const autoware_internal_planning_msgs::PathWithLaneId & reference,
  const geometry_msgs::Pose & ego, const autoware_utils_geometry::Polygon2d & object,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle)
{
  if (reference.points.size() < 2 || object.outer().size() < 4) {
    return -std::numeric_limits<double>::infinity();
  }
  const auto body = safety::create_extended_ego_polygon(ego, vehicle, 0.0, 0.0, false);
  const auto & origin = reference.points.front().point.pose.position;
  const auto arc = [&](const auto & p) {
    geometry_msgs::Point point;
    point.x = p.x();
    point.y = p.y();
    return autoware::motion_utils::calcSignedArcLength(reference.points, origin, point);
  };
  double ego_back = std::numeric_limits<double>::infinity();
  double object_front = -std::numeric_limits<double>::infinity();
  for (const auto & p : body.outer()) ego_back = std::min(ego_back, arc(p));
  for (const auto & p : object.outer()) object_front = std::max(object_front, arc(p));
  return ego_back - object_front;
}

double cooperative_rear_distance(
  const double ego_speed, const double rear_speed,
  const autoware::behavior_path_planner::lane_change::OvertakeParameters & p)
{
  if (!std::isfinite(ego_speed) || !std::isfinite(rear_speed) ||
      !std::isfinite(p.minimum_gap) || p.minimum_gap <= 0.0 ||
      !std::isfinite(p.npc_reaction_time) || p.npc_reaction_time < 0.0 ||
      !std::isfinite(p.npc_max_deceleration) || p.npc_max_deceleration <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double closing = std::max(0.0, rear_speed - ego_speed);
  return p.minimum_gap + closing * p.npc_reaction_time +
         closing * closing / (2.0 * p.npc_max_deceleration);
}

bool check_cooperative_rear(
  const autoware_internal_planning_msgs::PathWithLaneId & reference,
  const std::vector<safety::PoseWithVelocityStamped> & ego,
  const safety::ExtendedPredictedObject & object,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const autoware::behavior_path_planner::lane_change::OvertakeParameters & p,
  const double lateral_margin, safety::CollisionCheckDebugMap & debug_map,
  safety::RSSCollisionTrace * trace)
{
  safety::CollisionCheckDebug debug;
  const auto fail = [&](const char * reason) {
    debug.is_safe = false;
    debug.unsafe_reason = reason;
    debug_map[object.uuid.uuid] = debug;
    return false;
  };
  if (!p.enable || !p.assume_cooperative_npc || ego.size() < 2 ||
      ego.front().time != 0.0 || !std::isfinite(ego.back().time) ||
      ego.back().time <= 0.0 || object.predicted_paths.empty()) {
    return fail("cooperative_prediction_unavailable");
  }
  // Check the complete maneuver, including the endpoint, at 10 Hz even if the
  // source object's samples are 0.5 s apart. No yaw/position-based object skip.
  const double end = ego.back().time;
  for (size_t index = 0; index < object.predicted_paths.size(); ++index) {
    const auto & path = object.predicted_paths[index];
    if (path.path.size() < 2 || path.path.front().time > 0.0 ||
        path.path.back().time + 1e-6 < end) {
      return fail("cooperative_prediction_too_short");
    }
    for (double t = 0.0; ; t = std::min(end, t + 0.1)) {
      const auto e = safety::calc_interpolated_pose_with_velocity(ego, t);
      const auto o = safety::calc_interpolated_object_pose(path.path, t, object.shape);
      if (!e || !o) return fail("cooperative_prediction_unavailable");
      const auto body = safety::create_extended_ego_polygon(e->pose, vehicle, 0, 0, false);
      const double extension = cooperative_rear_distance(e->velocity, o->velocity, p);
      const auto region = safety::create_extended_object_polygon(*o, extension, lateral_margin, false);
      debug.expected_ego_pose = e->pose;
      debug.expected_object_pose = o->pose;
      debug.extended_ego_polygon = body;
      debug.extended_object_polygon = region;
      if (trace) {
        *trace = safety::RSSCollisionTrace{};
        trace->time = t;
        trace->predicted_path_index = index;
        trace->path_confidence = path.confidence;
        trace->ego_velocity = e->velocity;
        trace->object_velocity = o->velocity;
        trace->longitudinal_extension = extension;
        trace->lateral_margin = lateral_margin;
      }
      if (boost::geometry::intersects(body, o->polygon)) return fail("overlap_polygon");
      // Distance is measured between body boundaries on the common road, not
      // between the rear-axle base_link and an object's center.
      if (rear_body_gap(reference, e->pose, o->polygon, vehicle) + 1e-3 < extension ||
          boost::geometry::intersects(body, region)) {
        return fail("cooperative_rear_gap");
      }
      if (t >= end) break;
    }
  }
  debug.is_safe = true;
  debug_map[object.uuid.uuid] = std::move(debug);
  return true;
}
}
