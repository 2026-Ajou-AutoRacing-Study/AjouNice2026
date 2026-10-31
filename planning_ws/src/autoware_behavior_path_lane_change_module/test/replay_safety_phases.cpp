// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
// Offline native regression against snapshots exported from a finalized bag.
#include <autoware/behavior_path_lane_change_module/utils/safety_check.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

namespace lc = autoware::behavior_path_planner::utils::lane_change;
namespace check = autoware::behavior_path_planner::utils::path_safety_checker;
using Json = nlohmann::json;

geometry_msgs::Pose pose(const Json & value)
{
  geometry_msgs::Pose result;
  result.position.x = value.at(0);
  result.position.y = value.at(1);
  const double yaw = value.at(2);
  result.orientation.z = std::sin(yaw / 2.0);
  result.orientation.w = std::cos(yaw / 2.0);
  return result;
}

check::RSSparams parameters(const Json & j)
{
  check::RSSparams p;
  p.front_vehicle_deceleration = j.at("front_deceleration");
  p.rear_vehicle_deceleration = j.at("rear_deceleration");
  p.rear_vehicle_reaction_time = j.at("reaction_time");
  p.rear_vehicle_safety_time_margin = j.at("safety_time");
  p.longitudinal_distance_min_threshold = j.at("minimum_longitudinal");
  p.lateral_distance_max_threshold = j.at("lateral_margin");
  p.extended_polygon_policy = j.at("policy");
  return p;
}

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  std::ifstream stream(argv[1]);
  const Json input = Json::parse(stream);
  const auto & v = input.at("vehicle");
  const auto vehicle = autoware::vehicle_info_utils::createVehicleInfo(
    v.at("wheel_radius"), v.at("wheel_width"), v.at("wheel_base"), v.at("wheel_tread"),
    v.at("front_overhang"), v.at("rear_overhang"), v.at("left_overhang"),
    v.at("right_overhang"), v.at("vehicle_height"), v.at("max_steer_angle"));
  Json report{{"samples", Json::array()}, {"reproduced_legacy", 0},
    {"newly_safe", 0}, {"still_unsafe", 0}, {"mismatches", 0}};
  size_t reproduced = 0, newly_safe = 0, still_unsafe = 0, mismatches = 0;
  for (const auto & item : input.at("samples")) {
    autoware_internal_planning_msgs::PathWithLaneId path;
    for (const auto & p : item.at("path")) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose = pose(p);
      path.points.push_back(point);
    }
    std::vector<check::PoseWithVelocityStamped> ego;
    for (const auto & p : item.at("ego_prediction")) {
      ego.emplace_back(p.at(0), pose({p.at(1), p.at(2), p.at(3)}), p.at(4));
    }
    check::ExtendedPredictedObject object;
    object.uuid.uuid[0] = 1;
    const auto & shape = item.at("shape");
    object.shape.type = shape.at("type");
    object.shape.dimensions.x = shape.at("dimensions").at(0);
    object.shape.dimensions.y = shape.at("dimensions").at(1);
    object.shape.dimensions.z = shape.at("dimensions").at(2);
    for (const auto & p : shape.at("footprint")) {
      geometry_msgs::Point32 point;
      point.x = p.at(0); point.y = p.at(1); point.z = p.at(2);
      object.shape.footprint.points.push_back(point);
    }
    for (const auto & j : item.at("object_predictions")) {
      check::PredictedPathWithPolygon prediction;
      prediction.confidence = j.at("confidence");
      for (const auto & p : j.at("points")) {
        const auto position = pose({p.at(1), p.at(2), p.at(3)});
        prediction.path.emplace_back(p.at(0), position, p.at(4),
          autoware_utils::to_polygon2d(position, object.shape));
      }
      object.predicted_paths.push_back(std::move(prediction));
    }
    const auto prepare = parameters(item.at("prepare_params"));
    const auto execution = parameters(item.at("execution_params"));
    check::CollisionCheckDebugMap legacy_debug, debug;
    check::RSSCollisionTrace trace;
    const double infinity = std::numeric_limits<double>::infinity();
    const bool prepare_safe = check::check_safety_with_rss_diagnostics(
      path, ego, vehicle, {object}, legacy_debug, prepare, true, 1.0, 3.1416,
      &trace, 0.0, infinity);
    const bool legacy_safe = prepare_safe && check::check_safety_with_rss_diagnostics(
      path, ego, vehicle, {object}, legacy_debug, execution, true, 1.0, 3.1416,
      &trace, 0.0, infinity);
    const auto result = lc::check_safety_by_phase(path, ego, vehicle, {object}, debug,
      prepare, execution, item.at("prepare_duration"), 3.1416, true);
    const auto old_debug = legacy_debug.find(object.uuid.uuid);
    const bool reproduced_legacy = !legacy_safe && old_debug != legacy_debug.end() &&
      old_debug->second.unsafe_reason == item.at("unsafe_reason").get<std::string>() &&
      std::abs(trace.time - item.at("collision_time").get<double>()) < 1e-6 &&
      item.at("checked_stage") == (prepare_safe ? "execution" : "prepare");
    const bool expected = item.at("expected_phase_scoped_safe");
    const bool mismatch = !reproduced_legacy || result.safe != expected;
    reproduced += reproduced_legacy;
    newly_safe += reproduced_legacy && result.safe;
    still_unsafe += reproduced_legacy && !result.safe;
    mismatches += mismatch;
    report["samples"].push_back({{"case", item.at("case")},
      {"object_uuid", item.at("object_uuid")}, {"stamp", item.at("stamp")},
      {"legacy_reproduced", reproduced_legacy}, {"legacy_safe", legacy_safe},
      {"phase_scoped_safe", result.safe}, {"expected_phase_scoped_safe", expected},
      {"recorded_reason", item.at("unsafe_reason")}, {"result_stage", result.stage},
      {"mismatch", mismatch}});
  }
  report["reproduced_legacy"] = reproduced;
  report["newly_safe"] = newly_safe;
  report["still_unsafe"] = still_unsafe;
  report["mismatches"] = mismatches;
  std::cout << report.dump(2) << std::endl;
  return mismatches ? 1 : 0;
}
