#pragma once
// Optional NORMAL proposal construction. Never a safety/approval certificate.
#include <autoware/behavior_path_lane_change_module/utils/path.hpp>
#include <functional>
#include <ros/time.h>
namespace autoware::behavior_path_planner::utils::lane_change::world_candidate {
namespace detail {
// Internal geometry kernel, exposed for focused source-level regression only.
PathWithLaneId construct_geometry(const PathWithLaneId &,const PathWithLaneId &,
  const geometry_msgs::Pose &,size_t &,size_t &);
}
struct Input {
  behavior_path_planner::lane_change::CommonDataPtr common;
  LaneChangePhaseMetrics prepare_metric, changing_metric;
  PathWithLaneId prepare;
  std::vector<std::vector<int64_t>> sorted_lane_ids;
  double shift_length{};
  ros::WallTime deadline;
  double horizon{};
  double minimum_acceleration{}, minimum_jerk{}, maximum_jerk{};
};
struct Services {
  // Production adapter calls existing get_reference_path_from_target_lane;
  // no recorded geometry/IDs and no alternate resampling implementation.
  std::function<PathWithLaneId(const Input &)> native_reference;
  // EXACT existing PlannerData::findEgoSegmentIndex + anchor convention.
  std::function<std::pair<PathWithLaneId,size_t>(const PathWithLaneId &)> anchor;
  // EXACT caller certifiedEnvelope flags/current+target refs/upstream/vehicle.
  // Arguments: anchored path, begin, LC start, LC end, pre-anchor geometry.
  std::function<std::vector<double>(const PathWithLaneId &,size_t,size_t,size_t,
                                   const PathWithLaneId &)> hard_caps;
};
struct Result {
  LaneChangePath candidate;
  double construction_ms{};
  // Always false: caller must run SAME native seam/length/forbidden/isValidPath,
  // candidate safety, reachable admission and later fresh final-output gates.
  bool safety_validated{false};
};
bool enabled(bool requested,bool normal,bool strict,bool continuous,bool partial,
             bool reachable,bool physical_endpoint);
Result make(const Input &,const Services &);
} // namespace
