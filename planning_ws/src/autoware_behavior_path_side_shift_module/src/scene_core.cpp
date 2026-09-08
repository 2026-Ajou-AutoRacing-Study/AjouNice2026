#include <autoware/behavior_path_side_shift_module/scene_core.hpp>

#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>
#include <autoware/behavior_path_side_shift_module/utils.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner
{
namespace
{
size_t getForwardIndexByArclength(const PathWithLaneId & path, const size_t start_index, const double distance)
{
  if (path.points.empty()) return 0;
  double accumulated = 0.0;
  for (size_t index = start_index; index + 1 < path.points.size(); ++index) {
    const auto & current = path.points.at(index).point.pose.position;
    const auto & next = path.points.at(index + 1).point.pose.position;
    accumulated += std::hypot(next.x - current.x, next.y - current.y);
    if (accumulated >= distance) return index + 1;
  }
  return path.points.size() - 1;
}
}  // namespace

SideShiftSceneCore::SideShiftSceneCore(LegacySideShiftParameters parameters) : parameters_(std::move(parameters)) {}

void SideShiftSceneCore::setReferencePath(const PathWithLaneId & path)
{
  reference_path_ = path;
  path_shifter_.setPath(reference_path_);
}

void SideShiftSceneCore::setCurrentLanelets(const lanelet::ConstLanelets & lanelets)
{
  current_lanelets_ = lanelets;
}

void SideShiftSceneCore::requestLateralOffset(const double offset)
{
  requested_lateral_offset_ = offset;
}

ShiftLine SideShiftSceneCore::calcShiftLine(const size_t ego_index, const double ego_speed) const
{
  ShiftLine line;
  if (reference_path_.points.empty()) return line;
  const auto start_distance = std::max(
    parameters_.min_distance_to_start_shifting,
    std::abs(ego_speed) * parameters_.time_to_start_shifting);
  const auto current_shift = getClosestShiftLength(
    ShiftedPath{path_shifter_.getReferencePath(), {}}, reference_path_.points.at(ego_index).point.pose.position);
  const auto shift_length = requested_lateral_offset_ - current_shift;
  const auto jerk_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    shift_length, parameters_.shifting_lateral_jerk,
    std::max(std::abs(ego_speed), parameters_.min_shifting_speed));
  const auto end_distance = start_distance + std::max(jerk_distance, parameters_.min_shifting_distance);
  line.end_shift_length = requested_lateral_offset_;
  line.start_idx = getForwardIndexByArclength(reference_path_, ego_index, start_distance);
  line.end_idx = getForwardIndexByArclength(reference_path_, ego_index, end_distance);
  line.start = reference_path_.points.at(line.start_idx).point.pose;
  line.end = reference_path_.points.at(line.end_idx).point.pose;
  return line;
}

BehaviorModuleOutput SideShiftSceneCore::plan(const size_t ego_index, const double ego_speed)
{
  BehaviorModuleOutput output;
  if (reference_path_.points.empty() || current_lanelets_.empty() || ego_index >= reference_path_.points.size()) {
    return output;
  }
  const auto line = calcShiftLine(ego_index, ego_speed);
  path_shifter_.setShiftLines({line});
  ShiftedPath shifted;
  if (!path_shifter_.generate(&shifted) || shifted.path.points.empty()) return output;
  setOrientation(&shifted.path);

  const auto bounds = std::minmax_element(shifted.shift_length.begin(), shifted.shift_length.end());
  constexpr double threshold = 0.1;
  constexpr double margin = 0.5;
  const auto left_offset = std::max(
    *bounds.second + (*bounds.first > threshold ? margin : 0.0),
    parameters_.drivable_area_left_bound_offset);
  const auto right_offset = -std::min(
    *bounds.first - (*bounds.first < -threshold ? margin : 0.0),
    -parameters_.drivable_area_right_bound_offset);
  const auto lanes = utils::generateDrivableLanes(current_lanelets_);
  const auto shortened = utils::cutOverlappedLanes(shifted.path, lanes);
  output.path = shifted.path;
  output.reference_path = reference_path_;
  output.drivable_area_info.drivable_lanes = utils::expandLanelets(
    shortened, left_offset, right_offset, parameters_.drivable_area_types_to_skip);
  output.drivable_area_info.is_already_expanded = true;
  return output;
}
}  // namespace autoware::behavior_path_planner
