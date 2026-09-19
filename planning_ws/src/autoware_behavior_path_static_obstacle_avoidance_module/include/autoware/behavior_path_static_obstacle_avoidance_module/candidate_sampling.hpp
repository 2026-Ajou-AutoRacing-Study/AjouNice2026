#pragma once

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::behavior_path_planner::boundary_sampling
{
inline bool mayCrossBoundary(const std::string & type, const std::string & subtype)
{
  if (type == "virtual") return true;
  if (type != "line_thin" && type != "line_thick") return false;
  return subtype == "solid" || subtype == "dashed" || subtype == "solid_solid" ||
         subtype == "solid_dashed" || subtype == "dashed_solid";
}

template <class EgoPolygon, class ObjectPolygon>
bool hasHardClearance(
  const EgoPolygon & ego, const ObjectPolygon & object, const double hard_margin)
{
  constexpr double numerical_tolerance = 1e-3;
  return !boost::geometry::intersects(ego, object) &&
         boost::geometry::distance(ego, object) + numerical_tolerance >=
           std::max(0.0, hard_margin);
}

// Ratios shorten only future intervals: entry end and return start stay fixed.
// Return false rather than letting collapsed/out-of-range intervals reach PathShifter.
template <class Lines, class Path, class Arc>
bool shorten(Lines & lines, const Path & path, const Arc & arc,
             double entry_ratio, double return_ratio)
{
  if (arc.size() != path.points.size() || !std::is_sorted(arc.begin(), arc.end()) ||
      !(entry_ratio > 0.0 && entry_ratio <= 1.0 && return_ratio > 0.0 && return_ratio <= 1.0)) {
    return false;
  }
  for (auto & line : lines) {
    if (line.end_idx <= line.start_idx || line.end_idx >= arc.size()) return false;
    if (line.start_longitudinal <= 0.2) continue;
    const bool returning = std::abs(line.end_shift_length) < std::abs(line.start_shift_length);
    const auto ratio = returning ? return_ratio : entry_ratio;
    if (ratio == 1.0) continue;
    const auto length = line.getRelativeLongitudinal();
    if (!(length > 0.0)) return false;
    const auto target = returning ? line.start_longitudinal + length * ratio
                                  : line.end_longitudinal - length * ratio;
    const auto it = std::lower_bound(arc.begin(), arc.end(), target);
    if (it == arc.end()) return false;
    const auto index = static_cast<size_t>(it - arc.begin());
    if (returning) {
      line.end_idx = index;
      line.end = path.points.at(index).point.pose;
      line.end_longitudinal = *it;
    } else {
      line.start_idx = index;
      line.start = path.points.at(index).point.pose;
      line.start_longitudinal = *it;
    }
    if (line.end_idx <= line.start_idx) return false;
  }
  return true;
}

// Scale only a not-yet-started maneuver.  Entry/return lengths are sampled
// separately by shorten(); this adds lateral-depth (and therefore gradient,
// curvature and yaw) diversity without introducing a discontinuity at ego.
template <class Lines>
bool scaleLateralExcursion(Lines & lines, const double ratio)
{
  if (lines.empty() || !(ratio > 0.0 && ratio <= 1.0)) return false;
  if (ratio == 1.0) return true;

  constexpr double started_distance = 0.2;
  constexpr double shifted_ego_tolerance = 0.05;
  if (std::any_of(lines.begin(), lines.end(), [](const auto & line) {
        return line.start_longitudinal <= started_distance;
      }) || std::abs(lines.front().start_shift_length) > shifted_ego_tolerance) {
    return false;
  }

  // A fresh maneuver starts from the reference-path shift.  Scaling every
  // endpoint by the same ratio preserves continuity between merged lines and
  // keeps a return-to-center endpoint exactly at zero.
  for (auto & line : lines) {
    line.start_shift_length *= ratio;
    line.end_shift_length *= ratio;
  }
  return true;
}
}  // namespace autoware::behavior_path_planner::boundary_sampling
