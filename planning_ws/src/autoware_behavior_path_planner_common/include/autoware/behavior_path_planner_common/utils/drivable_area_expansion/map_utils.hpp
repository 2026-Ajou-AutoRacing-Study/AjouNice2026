#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__MAP_UTILS_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__MAP_UTILS_HPP_

#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
SegmentRtree extract_uncrossable_segments(const lanelet::LaneletMap & lanelet_map, const Point & ego_point,
  const DrivableAreaExpansionParameters & params);
bool has_types(const lanelet::ConstLineString3d & ls,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types);
}  // namespace autoware::behavior_path_planner::drivable_area_expansion

#endif  // AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__MAP_UTILS_HPP_
