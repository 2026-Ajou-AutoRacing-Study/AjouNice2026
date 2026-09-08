#include <autoware/behavior_path_goal_planner_module/goal_candidate_area_filter.hpp>

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/bus_stop_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_parking_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>

namespace autoware::behavior_path_planner
{
namespace
{
bool intersectsAny(
  const autoware_utils_geometry::LinearRing2d & footprint,
  const std::vector<autoware_utils_geometry::Polygon2d> & areas)
{
  for (const auto & area : areas) {
    if (boost::geometry::intersects(area, footprint)) return true;
  }
  return false;
}

bool withinAny(
  const autoware_utils_geometry::LinearRing2d & footprint,
  const std::vector<autoware_utils_geometry::Polygon2d> & areas)
{
  for (const auto & area : areas) {
    if (boost::geometry::within(footprint, area)) return true;
  }
  return false;
}
}  // namespace

bool isGoalCandidatePoseAllowedByAreas(
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const geometry_msgs::Pose & goal_pose, const GoalCandidateAreaConstraints & constraints)
{
  const auto footprint = autoware_utils_geometry::transform_vector(
    local_vehicle_footprint, autoware_utils_geometry::pose2transform(goal_pose));
  if (constraints.use_bus_stop_area && !withinAny(footprint, constraints.bus_stop_areas)) return false;
  if (intersectsAny(footprint, constraints.no_parking_areas)) return false;
  if (intersectsAny(footprint, constraints.no_stopping_areas)) return false;
  return boost::geometry::within(footprint, constraints.departure_check_area);
}

autoware_utils_geometry::Polygon2d toGoalCandidateAreaPolygon(
  const lanelet::ConstPolygon3d & polygon)
{
  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : polygon) {
    boost::geometry::append(converted.outer(), autoware_utils_geometry::Point2d{point.x(), point.y()});
  }
  if (!converted.outer().empty()) {
    const auto & first = converted.outer().front();
    const auto & last = converted.outer().back();
    if (first.x() != last.x() || first.y() != last.y()) {
      boost::geometry::append(converted.outer(), first);
    }
  }
  return converted;
}

GoalCandidateAreaConstraints appendGoalCandidateRegulatoryAreas(
  GoalCandidateAreaConstraints constraints, const lanelet::ConstLanelets & pull_over_lanes)
{
  for (const auto & regulatory : lanelet::utils::query::noParkingAreas(pull_over_lanes)) {
    for (const auto & area : regulatory->noParkingAreas()) {
      constraints.no_parking_areas.push_back(toGoalCandidateAreaPolygon(area));
    }
  }
  for (const auto & regulatory : lanelet::utils::query::noStoppingAreas(pull_over_lanes)) {
    for (const auto & area : regulatory->noStoppingAreas()) {
      constraints.no_stopping_areas.push_back(toGoalCandidateAreaPolygon(area));
    }
  }
  for (const auto & regulatory : lanelet::utils::query::busStopAreas(pull_over_lanes)) {
    for (const auto & area : regulatory->busStopAreas()) {
      constraints.bus_stop_areas.push_back(toGoalCandidateAreaPolygon(area));
    }
  }
  return constraints;
}
}  // namespace autoware::behavior_path_planner
