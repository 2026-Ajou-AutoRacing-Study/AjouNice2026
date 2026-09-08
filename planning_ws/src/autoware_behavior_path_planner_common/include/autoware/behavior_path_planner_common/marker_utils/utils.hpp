#pragma once

#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/parameters_core.hpp>
#include <autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <geometry_msgs/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstdint>
#include <string>
#include <vector>

namespace marker_utils
{
using autoware::behavior_path_planner::DrivableLanes;
using autoware::behavior_path_planner::ShiftLineArray;
using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware_internal_planning_msgs::PathWithLaneId;

void addFootprintMarker(visualization_msgs::Marker & marker, const geometry_msgs::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);
visualization_msgs::MarkerArray createFootprintMarkerArray(
  const geometry_msgs::Pose & pose, autoware::vehicle_info_utils::VehicleInfo vehicle_info,
  const std::string && ns, const int32_t & id, const float & r, const float & g, const float & b);
visualization_msgs::MarkerArray createPredictedPathMarkerArray(
  const autoware_perception_msgs::PredictedPath & path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, std::string && ns,
  const int32_t & id, const float & r, const float & g, const float & b);

visualization_msgs::MarkerArray createPoseMarkerArray(
  const geometry_msgs::Pose & pose, std::string ns, int32_t id, float r, float g, float b);
visualization_msgs::MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, std::string ns, int64_t lane_id, float r, float g, float b);
visualization_msgs::MarkerArray createShiftLineMarkerArray(
  const autoware::behavior_path_planner::ShiftLineArray & shift_lines, double base_shift,
  std::string ns, float r, float g, float b, float width);
visualization_msgs::MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> & shift_distance, const PathWithLaneId & reference, std::string ns,
  float r, float g, float b);
visualization_msgs::MarkerArray createShiftGradMarkerArray(
  const std::vector<double> & grad, const std::vector<double> & shift_distance,
  const PathWithLaneId & reference, std::string ns, float r, float g, float b);
visualization_msgs::MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, std::string ns, float r, float g, float b);
visualization_msgs::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::Polygon & polygon, std::string ns, int64_t lane_id, float r, float g,
  float b, float width = 0.3F);
visualization_msgs::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::PredictedObjects & objects, std::string ns, int64_t lane_id,
  float r, float g, float b);
visualization_msgs::MarkerArray createDrivableLanesMarkerArray(
  const std::vector<DrivableLanes> & drivable_lanes, std::string ns);
visualization_msgs::MarkerArray showPolygon(
  const CollisionCheckDebugMap & object_debug, std::string ns);
visualization_msgs::MarkerArray showPredictedPath(
  const CollisionCheckDebugMap & object_debug, std::string ns);
visualization_msgs::MarkerArray showSafetyCheckInfo(
  const CollisionCheckDebugMap & object_debug, std::string ns);

}  // namespace marker_utils
