#include "scene.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <iostream>

namespace autoware::behavior_velocity_planner
{
using autoware::motion_utils::calcSignedArcLength;
using autoware_utils::create_point;

SpeedBumpModule::SpeedBumpModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem, const PlannerParam & planner_param,
  const ros::NodeHandle & node, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface(module_id, node, std::move(time_keeper), std::move(planning_factor_interface)),
  lane_id_(lane_id), speed_bump_reg_elem_(speed_bump_reg_elem), planner_param_(planner_param)
{
  const auto height = static_cast<float>(speed_bump_reg_elem_.speedBump().attributeOr("height", 0.5));
  if (speed_bump_reg_elem_.speedBump().hasAttribute("slow_down_speed")) {
    speed_bump_slow_down_speed_ = static_cast<float>(
      speed_bump_reg_elem_.speedBump().attribute("slow_down_speed").asDouble().get() / 3.6);
  } else {
    geometry_msgs::Point32 p1;
    geometry_msgs::Point32 p2;
    p1.x = planner_param_.speed_calculation_min_height;
    p2.x = planner_param_.speed_calculation_max_height;
    p1.y = planner_param_.speed_calculation_max_speed;
    p2.y = planner_param_.speed_calculation_min_speed;
    speed_bump_slow_down_speed_ = calcSlowDownSpeed(p1, p2, height);
  }
  if (planner_param_.print_debug_info) {
    std::cout << "Speed Bump ID: " << module_id << " height [cm]: " << height * 100.0F
              << " slow speed [kph]: " << speed_bump_slow_down_speed_ * 3.6F << std::endl;
  }
}

bool SpeedBumpModule::modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId * path)
{
  if (!path || path->points.empty()) return false;
  debug_data_ = SpeedBumpDebugData{};
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto polygon = lanelet::utils::to2D(speed_bump_reg_elem_.speedBump()).basicPolygon();
  const auto status = getPathPolygonIntersectionStatus(*path, polygon, ego_pos, 2);
  debug_data_.path_polygon_intersection_status = status;
  for (const auto & p : speed_bump_reg_elem_.speedBump().basicPolygon()) {
    debug_data_.speed_bump_polygon.push_back(create_point(p.x(), p.y(), ego_pos.z));
  }
  return applySlowDownSpeed(*path, speed_bump_slow_down_speed_, status);
}

bool SpeedBumpModule::applySlowDownSpeed(
  autoware_internal_planning_msgs::PathWithLaneId & output, const float speed,
  const PathPolygonIntersectionStatus & status)
{
  if (isNoRelation(status)) return false;
  size_t start_idx{};
  if (status.first_intersection_point) {
    const auto & src = *status.first_intersection_point;
    const double offset =
      -(planner_data_->vehicle_info_.max_longitudinal_offset_m + planner_param_.slow_start_margin);
    const auto idx = insertPointWithOffset(src, offset, output.points, 5e-2);
    if (idx) {
      start_idx = *idx;
      debug_data_.slow_start_poses.push_back(output.points.at(start_idx).point.pose);
    } else if (calcSignedArcLength(output.points, src, 0) > offset) {
      start_idx = 0;
    } else {
      return false;
    }
  } else if (status.second_intersection_point || status.is_path_inside_of_polygon) {
    start_idx = 0;
  }

  size_t end_idx{};
  if (status.second_intersection_point) {
    const auto & src = *status.second_intersection_point;
    const double offset = planner_data_->vehicle_info_.rear_overhang_m + planner_param_.slow_end_margin;
    const auto idx = insertPointWithOffset(src, offset, output.points, 5e-2);
    if (idx) {
      end_idx = *idx;
      debug_data_.slow_end_points.push_back(output.points.at(end_idx).point.pose.position);
    } else if (calcSignedArcLength(output.points, src, output.points.size() - 1) < offset) {
      end_idx = output.points.size() - 1;
    } else {
      return false;
    }
  } else if (status.first_intersection_point || status.is_path_inside_of_polygon) {
    end_idx = output.points.size() - 1;
  }
  return insertConstSpeedToPathSection(output.points, start_idx, end_idx, speed);
}
}  // namespace autoware::behavior_velocity_planner
