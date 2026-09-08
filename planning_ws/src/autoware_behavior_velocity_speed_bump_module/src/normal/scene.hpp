#pragma once

#include "../util.hpp"

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner
{
class SpeedBumpModule : public SceneModuleInterface
{
public:
  struct SpeedBumpDebugData
  {
    double base_link2front{0.0};
    PathPolygonIntersectionStatus path_polygon_intersection_status;
    std::vector<geometry_msgs::Pose> slow_start_poses;
    std::vector<geometry_msgs::Point> slow_end_points;
    std::vector<geometry_msgs::Point> speed_bump_polygon;
  };

  struct PlannerParam
  {
    double slow_start_margin{0.0};
    double slow_end_margin{0.0};
    bool print_debug_info{false};
    float speed_calculation_min_height{0.0F};
    float speed_calculation_max_height{0.0F};
    float speed_calculation_min_speed{0.0F};
    float speed_calculation_max_speed{0.0F};
  };

  SpeedBumpModule(
    int64_t module_id, int64_t lane_id, const lanelet::autoware::SpeedBump & speed_bump_reg_elem,
    const PlannerParam & planner_param, const ros::NodeHandle & node,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface);

  bool modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId * path) override;
  visualization_msgs::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

private:
  int64_t lane_id_;
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem_;
  PlannerParam planner_param_;
  SpeedBumpDebugData debug_data_;
  float speed_bump_slow_down_speed_{0.0F};

  bool applySlowDownSpeed(
    autoware_internal_planning_msgs::PathWithLaneId & output, float speed_bump_speed,
    const PathPolygonIntersectionStatus & status);
};
}  // namespace autoware::behavior_velocity_planner
