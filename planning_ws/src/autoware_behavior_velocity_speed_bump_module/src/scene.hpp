#pragma once
#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <optional>
namespace autoware::behavior_velocity_planner::experimental
{
class SpeedBumpModule : public SceneModuleInterface
{
public:
  struct PlannerParam {
    double slow_start_margin{1.0}; double slow_end_margin{1.0}; bool print_debug_info{false};
    float min_height{0.0F}; float max_height{0.2F}; float min_speed{1.0F}; float max_speed{5.0F};
  };
  SpeedBumpModule(lanelet::Id id, const lanelet::autoware::SpeedBump & speed_bump,
    const PlannerParam & param, const ros::NodeHandle & node,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface);
  bool modifyPathVelocity(Trajectory & path, const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & data) override;
  visualization_msgs::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;
private:
  lanelet::Id id_;
  const lanelet::autoware::SpeedBump & speed_bump_;
  PlannerParam param_;
  float slow_speed_{0.0F};
  std::optional<double> slow_start_s_;
  std::optional<double> slow_end_s_;
  std::optional<geometry_msgs::Pose> slow_start_pose_;
  static float calculateSpeed(float h, const PlannerParam & param);
};
}  // namespace autoware::behavior_velocity_planner::experimental
