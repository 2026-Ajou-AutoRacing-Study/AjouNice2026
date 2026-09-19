#pragma once

#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace autoware::behavior_path_planner
{
// Conservative ingress/output subset of the upstream BPP node.  It preserves
// upstream mandatory/cyclic readiness and LaneDriving gating, then delegates
// only to the staged static side-shift slot.  Plugin-manager, RTC, turn-signal,
// debug and full scene-module orchestration are intentionally not represented.
class BehaviorPathPlannerRos1Node
{
public:
  BehaviorPathPlannerRos1Node(ros::NodeHandle node, ros::NodeHandle private_node);
  // Test-visible state distinguishes a scene-module run from merely publishing
  // the route reference in the candidate transport oracle.
  size_t activeModuleCount() const { return core_.activeModuleCount(); }
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  bool staticObstacleAvoidanceEnabled() const { return core_.staticObstacleAvoidanceEnabled(); }
#endif

private:
  void onRoute(const autoware_planning_msgs::LaneletRouteConstPtr & message);
  void onMap(const autoware_map_msgs::LaneletMapBinConstPtr & message);
  void onScenario(const autoware_internal_planning_msgs::ScenarioConstPtr & message);
  void onAcceleration(const geometry_msgs::AccelWithCovarianceStampedConstPtr & message);
  void onOperationMode(const autoware_adapi_v1_msgs::OperationModeStateConstPtr & message);
  void onPerception(const autoware_perception_msgs::PredictedObjectsConstPtr & message);
  void onOdometry(const nav_msgs::OdometryConstPtr & message);
  void onOccupancyGrid(const nav_msgs::OccupancyGridConstPtr & message);
  void onLateralOffset(const tier4_planning_msgs::LateralOffsetConstPtr & message);
  void onTimer(const ros::TimerEvent &);
  void updateRouteHandler();
  std::optional<BehaviorModuleOutput> makeReferenceOutput() const;

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber route_subscriber_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber scenario_subscriber_;
  ros::Subscriber acceleration_subscriber_;
  ros::Subscriber operation_mode_subscriber_;
  ros::Subscriber perception_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber occupancy_grid_subscriber_;
  ros::Subscriber costmap_subscriber_;
  ros::Subscriber lateral_offset_subscriber_;
  ros::Publisher path_publisher_;
  ros::Timer timer_;

  std::shared_ptr<PlannerData> planner_data_{std::make_shared<PlannerData>()};
  BehaviorPathPlannerCore core_;
  BppInputState input_state_;
  autoware_planning_msgs::LaneletRouteConstPtr route_;
  autoware_map_msgs::LaneletMapBinConstPtr map_;
  autoware_internal_planning_msgs::ScenarioConstPtr scenario_;
  bool map_dirty_{false};
  bool route_dirty_{false};
  double cyclic_timeout_seconds_{0.5};
  bool enable_traffic_signal_timeout_{false};
};
}  // namespace autoware::behavior_path_planner
