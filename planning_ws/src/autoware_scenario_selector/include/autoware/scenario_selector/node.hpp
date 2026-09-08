#pragma once

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <deque>
#include <memory>
#include <optional>
#include <string>

namespace autoware::scenario_selector
{
class ScenarioSelectorNode
{
public:
  explicit ScenarioSelectorNode(const ros::NodeHandle & node);
#ifdef AW149_SCENARIO_SELECTOR_TESTING
  void testSetScenario(const std::string & scenario) { current_scenario_ = scenario; }
  const std::string & testScenario() const { return current_scenario_; }
  void testOnRoute(const autoware_planning_msgs::LaneletRoute::ConstPtr & route) { onRoute(route); }
#endif

private:
  void onMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & message);
  void onRoute(const autoware_planning_msgs::LaneletRoute::ConstPtr & message);
  void onOdom(const nav_msgs::Odometry::ConstPtr & message);
  void onLaneDrivingTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & message);
  void onParkingTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & message);
  void onParkingCompleted(const std_msgs::Bool::ConstPtr & message);
  void onOperationMode(const autoware_adapi_v1_msgs::OperationModeState::ConstPtr & message);
  void onTimer(const ros::TimerEvent &);
  void updatePolledData();
  void applyOdom(const nav_msgs::Odometry::ConstPtr & message);

  bool isDataReady();
  void updateCurrentScenario();
  std::string selectScenarioByPosition();
  bool isSwitchToParking(bool is_stopped);
  bool isSwitchToLaneDriving();
  bool isAutonomous() const;
  bool isEmptyParkingTrajectory() const;
  autoware_planning_msgs::Trajectory::ConstPtr getScenarioTrajectory(const std::string & scenario) const;
  void publishTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & message);

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber map_sub_;
  ros::Subscriber route_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber lane_driving_sub_;
  ros::Subscriber parking_sub_;
  ros::Subscriber parking_completed_sub_;
  ros::Subscriber operation_mode_sub_;
  ros::Publisher scenario_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher processing_time_pub_;
  ros::Timer timer_;

  autoware_adapi_v1_msgs::OperationModeState::ConstPtr operation_mode_state_;
  autoware_planning_msgs::Trajectory::ConstPtr lane_driving_trajectory_;
  autoware_planning_msgs::Trajectory::ConstPtr parking_trajectory_;
  autoware_planning_msgs::LaneletRoute::ConstPtr route_;
  nav_msgs::Odometry::ConstPtr current_pose_;
  geometry_msgs::TwistStamped::ConstPtr twist_;
  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_buffer_;
  std::deque<nav_msgs::Odometry::ConstPtr> pending_odometry_;
  std::optional<bool> pending_parking_completed_;
  autoware_adapi_v1_msgs::OperationModeState::ConstPtr pending_operation_mode_state_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;

  std::string current_scenario_;
  bool is_parking_completed_{false};
  std::optional<ros::Time> lane_driving_stop_time_;
  std::optional<ros::Time> empty_parking_trajectory_time_;
  double update_rate_{10.0};
  double th_max_message_delay_sec_{1.0};
  double th_arrived_distance_m_{1.0};
  double th_stopped_time_sec_{1.0};
  double th_stopped_velocity_mps_{0.01};
  bool enable_mode_switching_{true};
};
}  // namespace autoware::scenario_selector
