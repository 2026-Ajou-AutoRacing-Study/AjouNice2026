#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include <autoware_internal_planning_msgs/PlanningFactorArray.h>
#include <autoware_internal_planning_msgs/RouteState.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <tier4_planning_msgs/StopReasonArray.h>

class QLabel;
class QTimer;

namespace trajectory_validator
{

class PlanningStatusPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit PlanningStatusPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void refreshPanel();

private:
  struct RateTracker
  {
    void record(const ros::WallTime & now);
    double rateHz() const;
    bool isFresh(const ros::WallTime & now, double timeout_sec) const;

    std::deque<ros::WallTime> samples;
  };

  void kinematicCallback(const nav_msgs::Odometry::ConstPtr & message);
  void trajectoryCallback(const autoware_planning_msgs::Trajectory::ConstPtr & message);
  void moduleStatusCallback(const std_msgs::String::ConstPtr & message);
  void motionStatusCallback(const std_msgs::String::ConstPtr & message);
  void planningFactorsCallback(
    const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & message);
  void stopReasonsCallback(
    const tier4_planning_msgs::StopReasonArray::ConstPtr & message);
  void routeStateCallback(
    const autoware_internal_planning_msgs::RouteState::ConstPtr & message);
  void routeCallback(const autoware_planning_msgs::LaneletRoute::ConstPtr & message);

  void updateDecisionFromModuleStatus(const std::string & status);
  void updateDecisionFromPlanningFactors(
    const autoware_internal_planning_msgs::PlanningFactorArray & message);
  void updateTrajectorySpeed();
  void setStatusLabel(QLabel * label, const QString & text, const QString & level);

  ros::NodeHandle node_handle_;
  ros::Subscriber kinematic_subscriber_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber module_status_subscriber_;
  ros::Subscriber motion_status_subscriber_;
  ros::Subscriber planning_factors_subscriber_;
  ros::Subscriber stop_reasons_subscriber_;
  ros::Subscriber route_state_subscriber_;
  ros::Subscriber route_subscriber_;

  QTimer * refresh_timer_;
  QLabel * kinematic_rate_label_;
  QLabel * trajectory_rate_label_;
  QLabel * vehicle_speed_label_;
  QLabel * trajectory_speed_label_;
  QLabel * behavior_label_;
  QLabel * decision_label_;
  QLabel * route_label_;

  mutable std::mutex mutex_;
  RateTracker kinematic_rate_;
  RateTracker trajectory_rate_;
  double vehicle_speed_kph_ = 0.0;
  double vehicle_position_x_ = 0.0;
  double vehicle_position_y_ = 0.0;
  double trajectory_speed_kph_ = 0.0;
  autoware_planning_msgs::Trajectory latest_trajectory_;
  bool kinematic_state_received_ = false;
  bool trajectory_speed_received_ = false;
  std::string bpp_behavior_ = "WAITING";
  std::string bpp_decision_ = "Waiting for planner status";
  std::string factor_behavior_;
  std::string factor_decision_;
  std::string stop_decision_;
  std::string motion_status_;
  bool factors_received_ = false;
  bool stop_reasons_received_ = false;
  bool route_state_received_ = false;
  bool route_received_ = false;
  std::uint8_t route_state_ = 0;
  std::size_t route_segment_count_ = 0;
  ros::WallTime bpp_status_time_;
  ros::WallTime factor_time_;
  ros::WallTime stop_reason_time_;
  ros::WallTime motion_status_time_;
};

}  // namespace trajectory_validator
