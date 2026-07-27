#pragma once

#include <string>

#include <morai_msgs/EgoVehicleStatus.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Float64.h>
#include <trajectory_validator/TrajectoryDebug.h>

class QLabel;

namespace trajectory_validator
{

class PurePursuitHudPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit PurePursuitHudPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void setLookaheadText(const QString & text);
  void setSteeringText(const QString & text);
  void setSpeedText(const QString & text);
  void setLateralErrorText(const QString & text);
  void setHeadingErrorText(const QString & text);
  void setLateralErrorMaxText(const QString & text);
  void setHeadingErrorMaxText(const QString & text);

Q_SIGNALS:
  void lookaheadTextChanged(const QString & text);
  void steeringTextChanged(const QString & text);
  void speedTextChanged(const QString & text);
  void lateralErrorTextChanged(const QString & text);
  void headingErrorTextChanged(const QString & text);
  void lateralErrorMaxTextChanged(const QString & text);
  void headingErrorMaxTextChanged(const QString & text);

private:
  void lookaheadCallback(const std_msgs::Float64::ConstPtr & message);
  void steeringCallback(const std_msgs::Float64::ConstPtr & message);
  void speedCallback(const morai_msgs::EgoVehicleStatus::ConstPtr & message);
  void errorCallback(const trajectory_validator::TrajectoryDebug::ConstPtr & message);

  ros::NodeHandle node_handle_;
  ros::Subscriber lookahead_subscriber_;
  ros::Subscriber steering_subscriber_;
  ros::Subscriber speed_subscriber_;
  ros::Subscriber error_subscriber_;
  QLabel * lookahead_label_;
  QLabel * steering_label_;
  QLabel * speed_label_;
  QLabel * lateral_error_label_;
  QLabel * heading_error_label_;
  QLabel * lateral_error_max_label_;
  QLabel * heading_error_max_label_;
  std::string current_run_id_;
  double lateral_error_max_m_ = 0.0;
  double heading_error_max_rad_ = 0.0;
};

}  // namespace trajectory_validator
