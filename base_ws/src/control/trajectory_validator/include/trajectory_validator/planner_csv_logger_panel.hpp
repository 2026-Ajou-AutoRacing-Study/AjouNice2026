#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Trigger.h>

class QLabel;
class QPushButton;

namespace trajectory_validator
{

class PlannerCsvLoggerPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit PlannerCsvLoggerPanel(QWidget * parent = nullptr);

private Q_SLOTS:
  void startLogging();
  void stopLogging();

private:
  void setStatus(const QString & text, const QString & color);

  ros::NodeHandle node_handle_;
  ros::ServiceClient start_client_;
  ros::ServiceClient stop_client_;
  QPushButton * start_button_;
  QPushButton * stop_button_;
  QLabel * status_label_;
};

}  // namespace trajectory_validator
