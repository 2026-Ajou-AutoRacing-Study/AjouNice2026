#ifndef BRINGUP__ROUTE_CONTROL_PANEL_HPP_
#define BRINGUP__ROUTE_CONTROL_PANEL_HPP_

#include <rviz/panel.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QString>

class QLabel;

namespace bringup
{

class RouteControlPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit RouteControlPanel(QWidget * parent = nullptr);

Q_SIGNALS:
  void statusReceived(const QString & status);

private Q_SLOTS:
  void applyStatus(const QString & status);

private:
  void onStatus(const std_msgs::String::ConstPtr & message);
  void setActiveRoute(const QString & route_key);

  ros::NodeHandle node_handle_;
  ros::Subscriber status_subscriber_;
  QLabel * route_a_indicator_;
  QLabel * route_b_indicator_;
  QLabel * status_label_;
};

}  // namespace bringup

#endif  // BRINGUP__ROUTE_CONTROL_PANEL_HPP_
