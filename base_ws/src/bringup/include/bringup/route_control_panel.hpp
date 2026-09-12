#ifndef BRINGUP__ROUTE_CONTROL_PANEL_HPP_
#define BRINGUP__ROUTE_CONTROL_PANEL_HPP_

#include <rviz/panel.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QString>

#include <string>

class QLabel;
class QPushButton;

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
  void requestRouteA();
  void applyStatus(const QString & status);

private:
  void onStatus(const std_msgs::String::ConstPtr & message);

  ros::NodeHandle node_handle_;
  ros::ServiceClient request_client_;
  ros::Subscriber status_subscriber_;
  QLabel * status_label_;
  QPushButton * request_button_;
  std::string request_service_name_;
};

}  // namespace bringup

#endif  // BRINGUP__ROUTE_CONTROL_PANEL_HPP_
