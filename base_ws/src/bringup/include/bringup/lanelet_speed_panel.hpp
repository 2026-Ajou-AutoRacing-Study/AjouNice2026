#pragma once
#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QJsonObject>
class QLabel;
class QListWidget;
namespace bringup
{
class LaneletSpeedPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit LaneletSpeedPanel(QWidget * parent = nullptr);
Q_SIGNALS:
  void statusReceived(const QString & value);
private Q_SLOTS:
  void applyStatus(const QString & value);
private:
  void command(const QJsonObject & value);
  ros::NodeHandle node_;
  ros::Publisher commands_;
  ros::Subscriber status_;
  QListWidget * lanes_;
  QLabel * message_;
  QLabel * path_;
};
}
