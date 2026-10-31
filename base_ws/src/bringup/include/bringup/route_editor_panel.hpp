#ifndef BRINGUP_ROUTE_EDITOR_PANEL_HPP_
#define BRINGUP_ROUTE_EDITOR_PANEL_HPP_

#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QJsonObject>

class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;

namespace bringup
{
class RouteEditorPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit RouteEditorPanel(QWidget * parent = nullptr);
Q_SIGNALS:
  void statusReceived(const QString & status);
private Q_SLOTS:
  void applyStatus(const QString & status);
private:
  void command(const QJsonObject & value);
  ros::NodeHandle node_;
  ros::Publisher commands_;
  ros::Subscriber status_;
  QComboBox * route_;
  QListWidget * points_;
  QLineEdit * path_;
  QLabel * message_;
  QPushButton * apply_;
};
}  // namespace bringup
#endif
