#ifndef REMAINING_DISTANCE_TIME_DISPLAY_HPP_
#define REMAINING_DISTANCE_TIME_DISPLAY_HPP_
#include <autoware_internal_msgs/MissionRemainingDistanceTime.h>
#include <QColor>
#include <QPainter>
namespace autoware::mission_details_overlay_rviz_plugin {
class RemainingDistanceTimeDisplay {
public:
  void update(const autoware_internal_msgs::MissionRemainingDistanceTime::ConstPtr & msg);
  void draw(QPainter & painter, const QRectF & rect, const QColor & text_color) const;
private: double distance_{0.0}; double time_{0.0};
}; }
#endif
