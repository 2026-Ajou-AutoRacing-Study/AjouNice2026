#include "remaining_distance_time_display.hpp"
#include <cmath>

namespace autoware::mission_details_overlay_rviz_plugin {
void RemainingDistanceTimeDisplay::update(const autoware_internal_msgs::MissionRemainingDistanceTime::ConstPtr & msg) {
  distance_ = msg->remaining_distance; time_ = msg->remaining_time;
}
void RemainingDistanceTimeDisplay::draw(QPainter & painter, const QRectF & rect, const QColor & color) const {
  painter.setPen(color); painter.setFont(QFont("Sans Serif", 15, QFont::Bold));
  const auto dist = distance_ <= 1000.0 ? QString::number(distance_, 'f', distance_ <= 10.0 ? 1 : 0) + " m" : QString::number(distance_ / 1000.0, 'f', distance_ <= 10000.0 ? 2 : 1) + " km";
  const auto time = time_ <= 60.0 ? QString::number(time_, 'f', 0) + " sec" : time_ <= 3600.0 ? QString::number(time_ / 60.0, 'f', time_ <= 600.0 ? 1 : 0) + " min" : QString::number(time_ / 3600.0, 'f', time_ <= 36000.0 ? 1 : 0) + " hrs";
  painter.drawText(QRectF(rect.left()+16, rect.top()+12, rect.width()-32, rect.height()/2-8), Qt::AlignRight|Qt::AlignVCenter, dist);
  painter.drawText(QRectF(rect.left()+16, rect.center().y(), rect.width()-32, rect.height()/2-12), Qt::AlignRight|Qt::AlignVCenter, time);
}
}
