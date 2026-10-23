#include "bringup/lanelet_speed_tool.hpp"
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/properties/float_property.h>
#include <rviz/viewport_mouse_event.h>
#include <OGRE/OgrePlane.h>
#include <QCursor>
namespace bringup
{
LaneletSpeedTool::LaneletSpeedTool()
{
  shortcut_key_ = 'l';
  height_ = new rviz::FloatProperty("Map height", 28.4,
    "Map click plane height. Use the top-down view for lanelet selection.", getPropertyContainer());
}
void LaneletSpeedTool::onInitialize()
{
  setName("Lanelet Speed");
  setCursor(QCursor(Qt::CrossCursor));
  points_ = node_.advertise<geometry_msgs::PointStamped>("/planning/lanelet_speed_editor/click", 10);
}
void LaneletSpeedTool::activate()
{
  setStatus("도로 내부를 클릭해 50 km/h 적용을 선택/해제하세요. Esc로 종료합니다.");
}
int LaneletSpeedTool::processMouseEvent(rviz::ViewportMouseEvent & event)
{
  if (!event.leftUp()) return 0;
  if (context_->getFixedFrame() != "map") return 0;
  Ogre::Plane plane(Ogre::Vector3::UNIT_Z, Ogre::Vector3(0, 0, height_->getFloat()));
  Ogre::Vector3 point;
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, point)) {
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.point.x = point.x; msg.point.y = point.y; msg.point.z = point.z;
    points_.publish(msg);
  }
  return Render;
}
}
PLUGINLIB_EXPORT_CLASS(bringup::LaneletSpeedTool, rviz::Tool)
