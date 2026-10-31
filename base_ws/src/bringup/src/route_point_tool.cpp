#include "bringup/route_point_tool.hpp"
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
RoutePointTool::RoutePointTool()
{
  shortcut_key_ = 'p';
  height_ = new rviz::FloatProperty("Map height", 28.4,
    "Height of the click plane in map. Points are snapped to the road by the editor.", getPropertyContainer());
}
void RoutePointTool::onInitialize()
{
  setName("Route Point");
  setCursor(QCursor(Qt::CrossCursor));
  points_ = node_.advertise<geometry_msgs::PointStamped>("/planning/route_editor/add_point", 10);
}
void RoutePointTool::activate()
{
  setStatus("도로 위를 클릭해 경유점을 추가하세요. Esc로 종료합니다.");
}
int RoutePointTool::processMouseEvent(rviz::ViewportMouseEvent & event)
{
  if (!event.leftUp()) return 0;
  if (context_->getFixedFrame() != "map") {
    setStatus("Fixed Frame을 map으로 설정하세요.");
    return 0;
  }
  Ogre::Plane plane(Ogre::Vector3::UNIT_Z, Ogre::Vector3(0, 0, height_->getFloat()));
  Ogre::Vector3 point;
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, point)) {
    geometry_msgs::PointStamped message;
    message.header.frame_id = "map";
    message.header.stamp = ros::Time::now();
    message.point.x = point.x; message.point.y = point.y; message.point.z = point.z;
    points_.publish(message);
  }
  return Render;
}
}
PLUGINLIB_EXPORT_CLASS(bringup::RoutePointTool, rviz::Tool)
