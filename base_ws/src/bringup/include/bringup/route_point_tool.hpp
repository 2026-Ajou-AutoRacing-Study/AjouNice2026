#ifndef BRINGUP_ROUTE_POINT_TOOL_HPP_
#define BRINGUP_ROUTE_POINT_TOOL_HPP_
#include <rviz/tool.h>
#include <ros/ros.h>
namespace rviz { class FloatProperty; }
namespace bringup
{
class RoutePointTool : public rviz::Tool
{
public:
  RoutePointTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override {}
  int processMouseEvent(rviz::ViewportMouseEvent & event) override;
private:
  ros::NodeHandle node_;
  ros::Publisher points_;
  rviz::FloatProperty * height_;
};
}
#endif
