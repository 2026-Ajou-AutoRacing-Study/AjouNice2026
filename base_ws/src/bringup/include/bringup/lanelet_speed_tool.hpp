#pragma once
#include <rviz/tool.h>
#include <ros/ros.h>
namespace rviz { class FloatProperty; }
namespace bringup
{
class LaneletSpeedTool : public rviz::Tool
{
public:
  LaneletSpeedTool();
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
