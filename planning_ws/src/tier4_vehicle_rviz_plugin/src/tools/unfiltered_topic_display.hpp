#ifndef TIER4_VEHICLE_RVIZ_PLUGIN_UNFILTERED_TOPIC_DISPLAY_HPP_
#define TIER4_VEHICLE_RVIZ_PLUGIN_UNFILTERED_TOPIC_DISPLAY_HPP_

#include <ros/ros.h>
#include <ros/message_traits.h>
#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>

#include <memory>

namespace rviz_plugins
{
// RViz1 MessageFilterDisplay requires a standard Header.  The ROS2 source uses
// RosTopicDisplay for vehicle reports that deliberately have no Header, so the
// ROS1 candidate retains an ordinary, selectable subscription without TF filtering.
template <typename Message>
class UnfilteredTopicDisplay : public rviz::Display
{
public:
  void onInitialize() override
  {
    rviz::Display::onInitialize();
    node_handle_ = std::make_unique<ros::NodeHandle>();
    topic_property_ = new rviz::RosTopicProperty(
      "Topic", "", ros::message_traits::datatype<Message>(),
      "Vehicle-report topic (unfiltered because this message has no standard Header).", this,
      [this]() { subscribe(); }, this);
    subscribe();
  }

  void subscribe()
  {
    unsubscribe();
    const auto topic = topic_property_ ? topic_property_->getTopicStd() : std::string{};
    if (node_handle_ && !topic.empty()) {
      subscriber_ = node_handle_->subscribe(topic, 10, &UnfilteredTopicDisplay::processMessage, this);
    }
  }

  void unsubscribe() { subscriber_.shutdown(); }

protected:
  virtual void processMessage(const typename Message::ConstPtr message) = 0;

private:
  std::unique_ptr<ros::NodeHandle> node_handle_;
  ros::Subscriber subscriber_;
  rviz::RosTopicProperty * topic_property_{nullptr};
};
}  // namespace rviz_plugins

#endif
