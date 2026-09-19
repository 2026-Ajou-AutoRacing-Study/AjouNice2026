#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_

#include <ros/ros.h>
#include <memory>

namespace autoware::component_interface_utils
{
template <class SpecT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher>;
  using SpecType = SpecT;
  explicit Publisher(const ros::Publisher & publisher) : publisher_(publisher) {}
  void publish(const typename SpecT::Message & msg) { publisher_.publish(msg); }
private:
  ros::Publisher publisher_;
};
}  // namespace autoware::component_interface_utils
#endif
