#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_

#include <ros/ros.h>
#include <memory>
#include <mutex>

namespace autoware::component_interface_utils
{
template <class SpecT>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription>;
  using SpecType = SpecT;
  Subscription() = default;
  void reset(const ros::Subscriber & subscriber) { subscriber_ = subscriber; }
  void store(const typename SpecT::Message::ConstPtr & message) { std::lock_guard<std::mutex> lock(mutex_); latest_ = message; }
  typename SpecT::Message::ConstPtr take()
  {
    std::lock_guard<std::mutex> lock(mutex_); auto value = latest_; latest_.reset(); return value;
  }
  bool take_and_update(typename SpecT::Message::ConstPtr & out) { auto value=take(); if(!value) return false; out=value; return true; }
  bool take_and_update(typename SpecT::Message & out) { auto value=take(); if(!value) return false; out=*value; return true; }
private:
  ros::Subscriber subscriber_;
  std::mutex mutex_;
  typename SpecT::Message::ConstPtr latest_;
};
}  // namespace autoware::component_interface_utils
#endif
