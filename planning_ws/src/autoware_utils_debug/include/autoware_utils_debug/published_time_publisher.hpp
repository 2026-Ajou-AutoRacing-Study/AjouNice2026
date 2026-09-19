#pragma once

#include <autoware_utils_debug/ros1_adapters.hpp>

#include <autoware_internal_msgs/PublishedTime.h>
#include <builtin_interfaces/Time.h>
#include <std_msgs/Header.h>

#include <limits>
#include <string>
#include <utility>

namespace autoware_utils_debug
{
using PublishedTime = autoware_internal_msgs::PublishedTime;

inline builtin_interfaces::Time ros_time_to_builtin_time(const ros::Time & time)
{
  // ROS1 ros::Time cannot express negative source time.  Positive values beyond
  // the ROS2 signed-second domain are rejected rather than silently wrapping.
  if (time.sec > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::range_error("ROS1 time exceeds builtin_interfaces signed-second domain");
  }
  builtin_interfaces::Time result;
  result.sec = static_cast<std::int32_t>(time.sec);
  result.nanosec = time.nsec;
  return result;
}

class BasicPublishedTimePublisher
{
public:
  explicit BasicPublishedTimePublisher(
    ros1::CallerNodeContext::Ptr context, std::string suffix = "/debug/published_time")
  : context_(std::move(context)), cache_(context_, std::move(suffix))
  {
    if (!context_) throw std::invalid_argument("PublishedTimePublisher requires context");
  }
  template <typename PrimaryT>
  void publish_if_subscribed(const ros1::IdentifiedPublisher<PrimaryT> & primary, const ros::Time & stamp)
  {
    ros::Publisher & auxiliary = cache_.observe(primary);
    if (auxiliary.getNumSubscribers() == 0) return;
    PublishedTime message;
    message.header.stamp = stamp;
    message.published_stamp = ros_time_to_builtin_time(ros::Time::now());
    auxiliary.publish(message);
  }
  template <typename PrimaryT>
  void publish_if_subscribed(const ros1::IdentifiedPublisher<PrimaryT> & primary, const std_msgs::Header & header)
  {
    ros::Publisher & auxiliary = cache_.observe(primary);
    if (auxiliary.getNumSubscribers() == 0) return;
    PublishedTime message;
    message.header = header;
    message.published_stamp = ros_time_to_builtin_time(ros::Time::now());
    auxiliary.publish(message);
  }
  std::size_t size() const { return cache_.size(); }
private:
  ros1::CallerNodeContext::Ptr context_;
  ros1::PublishedTimeCache<PublishedTime> cache_;
};
using PublishedTimePublisher = BasicPublishedTimePublisher;
}
