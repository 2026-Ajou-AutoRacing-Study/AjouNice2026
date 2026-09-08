#pragma once

#include <autoware_utils_debug/debug_traits.hpp>
#include <autoware_utils_debug/ros1_adapters.hpp>

#include <ros/message_traits.h>
#include <autoware_utils_debug/published_time_publisher.hpp>

#include <memory>
#include <string>
#include <type_traits>

namespace autoware_utils_debug
{
namespace debug_publisher
{
inline void assign_stamp(ros::Time & output, const ros::Time & input) { output=input; }
inline void assign_stamp(builtin_interfaces::Time & output, const ros::Time & input)
{ output=ros_time_to_builtin_time(input); }
template <class TMsg, class T,
  std::enable_if_t<debug_traits::is_debug_message<TMsg>::value, std::nullptr_t> = nullptr>
TMsg to_debug_msg(const T & data, const ros::Time & stamp)
{
  TMsg message;
  assign_stamp(message.stamp, stamp);
  message.data = data;
  return message;
}
}

class BasicDebugPublisher
{
public:
  BasicDebugPublisher(ros1::CallerNodeContext::Ptr context, const std::string & ns)
  : context_(std::move(context)), ns_(ns)
  {
    if (!context_) throw std::invalid_argument("DebugPublisher requires context");
  }
  template <typename T, std::enable_if_t<ros::message_traits::IsMessage<T>::value, std::nullptr_t> = nullptr>
  void publish(const std::string & name, const T & data)
  {
    context_->require_alive();
    publish_message(name, data);
  }
  template <typename TMsg, typename T,
    std::enable_if_t<!ros::message_traits::IsMessage<T>::value, std::nullptr_t> = nullptr>
  void publish(const std::string & name, const T & data)
  {
    publish_message(name, debug_publisher::to_debug_msg<TMsg>(data, ros::Time::now()));
  }
  std::size_t size() const { return publishers_.size(); }
private:
  template <typename T> void publish_message(const std::string & name, const T & message)
  {
    const auto type = std::type_index(typeid(T));
    auto it = publishers_.find(name);
    if (it != publishers_.end() && it->second.type != type) throw std::bad_cast();
    if (it == publishers_.end()) {
      context_->require_alive();
      Entry entry{type, context_->advertise_default<T>(ns_ + "/" + name)};
      it = publishers_.emplace(name, std::move(entry)).first;
    }
    context_->require_alive();
    it->second.publisher.publish(message);
  }
  ros1::CallerNodeContext::Ptr context_;
  std::string ns_;
  struct Entry { std::type_index type{typeid(void)}; ros::Publisher publisher; };
  std::unordered_map<std::string, Entry> publishers_;
};
using DebugPublisher = BasicDebugPublisher;
}
