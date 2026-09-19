#pragma once

#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_utils_debug/ros1_adapters.hpp>

#include <ros/ros.h>
#include <builtin_interfaces/Time.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// This adapter deliberately maps only the runtime surface used by this node.
// `~/` is always resolved through the private NodeHandle and a transient-local
// ROS2 publisher becomes a latched ROS1 publisher.
namespace rclcpp
{
inline builtin_interfaces::Time to_builtin_time(const ros::Time & time)
{
  builtin_interfaces::Time value;
  value.sec = static_cast<std::int32_t>(time.sec);
  value.nanosec = time.nsec;
  return value;
}

class Logger
{
public:
  explicit Logger(std::string name) : name_(std::move(name)) {}
  Logger get_child(const std::string & child) const { return Logger(name_ + "." + child); }
  const std::string & get_name() const { return name_; }

private:
  std::string name_;
};

inline Logger get_logger(const std::string & name) { return Logger(name); }
inline bool ok() { return ros::ok(); }
using Time = ros::Time;

class Duration
{
public:
  static ros::Duration from_seconds(const double seconds) { return ros::Duration(seconds); }
};

class Clock
{
public:
  using SharedPtr = std::shared_ptr<Clock>;
  ros::Time now() const { return ros::Time::now(); }
};

class QoS
{
public:
  explicit QoS(std::uint32_t depth) : depth_(depth) {}
  QoS transient_local() const { QoS result(*this); result.latch_ = true; return result; }
  std::uint32_t depth() const { return depth_; }
  bool latch() const { return latch_; }
private:
  std::uint32_t depth_;
  bool latch_{false};
};

template <typename MessageT> class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;
  explicit Publisher(autoware_utils_debug::ros1::IdentifiedPublisher<MessageT> publisher)
  : publisher_(std::move(publisher)) {}
  void publish(const MessageT & message) const { publisher_.publish(message); }
  const autoware_utils_debug::ros1::IdentifiedPublisher<MessageT> & identified() const
  { return publisher_; }
  const ros::Publisher & raw() const { return publisher_.raw(); }
private:
  autoware_utils_debug::ros1::IdentifiedPublisher<MessageT> publisher_;
};

template <typename MessageT> class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;
  explicit Subscription(ros::Subscriber subscriber) : subscriber_(std::move(subscriber)) {}
private:
  ros::Subscriber subscriber_;
};

class Parameter
{
public:
  Parameter() = default;
  Parameter(std::string name, XmlRpc::XmlRpcValue value) : name_(std::move(name)), value_(std::move(value)) {}
  const std::string & get_name() const { return name_; }
  double as_double() const
  {
    if (value_.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value_);
    if (value_.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value_);
    throw std::runtime_error("parameter is not numeric: " + name_);
  }
  bool as_bool() const
  {
    if (value_.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
      throw std::runtime_error("parameter is not bool: " + name_);
    }
    return static_cast<bool>(value_);
  }
  template <typename T> T get_value() const
  {
    if constexpr (std::is_same_v<T, double>) return as_double();
    if constexpr (std::is_same_v<T, int>) return static_cast<int>(value_);
    if constexpr (std::is_same_v<T, bool>) return as_bool();
    if constexpr (std::is_same_v<T, std::string>) return static_cast<std::string>(value_);
    if constexpr (std::is_same_v<T, std::vector<double>>) {
      if (value_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error("parameter is not a numeric array: " + name_);
      }
      std::vector<double> values;
      values.reserve(value_.size());
      for (int index = 0; index < value_.size(); ++index) {
        const auto & entry = value_[index];
        if (entry.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          values.push_back(static_cast<double>(entry));
        } else if (entry.getType() == XmlRpc::XmlRpcValue::TypeInt) {
          values.push_back(static_cast<int>(entry));
        } else {
          throw std::runtime_error("parameter array contains a non-numeric value: " + name_);
        }
      }
      return values;
    }
    throw std::runtime_error("unsupported dynamic parameter type: " + name_);
  }
private:
  std::string name_;
  XmlRpc::XmlRpcValue value_;
};

class Node
{
public:
  class OnSetParametersCallbackHandle
  {
  public:
    using SharedPtr = std::shared_ptr<OnSetParametersCallbackHandle>;
  };

  explicit Node(const ros::NodeHandle & private_node_handle)
  : node_handle_(std::make_shared<ros::NodeHandle>()), private_node_handle_(private_node_handle) {}
  Node(const ros::NodeHandle & node_handle, const ros::NodeHandle & private_node_handle)
  : node_handle_(std::make_shared<ros::NodeHandle>(node_handle)), private_node_handle_(private_node_handle) {}

  template <typename T>
  T declare_parameter(const std::string & name)
  {
    const auto ros1_name = normalize_parameter(name);
    if (!private_node_handle_.hasParam(ros1_name)) {
      throw std::runtime_error("required ROS parameter is missing: " + name);
    }
    return autoware_utils_rclcpp::detail::strict_get<T>(private_node_handle_, ros1_name);
  }

  template <typename T>
  T declare_parameter(const std::string & name, const T & default_value)
  {
    const auto ros1_name = normalize_parameter(name);
    if (!private_node_handle_.hasParam(ros1_name)) {
      return default_value;
    }
    return autoware_utils_rclcpp::detail::strict_get<T>(private_node_handle_, ros1_name);
  }

  Logger get_logger() const { return Logger(private_node_handle_.getNamespace()); }
  const ros::NodeHandle & node_handle() const { return *node_handle_; }
  const std::shared_ptr<ros::NodeHandle> & node_handle_owner() const { return node_handle_; }
  ros::NodeHandle * node_handle_ptr() { return node_handle_.get(); }
  ros::NodeHandle * private_node_handle_ptr() { return &private_node_handle_; }
  ros::NodeHandle & private_node_handle() { return private_node_handle_; }
  const ros::NodeHandle & private_node_handle() const { return private_node_handle_; }
  ros::Time now() const { return ros::Time::now(); }
  Clock::SharedPtr get_clock() const { return std::make_shared<Clock>(); }

  template <typename MessageT>
  typename Publisher<MessageT>::SharedPtr create_publisher(const std::string & topic, const std::uint32_t depth)
  { return create_publisher<MessageT>(topic, QoS{depth}); }
  template <typename MessageT>
  typename Publisher<MessageT>::SharedPtr create_publisher(const std::string & topic, const QoS & qos)
  {
    auto & handle = private_topic(topic) ? private_node_handle_ : *node_handle_;
    const std::string resolved = private_topic(topic) ? topic.substr(2) : topic;
    autoware_utils_debug::ros1::PublisherFactory factory;
    return std::make_shared<Publisher<MessageT>>(factory.advertise<MessageT>(handle, resolved, qos.depth(), qos.latch()));
  }
  template <typename MessageT, typename CallbackT>
  typename Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic, const std::uint32_t depth, CallbackT && callback)
  {
    auto & handle = private_topic(topic) ? private_node_handle_ : *node_handle_;
    const std::string resolved = private_topic(topic) ? topic.substr(2) : topic;
    return std::make_shared<Subscription<MessageT>>(handle.subscribe<MessageT>(resolved, depth, std::forward<CallbackT>(callback)));
  }
  Parameter get_parameter(const std::string & name) const
  {
    const auto ros1_name = normalize_parameter(name);
    XmlRpc::XmlRpcValue value;
    if (!private_node_handle_.getParam(ros1_name, value)) {
      throw std::runtime_error("required velocity_smoother parameter is missing: " + name);
    }
    return Parameter(name, value);
  }
  template <typename CallbackT>
  OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(CallbackT &&)
  {
    // ROS1 parameter-server updates are not callback-driven. The concrete
    // node explicitly snapshots and validates the supported runtime parameter
    // set immediately before handling its next input trajectory.
    return std::make_shared<OnSetParametersCallbackHandle>();
  }

private:
  static bool private_topic(const std::string & topic) { return topic.rfind("~/", 0) == 0; }
  static std::string normalize_parameter(std::string name)
  {
    std::replace(name.begin(), name.end(), '.', '/');
    return name;
  }
  std::shared_ptr<ros::NodeHandle> node_handle_;
  ros::NodeHandle private_node_handle_;
};
}  // namespace rclcpp

namespace autoware_utils_geometry
{
template <typename T>
inline void update_param(const std::vector<rclcpp::Parameter> &, const std::string &, T &)
{
  // ROS1 has no parameter-event callback. The node retains its construction-time snapshot.
}
}  // namespace autoware_utils_geometry

namespace autoware_utils
{
using namespace autoware_utils_geometry;
template <typename T>
inline void update_param(const std::vector<rclcpp::Parameter> &, const std::string &, T &)
{
  // ROS1 has no parameter-event callback. The node retains its construction-time snapshot.
}
}  // namespace autoware_utils

namespace rcl_interfaces::msg
{
struct SetParametersResult { bool successful{false}; std::string reason; };
}

#ifdef RCLCPP_DEBUG
#undef RCLCPP_DEBUG
#endif
#ifdef RCLCPP_INFO
#undef RCLCPP_INFO
#endif
#ifdef RCLCPP_WARN
#undef RCLCPP_WARN
#endif
#ifdef RCLCPP_ERROR
#undef RCLCPP_ERROR
#endif
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) ROS_INFO_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_WARN_STREAM(logger, args) ROS_WARN_STREAM_NAMED((logger).get_name().c_str(), args)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) ROS_WARN_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) ROS_INFO_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_INFO_SKIPFIRST_THROTTLE(logger, clock, ms, ...) ROS_INFO_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_INFO_EXPRESSION(logger, condition, ...) do { if (condition) { ROS_INFO_NAMED((logger).get_name().c_str(), __VA_ARGS__); } } while (0)
