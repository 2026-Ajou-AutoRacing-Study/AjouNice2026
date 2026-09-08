#pragma once
#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/velocity_smoother/ros1_compat.hpp>
#include <autoware_utils_debug/processing_time_publisher.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <builtin_interfaces/Time.h>
#include <ros/ros.h>
#include <cmath>
#include <memory>
#include <stdexcept>

#ifndef RCLCPP_DEBUG_STREAM
#define RCLCPP_DEBUG_STREAM(logger, args) ROS_DEBUG_STREAM(args)
#endif
#ifndef RCLCPP_INFO_STREAM
#define RCLCPP_INFO_STREAM(logger, args) ROS_INFO_STREAM(args)
#endif
#ifndef RCLCPP_ERROR_STREAM
#define RCLCPP_ERROR_STREAM(logger, args) ROS_ERROR_STREAM(args)
#endif

namespace autoware::motion_velocity_planner::full_native {
inline ros::Time time(const builtin_interfaces::Time & stamp) {
  if(stamp.sec<0 || stamp.nanosec>=1000000000U) throw std::invalid_argument("Invalid ROS1 timestamp");
  return ros::Time(static_cast<uint32_t>(stamp.sec),stamp.nanosec);
}
struct Clock { ros::Time now() const {return ros::Time::now();} };
class Node:public ros::NodeHandle {
public:
  explicit Node(ros::NodeHandle node):ros::NodeHandle(std::move(node)),
    debug_owner_(std::make_shared<ros::NodeHandle>(*this)),
    debug_context_(autoware_utils_debug::ros1::CallerNodeContext::borrow(debug_owner_)),
    smoother_parameters_(*this) {}
  template<class T>T declare_parameter(const std::string & key) {
    return autoware_utils_rclcpp::detail::strict_get<T>(*this,autoware_utils_rclcpp::to_ros1_parameter_name(key));
  }
  rclcpp::Logger get_logger() const {return rclcpp::get_logger(getNamespace());}
  std::shared_ptr<Clock> get_clock() const {return clock_;}
protected:
  std::shared_ptr<ros::NodeHandle> debug_owner_;
  autoware_utils_debug::ros1::CallerNodeContext::Ptr debug_context_;
  rclcpp::Node smoother_parameters_;
private:
  std::shared_ptr<Clock> clock_=std::make_shared<Clock>();
};
inline void flatten(const std::string & prefix,XmlRpc::XmlRpcValue value,ParameterMap & output) {
  if(value.getType()==XmlRpc::XmlRpcValue::TypeStruct) {
    for(auto it=value.begin();it!=value.end();++it)
      flatten(prefix.empty()?it->first:prefix+"/"+it->first,it->second,output);
  } else output.emplace(prefix,value);
}
inline ParameterMap parameters(const ros::NodeHandle & node) {
  XmlRpc::XmlRpcValue tree; ParameterMap result;
  if(node.getParam("",tree)) flatten("",tree,result);
  return result;
}
template<class T> void update_param(const ParameterMap & parameters,const std::string & key,T & value) {
  auto it=parameters.find(autoware_utils_rclcpp::to_ros1_parameter_name(key));
  if(it==parameters.end()) it=parameters.find(key);
  if(it==parameters.end()) return;
  if(it->second.getType()!=autoware_utils_rclcpp::detail::xmlrpc_type<T>::value)
    throw std::invalid_argument("Invalid parameter type: "+key);
  value=static_cast<T>(it->second);
}
}
