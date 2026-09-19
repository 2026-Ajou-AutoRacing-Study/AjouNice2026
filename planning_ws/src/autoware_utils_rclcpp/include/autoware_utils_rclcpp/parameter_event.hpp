// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <autoware_utils_rclcpp/parameter.hpp>
#include <algorithm>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware_utils_rclcpp::event_detail {
template<class T> struct is_vector : std::false_type {};
template<class T> struct is_vector<std::vector<T>> : std::true_type {};
template<class T> XmlRpc::XmlRpcValue encode(const T & value) {
  if constexpr(is_vector<T>::value) {
    XmlRpc::XmlRpcValue result;result.setSize(static_cast<int>(value.size()));
    for(size_t i=0;i<value.size();++i)result[static_cast<int>(i)]=encode<typename T::value_type>(value[i]);
    return result;
  } else {
    static_assert(detail::xmlrpc_type<T>::value!=XmlRpc::XmlRpcValue::TypeInvalid,"unsupported parameter event type");
    return XmlRpc::XmlRpcValue(value);
  }
}
template<class T> T decode(XmlRpc::XmlRpcValue value,const std::string & name) {
  static_assert(detail::xmlrpc_type<T>::value!=XmlRpc::XmlRpcValue::TypeInvalid,"unsupported parameter event type");
  if(value.getType()!=detail::xmlrpc_type<T>::value)
    throw std::invalid_argument("parameter event type mismatch: "+name);
  if constexpr(is_vector<T>::value) {
    T result;result.reserve(value.size());
    for(int i=0;i<value.size();++i)result.push_back(decode<typename T::value_type>(value[i],name));
    return result;
  } else return static_cast<T>(value);
}
}
// Historical spelling only: this class carries native XMLRPC values and links
// no ROS2 runtime. Both provider facade headers must include this single owner.
namespace rclcpp {
class Parameter {
  std::string name_;
  XmlRpc::XmlRpcValue value_;
public:
  Parameter()=default;
  Parameter(std::string name,XmlRpc::XmlRpcValue value):name_(std::move(name)),value_(std::move(value)) {}
  template<class T> Parameter(std::string name,const T & value)
    :Parameter(std::move(name),autoware_utils_rclcpp::event_detail::encode(value)) {}
  Parameter(std::string name,const char * value):Parameter(std::move(name),std::string(value)) {}
  const std::string & get_name() const {return name_;}
  XmlRpc::XmlRpcValue::Type get_type() const {return value_.getType();}
  const XmlRpc::XmlRpcValue & native_value() const {return value_;}
  template<class T> T get_value() const {
    return autoware_utils_rclcpp::event_detail::decode<T>(value_,name_);
  }
};
}
namespace autoware_utils {
template<class T> bool update_param(
  const std::vector<rclcpp::Parameter> & parameters,const std::string & name,T & output) {
  const auto key=autoware_utils_rclcpp::to_ros1_parameter_name(name);
  const auto found=std::find_if(parameters.begin(),parameters.end(),[&](const auto & parameter) {
    return autoware_utils_rclcpp::to_ros1_parameter_name(parameter.get_name())==key;
  });
  if(found==parameters.end())return false;
  // Fully decode before assignment, including array element validation.
  output=found->template get_value<T>();return true;
}
}
