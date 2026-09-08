#pragma once

#include <autoware_internal_debug_msgs/BoolStamped.h>
#include <autoware_internal_debug_msgs/Float32MultiArrayStamped.h>
#include <autoware_internal_debug_msgs/Float32Stamped.h>
#include <autoware_internal_debug_msgs/Float64MultiArrayStamped.h>
#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_internal_debug_msgs/Int32MultiArrayStamped.h>
#include <autoware_internal_debug_msgs/Int32Stamped.h>
#include <autoware_internal_debug_msgs/Int64MultiArrayStamped.h>
#include <autoware_internal_debug_msgs/Int64Stamped.h>
#include <autoware_internal_debug_msgs/StringStamped.h>

#include <type_traits>

namespace autoware_utils_debug::debug_traits
{
template <typename T> struct is_debug_message : std::false_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::BoolStamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Float32MultiArrayStamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Float32Stamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Float64MultiArrayStamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Float64Stamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Int32MultiArrayStamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Int32Stamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Int64MultiArrayStamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::Int64Stamped> : std::true_type {};
template <> struct is_debug_message<autoware_internal_debug_msgs::StringStamped> : std::true_type {};
}
