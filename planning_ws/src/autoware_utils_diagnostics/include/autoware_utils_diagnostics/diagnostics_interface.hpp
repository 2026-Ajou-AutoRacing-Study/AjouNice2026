// Copyright 2023 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE_UTILS_DIAGNOSTICS__DIAGNOSTICS_INTERFACE_HPP_
#define AUTOWARE_UTILS_DIAGNOSTICS__DIAGNOSTICS_INTERFACE_HPP_

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <ros/time.h>

#include <algorithm>
#include <cstdint>
#include <string>

namespace autoware_utils_diagnostics
{
template <typename NodeT = ros::NodeHandle>
class BasicDiagnosticsInterface
{
public:
  using DiagnosticsArrayPubPtr = ros::Publisher;

  BasicDiagnosticsInterface(NodeT * node, const std::string & diagnostic_name)
  {
    diagnostics_pub_ = node->template advertise<diagnostic_msgs::DiagnosticArray>(
      "/diagnostics", 10, false);
    const auto resolved_node_name = ros::this_node::getName();
    const auto separator = resolved_node_name.find_last_of('/');
    const auto node_name = separator == std::string::npos ? resolved_node_name :
      resolved_node_name.substr(separator + 1);
    diagnostics_status_msg_.name = node_name + std::string(": ") + diagnostic_name;
    diagnostics_status_msg_.hardware_id = node_name;
  }

  void clear()
  {
    diagnostics_status_msg_.values.clear();
    diagnostics_status_msg_.level = diagnostic_msgs::DiagnosticStatus::OK;
    diagnostics_status_msg_.message = "";
  }

  void add_key_value(const diagnostic_msgs::KeyValue & key_value_msg)
  {
    const auto it = std::find_if(
      diagnostics_status_msg_.values.begin(), diagnostics_status_msg_.values.end(),
      [&key_value_msg](const diagnostic_msgs::KeyValue & arg) { return arg.key == key_value_msg.key; });
    if (it != diagnostics_status_msg_.values.end()) {
      it->value = key_value_msg.value;
    } else {
      diagnostics_status_msg_.values.push_back(key_value_msg);
    }
  }

  template <typename T>
  void add_key_value(const std::string & key, const T & value)
  {
    diagnostic_msgs::KeyValue key_value;
    key_value.key = key;
    key_value.value = std::to_string(value);
    add_key_value(key_value);
  }

  void add_key_value(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::KeyValue key_value;
    key_value.key = key;
    key_value.value = value;
    add_key_value(key_value);
  }

  void add_key_value(const std::string & key, bool value)
  {
    diagnostic_msgs::KeyValue key_value;
    key_value.key = key;
    key_value.value = value ? "True" : "False";
    add_key_value(key_value);
  }

  void update_level_and_message(const int8_t level, const std::string & message)
  {
    if (level > diagnostic_msgs::DiagnosticStatus::OK) {
      if (!diagnostics_status_msg_.message.empty()) {
        diagnostics_status_msg_.message += "; ";
      }
      diagnostics_status_msg_.message += message;
    }
    if (level > diagnostics_status_msg_.level) {
      diagnostics_status_msg_.level = level;
    }
  }

  void publish(const ros::Time & publish_time_stamp)
  {
    diagnostics_pub_.publish(create_diagnostics_array(publish_time_stamp));
  }

private:
  diagnostic_msgs::DiagnosticArray create_diagnostics_array(const ros::Time & publish_time_stamp) const
  {
    diagnostic_msgs::DiagnosticArray diagnostics_msg;
    diagnostics_msg.header.stamp = publish_time_stamp;
    diagnostics_msg.status.push_back(diagnostics_status_msg_);
    if (diagnostics_msg.status.at(0).level == diagnostic_msgs::DiagnosticStatus::OK) {
      diagnostics_msg.status.at(0).message = "OK";
    }
    return diagnostics_msg;
  }

  DiagnosticsArrayPubPtr diagnostics_pub_;
  diagnostic_msgs::DiagnosticStatus diagnostics_status_msg_;
};

using DiagnosticsInterface = BasicDiagnosticsInterface<ros::NodeHandle>;
}  // namespace autoware_utils_diagnostics

#endif  // AUTOWARE_UTILS_DIAGNOSTICS__DIAGNOSTICS_INTERFACE_HPP_
