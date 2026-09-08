#pragma once
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>
#include <thread>

inline void spin_for(std::chrono::milliseconds duration)
{
  const auto end = std::chrono::steady_clock::now() + duration;
  do {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while (std::chrono::steady_clock::now() < end);
  ros::spinOnce();
}

inline bool metric_before_deadline(const ros::WallTime & deadline)
{
  if (ros::WallTime::now() < deadline) return true;
  ADD_FAILURE() << "No matching fresh evaluator metric within five seconds";
  return false;
}

inline XmlRpc::XmlRpcValue yaml_value(const YAML::Node & node)
{
  XmlRpc::XmlRpcValue result;
  if (node.IsSequence()) {
    result.setSize(node.size());
    for (size_t i = 0; i < node.size(); ++i) result[i] = yaml_value(node[i]);
  } else if (node.IsMap()) {
    for (const auto & item : node) result[item.first.as<std::string>()] = yaml_value(item.second);
  } else if (node.IsScalar()) {
    if (node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") return node.as<std::string>();
    try { return node.as<bool>(); } catch (const YAML::Exception &) {}
    try { return node.as<int>(); } catch (const YAML::Exception &) {}
    try { return node.as<double>(); } catch (const YAML::Exception &) {}
    return node.as<std::string>();
  } else {
    throw std::runtime_error("Unsupported null parameter in original evaluator fixture");
  }
  return result;
}
inline void load_original_parameters(const std::string & filename, ros::NodeHandle & node)
{
  const auto parameters = YAML::LoadFile(filename)["/**"]["ros__parameters"];
  if (!parameters.IsMap()) throw std::runtime_error("Missing original parameter root: " + filename);
  for (const auto & item : parameters) {
    std::string key = item.first.as<std::string>();
    std::replace(key.begin(), key.end(), '.', '/');
    node.setParam(key, yaml_value(item.second));
  }
}
