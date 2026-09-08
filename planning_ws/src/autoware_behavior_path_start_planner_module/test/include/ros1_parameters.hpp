#pragma once
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>
#include <algorithm>

namespace start_test_parameters {
inline XmlRpc::XmlRpcValue value(const YAML::Node & node)
{
  XmlRpc::XmlRpcValue result;
  if (node.IsSequence()) {
    result.setSize(node.size());
    for (size_t i = 0; i < node.size(); ++i) result[i] = value(node[i]);
  } else if (node.IsMap()) {
    for (const auto & item : node) result[item.first.as<std::string>()] = value(item.second);
  } else if (node.IsScalar()) {
    if (node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") return node.as<std::string>();
    try { return node.as<bool>(); } catch (const YAML::Exception &) {}
    try { return node.as<int>(); } catch (const YAML::Exception &) {}
    try { return node.as<double>(); } catch (const YAML::Exception &) {}
    return node.as<std::string>();
  } else { throw std::runtime_error("Unsupported null fixture parameter"); }
  return result;
}
inline void load(const std::string & filename, ros::NodeHandle & node)
{
  const auto parameters = YAML::LoadFile(filename)["/**"]["ros__parameters"];
  if (!parameters.IsMap()) throw std::runtime_error("Missing parameter root: " + filename);
  for (const auto & item : parameters) {
    auto key = item.first.as<std::string>();
    std::replace(key.begin(), key.end(), '.', '/');
    node.setParam(key, value(item.second));
  }
}
}
