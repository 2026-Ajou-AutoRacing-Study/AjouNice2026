#pragma once
#include <ros/ros.h>
#include <ros/file_log.h>
#include <builtin_interfaces/Time.h>
#include <algorithm>
#include <stdexcept>

namespace planning_diagnostics
{
class Ros1EvaluatorNode : public ros::NodeHandle
{
public:
  explicit Ros1EvaluatorNode(const ros::NodeHandle & node) : ros::NodeHandle(node) {}
  virtual ~Ros1EvaluatorNode() = default;
  template<class T> T declare_parameter(std::string name) const
  {
    std::replace(name.begin(), name.end(), '.', '/');
    T value;
    if (!getParam(name, value)) {
      throw std::runtime_error("Required evaluator parameter missing: " + resolveName(name));
    }
    return value;
  }
  static ros::Time now() { return ros::Time::now(); }
};
inline std::string ros1_log_directory() { return ros::file_log::getLogDirectory(); }
inline builtin_interfaces::Time builtin_stamp(const ros::Time & stamp)
{
  builtin_interfaces::Time result;
  result.sec = stamp.sec;
  result.nanosec = stamp.nsec;
  return result;
}
}
