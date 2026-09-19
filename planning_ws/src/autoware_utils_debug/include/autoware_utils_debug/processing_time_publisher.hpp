#pragma once

#include <autoware_utils_debug/ros1_adapters.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <iomanip>
#include <map>
#include <sstream>
#include <string>

namespace autoware_utils_debug
{
class ProcessingTimePublisher
{
public:
  explicit ProcessingTimePublisher(
    ros1::CallerNodeContext::Ptr context,
    const std::string & name = "~/debug/processing_time_ms")
  : context_(std::move(context))
  {
    if (!context_) throw std::invalid_argument("ProcessingTimePublisher requires context");
    publisher_ = context_->advertise_default<diagnostic_msgs::DiagnosticStatus>(name);
  }
  void publish(const std::map<std::string, double> & values)
  {
    context_->require_alive();
    diagnostic_msgs::DiagnosticStatus status;
    for (const auto & item : values) {
      diagnostic_msgs::KeyValue value;
      value.key = item.first;
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3) << item.second;
      value.value = stream.str();
      status.values.push_back(value);
    }
    context_->require_alive();
    publisher_.publish(status);
  }
private:
  ros1::CallerNodeContext::Ptr context_;
  ros::Publisher publisher_;
};
}
