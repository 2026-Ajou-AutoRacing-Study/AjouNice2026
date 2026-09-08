// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERTER_BASE_HPP_
#define AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERTER_BASE_HPP_

#include <ros/ros.h>

#include <stdexcept>
#include <string>

namespace autoware
{
namespace planning_topic_converter
{

template <typename InputType, typename OutputType>
class ConverterBase
{
public:
  ConverterBase(
    ros::NodeHandle & node, ros::NodeHandle & private_node, const std::string & node_name)
  : node_(node), private_node_(private_node), node_name_(node_name)
  {
    std::string input_topic;
    std::string output_topic;
    if (!private_node_.getParam("input_topic", input_topic) || input_topic.empty() ||
        !private_node_.getParam("output_topic", output_topic) || output_topic.empty()) {
      throw std::invalid_argument(
        "~input_topic and ~output_topic must be non-empty for " + node_name_);
    }
    pub_ = node_.advertise<OutputType>(output_topic, 1);
    sub_ = node_.subscribe<InputType>(input_topic, 1, &ConverterBase::process, this);
  }

protected:
  virtual void process(const typename InputType::ConstPtr & msg) = 0;
  ros::Publisher pub_;
  ros::Subscriber sub_;

private:
  ros::NodeHandle & node_;
  ros::NodeHandle & private_node_;
  std::string node_name_;
};

}  // namespace planning_topic_converter
}  // namespace autoware

#endif
