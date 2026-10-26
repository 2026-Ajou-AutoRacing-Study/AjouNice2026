// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_JUDGING_LINE_ROS_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_JUDGING_LINE_ROS_HPP_
#include <autoware/behavior_velocity_traffic_light_module/signal_judging_line.hpp>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <stdexcept>
#include <string>

namespace autoware::behavior_velocity_planner::signal_compliance
{
inline double judgingNumber(XmlRpc::XmlRpcValue & value)
{
  if(value.getType()==XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
  if(value.getType()==XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
  throw std::invalid_argument("expected finite numeric coordinate");
}
inline std::vector<JudgingPoint> judgingPoints(XmlRpc::XmlRpcValue & value)
{
  if(value.getType()!=XmlRpc::XmlRpcValue::TypeArray||value.size()<4||value.size()>32||value.size()%2)
    throw std::invalid_argument("expected flattened XY coordinates");
  std::vector<JudgingPoint> points;
  for(int i=0;i<value.size();i+=2) points.push_back({judgingNumber(value[i]),judgingNumber(value[i+1])});
  return points;
}
// Invalid/absent geometry never refuses process startup. The scene keeps the
// ordinary map stop target but cannot authorize timed yellow with virtual ETA.
inline std::vector<SignalJudgingLine> parseSignalJudgingLines(XmlRpc::XmlRpcValue & values)
{
  if(values.getType()!=XmlRpc::XmlRpcValue::TypeArray||values.size()>32) {
    ROS_WARN("[signal_judging_line] absent/malformed profile; no original-line yellow ETA");
    return {};
  }
  std::vector<SignalJudgingLine> lines;
  for(int i=0;i<values.size();++i) {
    SignalJudgingLine line;
    try {
      auto & value=values[i];
      if(value.getType()!=XmlRpc::XmlRpcValue::TypeStruct||!value.hasMember("regulatory_id")||
         value["regulatory_id"].getType()!=XmlRpc::XmlRpcValue::TypeInt)
        throw std::invalid_argument("integer regulatory_id required");
      line.regulatory_id=static_cast<int>(value["regulatory_id"]);
      if(!value.hasMember("line_xy")||!value.hasMember("approach_xy"))
        throw std::invalid_argument("line_xy and approach_xy required");
      line.points=judgingPoints(value["line_xy"]);
      auto & direction=value["approach_xy"];
      if(direction.getType()!=XmlRpc::XmlRpcValue::TypeArray||direction.size()!=2)
        throw std::invalid_argument("unit approach_xy required");
      line.approach_direction={judgingNumber(direction[0]),judgingNumber(direction[1])};
      if(!validJudgingLine(line)) throw std::invalid_argument("invalid line/direction geometry");
    } catch(const std::exception & error) {
      line.valid=false;
      ROS_WARN("[signal_judging_line] index=%d reg=%ld disabled: %s",i,line.regulatory_id,error.what());
    } catch(...) {
      line.valid=false;
      ROS_WARN("[signal_judging_line] index=%d malformed XMLRPC geometry disabled",i);
    }
    if(line.regulatory_id<=0) continue;
    for(auto & previous:lines) if(previous.regulatory_id==line.regulatory_id) {
      previous.valid=false;line.valid=false;
      ROS_WARN("[signal_judging_line] duplicate reg=%ld disabled",line.regulatory_id);
    }
    lines.push_back(line);
  }
  return lines;
}
inline std::vector<SignalJudgingLine> loadSignalJudgingLines(
  ros::NodeHandle & node,const std::string & name)
{
  XmlRpc::XmlRpcValue values;
  if(!node.getParam(name,values)) {
    ROS_WARN("[signal_judging_line] profile absent; no original-line yellow ETA");
    return {};
  }
  return parseSignalJudgingLines(values);
}
}  // namespace autoware::behavior_velocity_planner::signal_compliance
#endif
