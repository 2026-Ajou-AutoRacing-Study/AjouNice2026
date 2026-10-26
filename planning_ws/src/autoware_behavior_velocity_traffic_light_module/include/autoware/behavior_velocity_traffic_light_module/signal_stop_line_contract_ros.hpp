// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_STOP_LINE_CONTRACT_ROS_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_STOP_LINE_CONTRACT_ROS_HPP_
#include <autoware/behavior_velocity_traffic_light_module/signal_stop_line_contract.hpp>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <stdexcept>
#include <string>

namespace autoware::behavior_velocity_planner::signal_compliance
{
inline double contractNumber(XmlRpc::XmlRpcValue & value)
{
  if (value.getType()==XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
  if (value.getType()==XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
  throw std::invalid_argument("expected finite numeric field");
}
inline std::vector<MapPoint> contractPoints(XmlRpc::XmlRpcValue & value)
{
  if (value.getType()!=XmlRpc::XmlRpcValue::TypeArray || (value.size()!=4&&value.size()!=6))
    throw std::invalid_argument("expected flattened XY list of length4 or6");
  std::vector<MapPoint> result;
  for(int i=0;i<value.size();i+=2) result.push_back({contractNumber(value[i]),contractNumber(value[i+1])});
  return result;
}
// An absent/malformed profile never refuses process startup. A recognized but
// invalid/duplicate regulatory ID is retained invalid to disable its timed GO.
inline std::vector<StopLineContract> loadStopLineContracts(ros::NodeHandle & node,const std::string & name)
{
  XmlRpc::XmlRpcValue values;
  if (!node.getParam(name,values)) {
    ROS_WARN("[signal_stop_line_contract] no profile; retaining map stop/crossing behavior");
    return {};
  }
  if (values.getType()!=XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("[signal_stop_line_contract] profile is not a list; retaining map behavior");
    return {};
  }
  std::vector<StopLineContract> result;
  for(int i=0;i<values.size();++i) {
    StopLineContract c;
    try {
      auto & value=values[i];
      if (value.getType()!=XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember("regulatory_id") ||
          value["regulatory_id"].getType()!=XmlRpc::XmlRpcValue::TypeInt)
        throw std::invalid_argument("regulatory_id must be an XMLRPC integer");
      c.regulatory_id=static_cast<int>(value["regulatory_id"]);
      for(const auto & key:{"expected_stop_line_xy","crossing_line_xy","additional_stop_margin_m","max_anchor_error_m"})
        if(!value.hasMember(key)) throw std::invalid_argument("missing required contract field");
      c.expected_stop_line=contractPoints(value["expected_stop_line_xy"]);
      c.crossing_line=contractPoints(value["crossing_line_xy"]);
      c.additional_stop_margin_m=contractNumber(value["additional_stop_margin_m"]);
      c.max_anchor_error_m=contractNumber(value["max_anchor_error_m"]);
      if(!validStopLineContract(c)) throw std::invalid_argument("invalid line geometry/margin/anchor tolerance");
    } catch(const std::exception & error) {
      c.valid=false;
      ROS_WARN("[signal_stop_line_contract] index=%d reg=%ld ignored: %s; no startup refusal",i,c.regulatory_id,error.what());
    } catch(...) {
      c.valid=false;
      ROS_WARN("[signal_stop_line_contract] index=%d malformed XMLRPC; no startup refusal",i);
    }
    if(c.regulatory_id<=0) continue;
    for(auto & existing:result) if(existing.regulatory_id==c.regulatory_id) {
      existing.valid=false;c.valid=false;
      ROS_WARN("[signal_stop_line_contract] duplicate reg=%ld: override disabled",c.regulatory_id);
    }
    result.push_back(c);
  }
  return result;
}
}
#endif
