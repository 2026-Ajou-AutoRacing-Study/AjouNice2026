#pragma once
#include <ros/serialization.h>
#include <ros/message_traits.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace autoware::behavior_path_planner::utils::lane_change {
template<class Message> boost::property_tree::ptree snapshot_ros_message(const Message & message)
{
  const auto size=ros::serialization::serializationLength(message);
  std::vector<uint8_t> bytes(size);
  ros::serialization::OStream stream(bytes.data(),size);
  ros::serialization::serialize(stream,message);
  static constexpr char digits[]="0123456789abcdef";
  std::string hex;hex.reserve(2*size);
  for(const auto byte:bytes){hex.push_back(digits[byte>>4]);hex.push_back(digits[byte&15]);}
  boost::property_tree::ptree result;
  result.put("datatype",ros::message_traits::DataType<Message>::value());
  result.put("md5",ros::message_traits::MD5Sum<Message>::value());
  result.put("bytes",size);result.put("hex",hex);
  return result;
}
template<class Message> Message decode_snapshot_ros_message(const boost::property_tree::ptree & packet)
{
  if(packet.get<std::string>("datatype")!=ros::message_traits::DataType<Message>::value() ||
     packet.get<std::string>("md5")!=ros::message_traits::MD5Sum<Message>::value())
    throw std::logic_error("snapshot ROS schema mismatch");
  const auto hex=packet.get<std::string>("hex");
  if(hex.size()%2 || hex.size()/2!=packet.get<size_t>("bytes"))
    throw std::logic_error("snapshot hex length mismatch");
  const auto nibble=[](char c)->unsigned{
    if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;
    throw std::logic_error("snapshot invalid hex");};
  std::vector<uint8_t> bytes;bytes.reserve(hex.size()/2);
  for(size_t i=0;i<hex.size();i+=2)bytes.push_back((nibble(hex[i])<<4)|nibble(hex[i+1]));
  ros::serialization::IStream stream(bytes.data(),bytes.size());Message result;
  ros::serialization::deserialize(stream,result);
  if(stream.getLength()!=0)throw std::logic_error("snapshot trailing bytes");
  return result;
}
}
