// Native ROS1 ownership for original map-loader algorithm bodies.
#pragma once
#include <ros/ros.h>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <memory>
#include <string>
#include <utility>
#include <type_traits>
#define RCLCPP_ERROR(logger,...) ROS_ERROR(__VA_ARGS__)
#define RCLCPP_WARN(logger,...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_INFO(logger,...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_ERROR_STREAM(logger,args) ROS_ERROR_STREAM(args)
#define RCLCPP_DEBUG_STREAM(logger,args) ROS_DEBUG_STREAM(args)
namespace autoware::map_loader::map_native {
class Node:public ros::NodeHandle {
public:
  explicit Node(ros::NodeHandle node):ros::NodeHandle(std::move(node)) {}
  std::string get_logger() const {return getNamespace();}
  ros::Time now() const {return ros::Time::now();}
  template<class T> T declare_parameter(const std::string & name) const {
    if constexpr(std::is_same_v<T,float>) return static_cast<float>(declare_parameter<double>(name));
    else return autoware_utils_rclcpp::detail::strict_get<T>(*this,name);
  }
  template<class T> std::shared_ptr<ros::Publisher> create_publisher(
    const std::string & name,unsigned depth,bool latch=false) {
    return std::make_shared<ros::Publisher>(advertise<T>(name,depth,latch));
  }
  template<class Service,class Callback> ros::ServiceServer create_service(
    const std::string & name,Callback callback) {
    using Request=typename Service::Request;using Response=typename Service::Response;
    boost::function<bool(Request &,Response &)> adapter = [callback](Request & request,Response & response) {
      auto req=std::make_shared<Request>(request);auto res=std::make_shared<Response>();
      const bool result=callback(req,res);response=*res;return result;
    };
    return advertiseService<Request,Response>(name,adapter);
  }
};
}
