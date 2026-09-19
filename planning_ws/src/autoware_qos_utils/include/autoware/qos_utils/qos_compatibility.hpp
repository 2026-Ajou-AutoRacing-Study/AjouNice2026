#ifndef AUTOWARE__QOS_UTILS__QOS_COMPATIBILITY_HPP_
#define AUTOWARE__QOS_UTILS__QOS_COMPATIBILITY_HPP_

#include <ros/advertise_service_options.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/service_client_options.h>
#include <ros/service_server.h>
#include <ros/spinner.h>
#include <ros/subscribe_options.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <cstdint>
#include <map>
#include <string>

namespace autoware
{
namespace qos_utils
{

// ROS 1 has no DDS reliability or durability QoS contract.  The bounded mapping
// below preserves only the representable ROS 1 properties: queue depth 10 and
// non-latched publication.  Callers must not infer DDS reliable/volatile parity.
struct TopicEndpointPolicy
{
  enum : std::uint32_t {queue_depth = 10U};
  static bool latch() {return false;}
  static bool dds_reliability_equivalent() {return false;}
  static bool dds_durability_equivalent() {return false;}
};

class SingleThreadCallbackDomain
{
public:
  SingleThreadCallbackDomain()
  : queue_(), node_(), spinner_(1U, &queue_)
  {
    node_.setCallbackQueue(&queue_);
    spinner_.start();
  }

  ~SingleThreadCallbackDomain() {spinner_.stop();}

  ros::NodeHandle & node() {return node_;}
  ros::CallbackQueue & queue() {return queue_;}

  SingleThreadCallbackDomain(const SingleThreadCallbackDomain &) = delete;
  SingleThreadCallbackDomain & operator=(const SingleThreadCallbackDomain &) = delete;

private:
  ros::CallbackQueue queue_;
  ros::NodeHandle node_;
  ros::AsyncSpinner spinner_;
};

class ServiceClientAdapter
{
public:
  template<class Service>
  ros::ServiceClient create(
    const std::string & service_name, const bool persistent = false,
    const std::map<std::string, std::string> & header_values = {})
  {
    return domain_.node().serviceClient<Service>(service_name, persistent, header_values);
  }

private:
  SingleThreadCallbackDomain domain_;
};

class ServiceServerAdapter
{
public:
  template<class Service, class Callback>
  ros::ServiceServer create(const std::string & service_name, const Callback & callback)
  {
    ros::AdvertiseServiceOptions options = ros::AdvertiseServiceOptions::create<Service>(
      service_name, callback, ros::VoidConstPtr(), &domain_.queue());
    return domain_.node().advertiseService(options);
  }

private:
  SingleThreadCallbackDomain domain_;
};

// Process-lifetime domains avoid changing consumer class layouts while keeping
// client and server execution isolated from the global callback queue.
inline ServiceClientAdapter & ordered_service_clients()
{
  static ServiceClientAdapter adapter;
  return adapter;
}

inline ServiceServerAdapter & ordered_service_servers()
{
  static ServiceServerAdapter adapter;
  return adapter;
}

template<class Message>
ros::Publisher advertise_topic(ros::NodeHandle & node, const std::string & topic)
{
  return node.advertise<Message>(
    topic, TopicEndpointPolicy::queue_depth, TopicEndpointPolicy::latch());
}

template<class Message, class Callback>
ros::Subscriber subscribe_topic(
  ros::NodeHandle & node, const std::string & topic, const Callback & callback)
{
  ros::SubscribeOptions options = ros::SubscribeOptions::create<Message>(
    topic, TopicEndpointPolicy::queue_depth, callback, ros::VoidConstPtr(), nullptr);
  return node.subscribe(options);
}

}  // namespace qos_utils
}  // namespace autoware

#endif  // AUTOWARE__QOS_UTILS__QOS_COMPATIBILITY_HPP_
