#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_

#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <autoware/component_interface_utils/rclcpp/service_client.hpp>
#include <autoware/component_interface_utils/rclcpp/service_server.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_publisher.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_subscription.hpp>
#include <autoware/component_interface_utils/specs.hpp>

namespace autoware::component_interface_utils
{
template <class SpecT>
typename Client<SpecT>::SharedPtr create_client_impl(NodeInterface::SharedPtr interface) { return std::make_shared<Client<SpecT>>(interface); }
template <class SpecT, class CallbackT>
typename Service<SpecT>::SharedPtr create_service_impl(NodeInterface::SharedPtr interface, CallbackT callback) { return std::make_shared<Service<SpecT>>(interface, callback); }
template <class SpecT>
typename Publisher<SpecT>::SharedPtr create_publisher_impl(NodeInterface::SharedPtr interface)
{
  const auto qos=get_qos<SpecT>(); return std::make_shared<Publisher<SpecT>>(interface->node.advertise<typename SpecT::Message>(SpecT::name,qos.queue_size,qos.latch));
}
template <class SpecT, class CallbackT>
typename Subscription<SpecT>::SharedPtr create_subscription_impl(NodeInterface::SharedPtr interface, CallbackT callback)
{
  const auto qos=get_qos<SpecT>(); auto out=std::make_shared<Subscription<SpecT>>();
  auto wrapper=[out,callback](const typename SpecT::Message::ConstPtr & message){ out->store(message); callback(message); };
  out->reset(interface->node.subscribe<typename SpecT::Message>(SpecT::name,qos.queue_size,wrapper)); return out;
}
template <class SpecT>
typename Subscription<SpecT>::SharedPtr create_polling_subscription_impl(NodeInterface::SharedPtr interface)
{
  const auto qos=get_qos<SpecT>(); auto out=std::make_shared<Subscription<SpecT>>();
  out->reset(interface->node.subscribe<typename SpecT::Message>(SpecT::name,qos.queue_size,[out](const typename SpecT::Message::ConstPtr & message){out->store(message);})); return out;
}
}  // namespace autoware::component_interface_utils
#endif
