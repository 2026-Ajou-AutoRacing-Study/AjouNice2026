#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_

#include <autoware/component_interface_utils/rclcpp/create_interface.hpp>
#include <functional>
#include <optional>

namespace autoware::component_interface_utils
{
class NodeAdaptor
{
 public:
  explicit NodeAdaptor(const ros::NodeHandle & node) : interface_(std::make_shared<NodeInterface>(node)) {}
  template <class SharedPtrT, class InstanceT>
  using MessagePtrCallback = void (InstanceT::*)(typename SharedPtrT::element_type::SpecType::Message::ConstPtr);
  template <class SharedPtrT, class InstanceT>
  using MessageRefCallback = void (InstanceT::*)(const typename SharedPtrT::element_type::SpecType::Message &);
  template <class SharedPtrT, class InstanceT>
  using ServiceCallback = void (InstanceT::*)(
    std::shared_ptr<typename SharedPtrT::element_type::SpecType::Service::Request>,
    std::shared_ptr<typename SharedPtrT::element_type::SpecType::Service::Response>);
  template <class SharedPtrT> void init_cli(SharedPtrT & cli) const { cli=create_client_impl<typename SharedPtrT::element_type::SpecType>(interface_); }
  template <class SharedPtrT, class CallbackT> void init_srv(SharedPtrT & srv, CallbackT callback) const { srv=create_service_impl<typename SharedPtrT::element_type::SpecType>(interface_,callback); }
  template <class SharedPtrT> void init_pub(SharedPtrT & pub) const { pub=create_publisher_impl<typename SharedPtrT::element_type::SpecType>(interface_); }
  template <class SharedPtrT, class CallbackT> void init_sub(SharedPtrT & sub, CallbackT callback) const { sub=create_subscription_impl<typename SharedPtrT::element_type::SpecType>(interface_,callback); }
  template <class SharedPtrT> void init_sub(SharedPtrT & sub, std::nullptr_t) const { sub=create_polling_subscription_impl<typename SharedPtrT::element_type::SpecType>(interface_); }
  template <class SharedPtrT, class InstanceT>
  void init_sub(SharedPtrT & sub, InstanceT * instance, MessagePtrCallback<SharedPtrT, InstanceT> callback) const
  {
    init_sub(sub, [instance, callback](typename SharedPtrT::element_type::SpecType::Message::ConstPtr message) {
      (instance->*callback)(message);
    });
  }
  template <class SharedPtrT, class InstanceT>
  void init_sub(SharedPtrT & sub, InstanceT * instance, MessageRefCallback<SharedPtrT, InstanceT> callback) const
  {
    init_sub(sub, [instance, callback](typename SharedPtrT::element_type::SpecType::Message::ConstPtr message) {
      auto copy = *message;
      (instance->*callback)(copy);
    });
  }
  template <class SharedPtrT, class InstanceT>
  void init_srv(SharedPtrT & srv, InstanceT * instance, ServiceCallback<SharedPtrT, InstanceT> callback) const
  {
    init_srv(srv, [instance, callback](auto & request, auto & response) {
      auto request_ptr = std::make_shared<std::decay_t<decltype(request)>>(request);
      auto response_ptr = std::make_shared<std::decay_t<decltype(response)>>(response);
      (instance->*callback)(request_ptr, response_ptr);
      response = *response_ptr;
    });
  }
  template <class P, class S> void relay_message(P & pub,S & sub) const { init_pub(pub); init_sub(sub,[pub](const typename P::element_type::SpecType::Message::ConstPtr & msg){pub->publish(*msg);}); }
  template <class C, class S> void relay_service(C & cli,S & srv,std::optional<double> timeout=std::nullopt) const { init_cli(cli); init_srv(srv,[cli,timeout](auto & req,auto & res){res=*cli->call(req,timeout);}); }
 private:
  NodeInterface::SharedPtr interface_;
};
}  // namespace autoware::component_interface_utils
#endif
