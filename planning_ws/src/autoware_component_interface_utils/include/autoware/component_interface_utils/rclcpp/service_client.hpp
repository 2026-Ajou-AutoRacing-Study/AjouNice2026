#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_

#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>

#include <future>
#include <memory>
#include <optional>

namespace autoware::component_interface_utils
{
template <class SpecT>
class Client
{
public:
  using SharedPtr = std::shared_ptr<Client>;
  using SpecType = SpecT;
  using ResponsePtr = std::shared_ptr<typename SpecT::Service::Response>;
  using SharedFuture = std::shared_future<ResponsePtr>;
  explicit Client(NodeInterface::SharedPtr interface) : interface_(interface), client_(interface->node.serviceClient<typename SpecT::Service>(SpecT::name)) {}
  ResponsePtr call(const typename SpecT::Service::Request & request, std::optional<double> timeout = std::nullopt)
  {
    if (!client_.exists()) { interface_->log(4, SpecT::name); throw ServiceUnready(SpecT::name); }
    auto future = async_send_request(request);
    if (timeout && future.wait_for(std::chrono::duration<double>(*timeout)) != std::future_status::ready) {
      interface_->log(5, SpecT::name); throw ServiceTimeout(SpecT::name);
    }
    return future.get();
  }
  SharedFuture async_send_request(const typename SpecT::Service::Request & request)
  {
    return async_send_request(request, [](SharedFuture) {});
  }
  template <class CallbackT>
  SharedFuture async_send_request(const typename SpecT::Service::Request & request, CallbackT callback)
  {
    auto promise = std::make_shared<std::promise<ResponsePtr>>(); auto future = promise->get_future().share();
    auto interface = interface_; auto client = client_; auto copy = request;
    interface_->log(0, SpecT::name, to_yaml(copy));
    interface_->dispatch_client([promise, future, interface, client, copy, callback]() mutable {
      typename SpecT::Service service; service.request = copy;
      if (!client.call(service)) { promise->set_exception(std::make_exception_ptr(ServiceUnready(SpecT::name))); return; }
      auto response = std::make_shared<typename SpecT::Service::Response>(service.response);
      interface->log(3, SpecT::name, to_yaml(*response));
      promise->set_value(response); callback(future);
    });
    return future;
  }
  bool service_is_ready() const { return client_.exists(); }
private:
  mutable ros::ServiceClient client_;
  NodeInterface::SharedPtr interface_;
};
}  // namespace autoware::component_interface_utils
#endif
