#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_

#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <boost/function.hpp>
#include <memory>
#include <type_traits>

namespace autoware::component_interface_utils
{
template <class SpecT>
class Service
{
 public:
  using SharedPtr = std::shared_ptr<Service>; using SpecType = SpecT;
  template <class CallbackT>
  Service(NodeInterface::SharedPtr interface, CallbackT callback) : interface_(interface)
  {
    using Request = typename SpecT::Service::Request;
    using Response = typename SpecT::Service::Response;
    boost::function<bool(Request &, Response &)> wrapped = [this, callback](Request & request, Response & response) {
      interface_->log(1, SpecT::name, to_yaml(request));
      if constexpr (has_status<Response>::value) {
        try {
          callback(request, response);
        } catch (const ServiceException & error) {
          error.set(response.status);
        }
      } else {
        callback(request, response);
      }
      interface_->log(2, SpecT::name, to_yaml(response)); return true;
    };
    service_ = interface_->node.advertiseService(SpecT::name, wrapped);
  }
 private:
  template <class ResponseT, class = void>
  struct has_status : std::false_type {};
  template <class ResponseT>
  struct has_status<ResponseT, std::void_t<decltype(std::declval<ResponseT &>().status)>> : std::true_type {};
  ros::ServiceServer service_; NodeInterface::SharedPtr interface_;
};
}  // namespace autoware::component_interface_utils
#endif
