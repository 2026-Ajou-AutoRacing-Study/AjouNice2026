#include <autoware/component_interface_utils/rclcpp.hpp>
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace {
struct BoolService { using Service = std_srvs::SetBool; static constexpr char name[] = "/component_interface_utils_test/bool"; };
struct StringTopic {
  using Message = std_msgs::String; static constexpr char name[] = "/component_interface_utils_test/string";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = autoware::component_interface_specs::QosReliability::Reliable;
  static constexpr auto durability = autoware::component_interface_specs::QosDurability::Volatile;
};
struct MemberCallbacks {
  explicit MemberCallbacks(std::atomic<int> & service_calls) : service_calls(service_calls) {}
  void on_pointer(std_msgs::String::ConstPtr message) { pointer_value = message->data; ++pointer_calls; }
  void on_reference(const std_msgs::String & message) { reference_value = message.data; ++reference_calls; }
  void on_service(std::shared_ptr<std_srvs::SetBool::Request> request, std::shared_ptr<std_srvs::SetBool::Response> response) {
    ++service_calls;
    if (!request->data) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    response->success = true;
  }
  std::atomic<int> & service_calls;
  std::atomic<int> pointer_calls{0};
  std::atomic<int> reference_calls{0};
  std::string pointer_value;
  std::string reference_value;
};
TEST(ros1_wrapper, constructs_publish_subscribe_client_and_service)
{
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  autoware::component_interface_utils::NodeAdaptor adaptor(node);
  autoware::component_interface_utils::Publisher<StringTopic>::SharedPtr publisher;
  autoware::component_interface_utils::Subscription<StringTopic>::SharedPtr subscription;
  autoware::component_interface_utils::Subscription<StringTopic>::SharedPtr pointer_subscription;
  autoware::component_interface_utils::Subscription<StringTopic>::SharedPtr reference_subscription;
  autoware::component_interface_utils::Client<BoolService>::SharedPtr client;
  autoware::component_interface_utils::Service<BoolService>::SharedPtr service;
  std::atomic<int> service_calls{0};
  MemberCallbacks callbacks(service_calls);
  adaptor.init_pub(publisher);
  adaptor.init_sub(subscription, nullptr);
  adaptor.init_sub(pointer_subscription, &callbacks, &MemberCallbacks::on_pointer);
  adaptor.init_sub(reference_subscription, &callbacks, &MemberCallbacks::on_reference);
  adaptor.init_cli(client);
  adaptor.init_srv(service, &callbacks, &MemberCallbacks::on_service);
  EXPECT_TRUE(client->service_is_ready());
  std_srvs::SetBool::Request request;
  request.data = true;
  const auto response = client->call(request, 1.0);
  ASSERT_TRUE(response);
  EXPECT_TRUE(response->success);

  std_srvs::SetBool::Request delayed;
  delayed.data = false;
  EXPECT_THROW(client->call(delayed, 0.01), autoware::component_interface_utils::ServiceTimeout);
  for (int i = 0; i < 100 && service_calls.load() < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(service_calls.load(), 2);

  std_msgs::String value;
  value.data = "wrapper";
  std_msgs::String received;
  bool received_message = false;
  for (int i = 0; i < 100 && !received_message; ++i) {
    publisher->publish(value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    received_message = subscription->take_and_update(received);
  }
  EXPECT_TRUE(received_message);
  if (received_message) {
    EXPECT_EQ(received.data, "wrapper");
  }
  for (int i = 0; i < 100 && (callbacks.pointer_calls.load() == 0 || callbacks.reference_calls.load() == 0); ++i) {
    publisher->publish(value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(callbacks.pointer_value, "wrapper");
  EXPECT_EQ(callbacks.reference_value, "wrapper");
  EXPECT_GT(callbacks.pointer_calls.load(), 0);
  EXPECT_GT(callbacks.reference_calls.load(), 0);
  spinner.stop();
}
}
