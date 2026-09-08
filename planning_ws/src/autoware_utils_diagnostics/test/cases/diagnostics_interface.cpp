#include "autoware_utils_diagnostics/diagnostics_interface.hpp"

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

TEST(DiagnosticsInterfaceTest, Instantiation)
{
  ros::NodeHandle node;
  autoware_utils_diagnostics::DiagnosticsInterface diag(&node, "diag_name");
}

TEST(DiagnosticsInterfaceTest, PubSubAcrossClearCycles)
{
  if (!ros::master::check()) {
    GTEST_SKIP() << "requires a running ROS master for ROS1 pub/sub characterization";
  }

  ros::NodeHandle node;
  autoware_utils_diagnostics::DiagnosticsInterface diag(&node, "diag_name");
  std::mutex mutex;
  std::optional<diagnostic_msgs::DiagnosticArray> last_received;
  const auto sub = node.subscribe<diagnostic_msgs::DiagnosticArray>(
    "/diagnostics", 10,
    [&mutex, &last_received](const diagnostic_msgs::DiagnosticArray::ConstPtr & msg) {
      std::lock_guard<std::mutex> lock(mutex);
      last_received = *msg;
    });
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const auto wait_for_message = [&mutex, &last_received]() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (last_received) {
          return last_received;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::optional<diagnostic_msgs::DiagnosticArray>{};
  };
  const auto reset_received = [&mutex, &last_received]() {
    std::lock_guard<std::mutex> lock(mutex);
    last_received.reset();
  };

  const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub.getNumPublishers() == 0 && std::chrono::steady_clock::now() < ready_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (sub.getNumPublishers() == 0) {
    spinner.stop();
    GTEST_SKIP() << "master reachable but pub/sub connection did not establish";
  }

  reset_received();
  diag.add_key_value("k1", std::string("v1"));
  diag.add_key_value("k2", 3);
  diag.update_level_and_message(diagnostic_msgs::DiagnosticStatus::WARN, "warn-msg");
  diag.publish(ros::Time(101, 2));
  const auto first = wait_for_message();
  ASSERT_TRUE(first.has_value());
  ASSERT_EQ(first->status.size(), 1u);
  EXPECT_EQ(first->header.stamp, ros::Time(101, 2));
  EXPECT_EQ(first->status[0].name, "autoware_utils_diagnostics_test: diag_name");
  EXPECT_EQ(first->status[0].hardware_id, "autoware_utils_diagnostics_test");
  ASSERT_EQ(first->status[0].values.size(), 2u);
  EXPECT_EQ(first->status[0].values[0].key, "k1");
  EXPECT_EQ(first->status[0].values[0].value, "v1");
  EXPECT_EQ(first->status[0].level, diagnostic_msgs::DiagnosticStatus::WARN);
  EXPECT_EQ(first->status[0].message, "warn-msg");

  reset_received();
  diag.clear();
  diag.add_key_value("k3", std::string("v3"));
  diag.publish(ros::Time(102, 3));
  const auto second = wait_for_message();
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(second->status.size(), 1u);
  ASSERT_EQ(second->status[0].values.size(), 1u);
  EXPECT_EQ(second->status[0].values[0].key, "k3");
  EXPECT_EQ(second->status[0].level, diagnostic_msgs::DiagnosticStatus::OK);
  EXPECT_EQ(second->status[0].message, "OK");

  reset_received();
  diag.clear();
  diag.add_key_value("k1", std::string("a"));
  diag.add_key_value("k1", std::string("b"));
  diag.publish(ros::Time(103, 4));
  const auto duplicate = wait_for_message();
  ASSERT_TRUE(duplicate.has_value());
  ASSERT_EQ(duplicate->status[0].values.size(), 1u);
  EXPECT_EQ(duplicate->status[0].values[0].value, "b");
  spinner.stop();
}
