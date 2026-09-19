#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <std_msgs/String.h>
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using Latest = autoware_utils_rclcpp::InterProcessPollingSubscriber<std_msgs::String, autoware_utils_rclcpp::polling_policy::Latest>;
using Newest = autoware_utils_rclcpp::InterProcessPollingSubscriber<std_msgs::String, autoware_utils_rclcpp::polling_policy::Newest>;
using All = autoware_utils_rclcpp::InterProcessPollingSubscriber<std_msgs::String, autoware_utils_rclcpp::polling_policy::All>;

template<class Subscription> typename std_msgs::String::ConstPtr wait_one(const std::shared_ptr<Subscription> & sub)
{
  for (int i = 0; i < 80; ++i) { if (auto msg = sub->take_data()) return msg; ros::spinOnce(); std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
  return typename std_msgs::String::ConstPtr();
}

TEST(Polling, DepthRulesAndInitialValues)
{
  ros::NodeHandle node;
  EXPECT_THROW(Latest::create_subscription(&node, "/latest_deep", 2), std::invalid_argument);
  EXPECT_THROW(Newest::create_subscription(&node, "/newest_deep", 2), std::invalid_argument);
  EXPECT_NO_THROW(All::create_subscription(&node, "/all_deep", 10));
  auto latest = Latest::create_subscription(&node, "/initial", 1);
  EXPECT_FALSE(latest->take_data()); EXPECT_FALSE(latest->last_taken_data_timestamp());
}

TEST(Polling, LatestRetainsNewestClearsAndAllDrains)
{
  ros::NodeHandle node;
  auto pub = node.advertise<std_msgs::String>("/polling_contract", 10);
  auto latest = Latest::create_subscription(&node, "/polling_contract", 1);
  auto newest = Newest::create_subscription(&node, "/polling_contract", 1);
  auto all = All::create_subscription(&node, "/polling_contract", 10);
  for (int i = 0; i < 30 && pub.getNumSubscribers() < 3; ++i) { ros::spinOnce(); std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
  std_msgs::String one; one.data = "one"; pub.publish(one);
  auto l1 = wait_one(latest); auto n1 = wait_one(newest);
  ASSERT_TRUE(l1); ASSERT_TRUE(n1); EXPECT_EQ(l1->data, "one"); EXPECT_EQ(n1->data, "one");
  const auto ltime = latest->last_taken_data_timestamp(); EXPECT_TRUE(ltime);
  EXPECT_EQ(latest->take_data(), l1); EXPECT_EQ(latest->last_taken_data_timestamp(), ltime);
  EXPECT_FALSE(newest->take_data()); EXPECT_FALSE(newest->last_taken_data_timestamp());
  std_msgs::String two; two.data = "two"; std_msgs::String three; three.data = "three"; pub.publish(two); pub.publish(three);
  std::vector<std_msgs::String::ConstPtr> all_messages;
  for (int i = 0; i < 80 && all_messages.size() < 2; ++i) { all_messages = all->take_data(); ros::spinOnce(); std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
  ASSERT_EQ(all_messages.size(), 2U); EXPECT_EQ(all_messages[0]->data, "two"); EXPECT_EQ(all_messages[1]->data, "three"); EXPECT_TRUE(all->last_taken_data_timestamp());
  EXPECT_TRUE(all->take_data().empty()); EXPECT_FALSE(all->last_taken_data_timestamp());
}

TEST(Polling, NormalSpinnerDoesNotDrainThePrivateQueue)
{
  ros::NodeHandle node;
  auto pub = node.advertise<std_msgs::String>("/private_queue_contract", 1);
  auto newest = Newest::create_subscription(&node, "/private_queue_contract", 1);
  for (int i = 0; i < 30 && pub.getNumSubscribers() < 1; ++i) {
    ros::spinOnce(); std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  std_msgs::String message; message.data = "private"; pub.publish(message);
  for (int i = 0; i < 20; ++i) {
    ros::spinOnce(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(newest->last_taken_data_timestamp());
  const auto received = wait_one(newest);
  ASSERT_TRUE(received);
  EXPECT_EQ(received->data, "private");
  EXPECT_TRUE(newest->last_taken_data_timestamp());
}
