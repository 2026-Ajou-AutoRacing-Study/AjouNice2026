// Copyright 2024 TIER IV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware_test_utils/autoware_test_utils.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gtest/gtest.h>

#include <string>

class RelayNode
{
public:
  explicit RelayNode(ros::NodeHandle & node) : node_(node)
  {
    subscription_ = node_.subscribe<std_msgs::String>(
      "input_topic", 10, [this](const std_msgs::String::ConstPtr & msg) {
        ROS_INFO_STREAM("Received message: " << msg->data);
        publisher_.publish(*msg);
      });
    publisher_ = node_.advertise<std_msgs::String>("output_topic", 10);
  }

private:
  ros::NodeHandle & node_;
  ros::Subscriber subscription_;
  ros::Publisher publisher_;
};

TEST(AutowareTestManagerTest, TestRelayNode)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "autoware_test_manager_test");
  }

  const std::string input_topic_name = "input_topic";
  const std::string output_topic_name = "output_topic";

  // Setup target node and its test manager
  ros::NodeHandle target_node;
  RelayNode relay(target_node);
  autoware::test_utils::AutowareTestManager manager;

  // Setup subscriber for test manager
  std::string received_msg;
  manager.set_subscriber<std_msgs::String>(
    "output_topic",
    [&received_msg](const std_msgs::String::ConstPtr & msg) { received_msg = msg->data; });

  // Publish a message to the relay node
  std_msgs::String msg;
  msg.data = "Hello, Relay!";
  manager.test_pub_msg<std_msgs::String>(target_node, input_topic_name, msg);

  // Check that the message was relayed and received by the test node
  EXPECT_EQ(received_msg, "Hello, Relay!");
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "autoware_test_manager_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
