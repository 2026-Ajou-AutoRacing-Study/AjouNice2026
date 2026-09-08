#include <gtest/gtest.h>
#include <ros/ros.h>

#include <atomic>

#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_specs/map.hpp>
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_specs/system.hpp>

namespace specs = autoware::component_interface_specs;

template <class Spec>
void expect_latched_replay(ros::NodeHandle & node)
{
  using Message = typename Spec::Message;
  const auto qos = specs::get_qos<Spec>();
  ASSERT_TRUE(qos.latch);
  auto publisher = node.advertise<Message>(Spec::name, qos.queue_size, qos.latch);
  ASSERT_TRUE(publisher);
  publisher.publish(Message{});
  ros::WallDuration(0.05).sleep();
  std::atomic<int> received{0};
  auto subscriber = node.subscribe<Message>(Spec::name, qos.queue_size,
    [&received](const typename Message::ConstPtr &) { ++received; });
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::ok() && received.load() == 0 && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(received.load(), 1);
}

TEST(QosRuntime, TransientLocalReplaysToLateSubscriber)
{
  ros::NodeHandle node;
  expect_latched_replay<specs::localization::InitializationState>(node);
  expect_latched_replay<specs::map::MapProjectorInfo>(node);
  expect_latched_replay<specs::map::PointCloudMap>(node);
  expect_latched_replay<specs::map::VectorMap>(node);
  expect_latched_replay<specs::planning::RouteState>(node);
  expect_latched_replay<specs::planning::LaneletRoute>(node);
  expect_latched_replay<specs::system::OperationModeState>(node);
}

TEST(QosRuntime, VolatileDoesNotReplayToLateSubscriber)
{
  using Spec = specs::planning::Trajectory;
  using Message = typename Spec::Message;
  ros::NodeHandle node;
  const auto qos = specs::get_qos<Spec>();
  ASSERT_FALSE(qos.latch);
  auto publisher = node.advertise<Message>(Spec::name, qos.queue_size, qos.latch);
  publisher.publish(Message{});
  ros::WallDuration(0.05).sleep();
  std::atomic<int> received{0};
  auto subscriber = node.subscribe<Message>(Spec::name, qos.queue_size,
    [&received](const Message::ConstPtr &) { ++received; });
  const auto deadline = ros::WallTime::now() + ros::WallDuration(0.4);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(received.load(), 0);
}
