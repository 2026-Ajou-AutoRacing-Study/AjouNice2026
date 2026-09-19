#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/processing_time_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <autoware_internal_debug_msgs/Float64Stamped.h>

#include <gtest/gtest.h>

#include <memory>

namespace
{
class RosFixture : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!ros::isInitialized()) { int argc = 0; ros::init(argc, nullptr, "autoware_utils_debug_actual_test", ros::init_options::AnonymousName); }
  }
};

TEST_F(RosFixture, DefaultQosProjectionRejectsNonDefault)
{
  EXPECT_EQ(autoware_utils_debug::ros1::only_source_default_qos().queue_size, 1u);
  EXPECT_FALSE(autoware_utils_debug::ros1::only_source_default_qos().latch);
  autoware_utils_debug::ros1::SourceDefaultQos bad; bad.depth = 2;
  EXPECT_THROW(autoware_utils_debug::ros1::only_source_default_qos(bad), std::invalid_argument);
}

TEST_F(RosFixture, DebugScalarPreservesNativeBuiltinTimestamp)
{
  const auto message=autoware_utils_debug::debug_publisher::to_debug_msg<
    autoware_internal_debug_msgs::Float64Stamped>(2.5,ros::Time(123,456));
  EXPECT_EQ(message.stamp.sec,123);
  EXPECT_EQ(message.stamp.nanosec,456U);
  EXPECT_DOUBLE_EQ(message.data,2.5);
  EXPECT_THROW((autoware_utils_debug::debug_publisher::to_debug_msg<
    autoware_internal_debug_msgs::Float64Stamped>(0.0,ros::Time(0x80000000U,0))),std::range_error);
}

TEST_F(RosFixture, ContextRejectsExpiredOwnerBeforePublication)
{
  auto owner = std::make_shared<ros::NodeHandle>();
  auto context = autoware_utils_debug::ros1::CallerNodeContext::borrow(owner);
  autoware_utils_debug::DebugPublisher publisher(context, "/debug");
  owner.reset();
  EXPECT_THROW(publisher.publish("flag", std_msgs::String{}), std::logic_error);
}

TEST_F(RosFixture, DebugPublisherCachesNameAndRejectsTypeReuse)
{
  auto owner = std::make_shared<ros::NodeHandle>();
  auto context = autoware_utils_debug::ros1::CallerNodeContext::borrow(owner);
  autoware_utils_debug::DebugPublisher publisher(context, "/debug");
  std_msgs::String text; text.data = "x";
  publisher.publish("value", text);
  publisher.publish("value", text);
  EXPECT_EQ(publisher.size(), 1u);
  EXPECT_THROW(publisher.publish("value", std_msgs::Bool{}), std::bad_cast);
}

TEST_F(RosFixture, SeparateFactoriesNeverMergeSameTopicPrimaryEndpoints)
{
  auto owner = std::make_shared<ros::NodeHandle>();
  auto context = autoware_utils_debug::ros1::CallerNodeContext::borrow(owner);
  autoware_utils_debug::ros1::PublisherFactory first_factory;
  autoware_utils_debug::ros1::PublisherFactory second_factory;
  auto first = first_factory.advertise<std_msgs::String>(*owner, "/primary", 1, false);
  auto second = second_factory.advertise<std_msgs::String>(*owner, "/primary", 1, false);
  const auto copied_first = first;
  EXPECT_NE(first.token().value, 0u);
  EXPECT_NE(second.token().value, 0u);
  EXPECT_NE(first.token().value, second.token().value);
  EXPECT_EQ(first.token().value, copied_first.token().value);
  autoware_utils_debug::ros1::PublishedTimeCache<autoware_utils_debug::PublishedTime> cache(context, "/debug/published_time");
  auto & first_endpoint = cache.observe(first);
  auto & same_endpoint = cache.observe(copied_first);
  auto & second_endpoint = cache.observe(second);
  EXPECT_EQ(&first_endpoint, &same_endpoint);
  EXPECT_NE(&first_endpoint, &second_endpoint);
  EXPECT_EQ(cache.size(), 2u);
  owner.reset();
  EXPECT_THROW(cache.observe(second), std::logic_error);
  EXPECT_EQ(cache.size(), 2u);
}

TEST_F(RosFixture, ProcessingPublisherUsesCallerLifetimeGuard)
{
  auto owner = std::make_shared<ros::NodeHandle>();
  auto context = autoware_utils_debug::ros1::CallerNodeContext::borrow(owner);
  autoware_utils_debug::ProcessingTimePublisher publisher(context);
  owner.reset();
  EXPECT_THROW(publisher.publish({{"one", 1.0}}), std::logic_error);
}
}
