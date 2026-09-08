#include "autoware_utils_logging/logger_level_configure.hpp"

#include <logging_demo/ConfigLogger.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <memory>
#include <map>

namespace
{
class LoggerLevelConfigureServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    private_nh_ = std::make_unique<ros::NodeHandle>("~");
    configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(*private_nh_);
    spinner_ = std::make_unique<ros::AsyncSpinner>(1);
    spinner_->start();
    client_ = private_nh_->serviceClient<logging_demo::ConfigLogger>("config_logger");
    ASSERT_TRUE(client_.waitForExistence(ros::Duration(2.0)));
  }

  void TearDown() override
  {
    configure_.reset();
    spinner_->stop();
  }

  std::unique_ptr<ros::NodeHandle> private_nh_;
  std::unique_ptr<autoware_utils_logging::LoggerLevelConfigure> configure_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::ServiceClient client_;
};

TEST_F(LoggerLevelConfigureServiceTest, PreservesNormalSuccessResponse)
{
  logging_demo::ConfigLogger service;
  service.request.logger_name = "ros";
  service.request.level = "INFO";
  ASSERT_TRUE(client_.call(service));
  EXPECT_TRUE(service.response.success);
}

TEST_F(LoggerLevelConfigureServiceTest, InvalidLevelIsApplicationFailureNotTransportFailure)
{
  logging_demo::ConfigLogger service;
  service.request.logger_name = "autoware_utils_logging.test";
  service.request.level = "NOT_A_LEVEL";
  ASSERT_TRUE(client_.call(service));
  EXPECT_FALSE(service.response.success);
}

TEST_F(LoggerLevelConfigureServiceTest, UnsetIsRejectedWithoutChangingLoggerConfiguration)
{
  logging_demo::ConfigLogger set_level;
  set_level.request.logger_name = "ros";
  set_level.request.level = "WARN";
  ASSERT_TRUE(client_.call(set_level));
  ASSERT_TRUE(set_level.response.success);

  logging_demo::ConfigLogger unset;
  unset.request.logger_name = "ros";
  unset.request.level = "UNSET";
  ASSERT_TRUE(client_.call(unset));
  EXPECT_FALSE(unset.response.success);

  std::map<std::string, ros::console::levels::Level> loggers;
  ASSERT_TRUE(ros::console::get_loggers(loggers));
  const auto logger = loggers.find("ros");
  ASSERT_NE(logger, loggers.end());
  EXPECT_EQ(logger->second, ros::console::levels::Warn);
}
}  // namespace
