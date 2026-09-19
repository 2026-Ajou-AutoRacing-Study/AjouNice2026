#include <autoware_utils_rclcpp/parameter.hpp>
#include <gtest/gtest.h>

TEST(Parameter, PresentMissingAndTypeMismatch)
{
  ros::NodeHandle node("/autoware_utils_rclcpp_parameter_test");
  node.setParam("integer", 12345); node.setParam("text", "str");
  EXPECT_EQ(autoware_utils_rclcpp::get_or_declare_parameter<int>(node, "integer"), 12345);
  EXPECT_EQ(autoware_utils_rclcpp::get_or_declare_parameter<std::string>(node, "text"), "str");
  EXPECT_EQ(autoware_utils_rclcpp::get_or_declare_parameter<int>(node, "missing"), 0);
  EXPECT_THROW(autoware_utils_rclcpp::get_or_declare_parameter<int>(node, "text"), std::runtime_error);
  node.setParam("whole", 7); node.setParam("fraction", 7.0);
  EXPECT_THROW(autoware_utils_rclcpp::get_or_declare_parameter<double>(node, "whole"), std::runtime_error);
  EXPECT_THROW(autoware_utils_rclcpp::get_or_declare_parameter<int>(node, "fraction"), std::runtime_error);
}

TEST(Parameter, UpdateOnlyExisting)
{
  ros::NodeHandle node("/autoware_utils_rclcpp_parameter_update");
  node.setParam("value", 42); int value = 0;
  EXPECT_TRUE(autoware_utils_rclcpp::update_param(node, "value", value)); EXPECT_EQ(value, 42);
  EXPECT_FALSE(autoware_utils_rclcpp::update_param(node, "none", value)); EXPECT_EQ(value, 42);
}
