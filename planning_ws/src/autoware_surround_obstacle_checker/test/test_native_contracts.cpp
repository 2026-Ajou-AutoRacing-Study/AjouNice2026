// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include "../src/full_native_support.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
using surround_obstacle_checker_node::ParamListener;
TEST(SurroundNativeContracts, MissingParametersFailInitialization) {
  ros::NodeHandle node("/surround_missing_parameters");
  EXPECT_THROW(ParamListener{node},std::runtime_error);
}
TEST(SurroundNativeContracts, OriginalSchemaDefaultsAndRuntimeUpdate) {
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_surround_obstacle_checker)/config/ros1/surround_obstacle_checker.yaml\" /surround_native_parameters"),0);
  ros::NodeHandle node("/surround_native_parameters");
  ParamListener listener(node);
  const auto initial=listener.get_params();
  EXPECT_EQ(initial.object_types.size(),8U);EXPECT_EQ(initial.obstacle_types.size(),9U);
  EXPECT_DOUBLE_EQ(initial.state_clear_time,0.2);
  EXPECT_TRUE(initial.object_types_map.at("car").enable_check);
  node.setParam("car/enable_check",false);
  EXPECT_FALSE(listener.get_params().object_types_map.at("car").enable_check);
  node.setParam("car/enable_check",true);
}
TEST(SurroundNativeContracts, RejectInvalidUpdateWithoutReplacingLastValidSnapshot) {
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_surround_obstacle_checker)/config/ros1/surround_obstacle_checker.yaml\" /surround_native_parameters"),0);
  ros::NodeHandle node("/surround_native_parameters");
  ParamListener listener(node);
  node.setParam("state_clear_time",-1.0);
  EXPECT_DOUBLE_EQ(listener.get_params().state_clear_time,0.2);
  EXPECT_THROW(ParamListener{node},std::invalid_argument);
  node.setParam("state_clear_time",0.2);
  node.setParam("object_types",42);
  EXPECT_EQ(listener.get_params().object_types.size(),8U);
  EXPECT_THROW(ParamListener{node},std::runtime_error);
  node.deleteParam("object_types");
}
TEST(SurroundNativeContracts, TimestampNanosecondsAndSignedSecondsBoundary) {
  const auto converted=autoware::surround_obstacle_checker::full_native::stamp(ros::Time(123,456));
  EXPECT_EQ(converted.sec,123);EXPECT_EQ(converted.nanosec,456U);
  EXPECT_THROW(autoware::surround_obstacle_checker::full_native::stamp(ros::Time(2147483648U,0)),std::out_of_range);
}
int main(int argc,char ** argv) {
  ros::init(argc,argv,"surround_native_contracts");
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
