// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_lane_change_module/manager.hpp>
#include <gtest/gtest.h>
namespace autoware::behavior_path_planner {
class ParameterEventManager : public LaneChangeLeftModuleManager {
public:
  const LaneChangeParameters & parameters() const {return *parameters_;}
};
TEST(LaneChangeNativeParameterEvents, ProviderInitializationLoadsRequiredSourceParameters) {
  ros::NodeHandle node("/lane_change_native_events");ParameterEventManager manager;
  SceneModuleManagerInterface * base=&manager;
  ASSERT_TRUE(base->initialize(&node));
  EXPECT_DOUBLE_EQ(manager.parameters().time_limit,50.0);
  EXPECT_DOUBLE_EQ(manager.parameters().backward_lane_length,200.0);
}
TEST(LaneChangeNativeParameterEvents, MissingRequiredParametersAreNotZeroDefaults) {
  ros::NodeHandle node("/lane_change_native_missing");ParameterEventManager manager;
  EXPECT_THROW(manager.initialize(&node),std::runtime_error);
}
TEST(LaneChangeNativeParameterEvents, OriginalCallbackUpdatesScalarsAndRetainsRejectedValue) {
  ros::NodeHandle node("/lane_change_native_events");ParameterEventManager manager;
  ASSERT_TRUE(manager.initialize(&node));
  manager.updateModuleParams({rclcpp::Parameter("lane_change.time_limit",70.0),
    rclcpp::Parameter("lane_change/backward_lane_length",180.0),
    rclcpp::Parameter("lane_change.publish_debug_marker",false)});
  EXPECT_DOUBLE_EQ(manager.parameters().time_limit,70.0);
  EXPECT_DOUBLE_EQ(manager.parameters().backward_lane_length,180.0);
  EXPECT_FALSE(manager.parameters().publish_debug_marker);
  manager.updateModuleParams({rclcpp::Parameter("lane_change.time_limit",5.0)});
  EXPECT_DOUBLE_EQ(manager.parameters().time_limit,70.0);
}
}
int main(int argc,char ** argv) {
  ros::init(argc,argv,"lane_change_native_events");
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
