// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_lane_change_module/manager.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
#include <limits>
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
TEST(LaneChangeNativeParameterEvents, RecoveryTimeLoadsAndRejectsInvalidUpdates) {
  ros::NodeHandle node("/lane_change_native_events");
  node.setParam("lane_change/execution/recovery_time",0.75);
  ParameterEventManager manager;ASSERT_TRUE(manager.initialize(&node));
  EXPECT_DOUBLE_EQ(manager.parameters().execution_recovery_time,0.75);
  manager.updateModuleParams({rclcpp::Parameter("lane_change.execution.recovery_time",1.25)});
  EXPECT_DOUBLE_EQ(manager.parameters().execution_recovery_time,1.25);
  manager.updateModuleParams({rclcpp::Parameter("lane_change.execution.recovery_time",-1.0)});
  EXPECT_DOUBLE_EQ(manager.parameters().execution_recovery_time,1.25);
  manager.updateModuleParams({rclcpp::Parameter("lane_change.execution.recovery_time",
    std::numeric_limits<double>::quiet_NaN())});
  EXPECT_DOUBLE_EQ(manager.parameters().execution_recovery_time,1.25);
  node.setParam("lane_change/execution/recovery_time",0.0);
  ParameterEventManager invalid;EXPECT_THROW(invalid.initialize(&node),std::invalid_argument);
  node.setParam("lane_change/execution/recovery_time",1.0);
}
TEST(LaneChangeNativeParameterEvents, ApprovedReplacementToggleLoadsAndUpdates) {
  ros::NodeHandle node("/lane_change_native_events");
  node.setParam("lane_change/execution/always_update_approved",false);
  ParameterEventManager manager;ASSERT_TRUE(manager.initialize(&node));
  EXPECT_FALSE(manager.parameters().always_update_approved);
  EXPECT_EQ(manager.parameters().trajectory.prepare_duration_samples,
    std::vector<double>({0,.5,1,1.5,2,2.5,4}));
  manager.updateModuleParams({rclcpp::Parameter("lane_change.execution.always_update_approved",true)});
  EXPECT_TRUE(manager.parameters().always_update_approved);
  manager.updateModuleParams({rclcpp::Parameter("lane_change.execution.always_update_approved",false)});
  EXPECT_FALSE(manager.parameters().always_update_approved);
}

TEST(LaneChangeNativeParameterEvents, DeceleratingLaneChangePolicyDefaultsOffAndUpdates) {
  ros::NodeHandle node("/lane_change_native_events");
  const std::string key = "lane_change/trajectory/allow_decelerating_lane_change";
  node.deleteParam(key);
  ParameterEventManager defaults;ASSERT_TRUE(defaults.initialize(&node));
  EXPECT_FALSE(defaults.parameters().trajectory.allow_decelerating_lane_change);
  node.setParam(key, true);
  ParameterEventManager enabled;ASSERT_TRUE(enabled.initialize(&node));
  EXPECT_TRUE(enabled.parameters().trajectory.allow_decelerating_lane_change);
  enabled.updateModuleParams({rclcpp::Parameter(
    "lane_change.trajectory.allow_decelerating_lane_change", false)});
  EXPECT_FALSE(enabled.parameters().trajectory.allow_decelerating_lane_change);
  enabled.updateModuleParams({rclcpp::Parameter(
    "lane_change.trajectory.allow_decelerating_lane_change", true)});
  EXPECT_TRUE(enabled.parameters().trajectory.allow_decelerating_lane_change);
  node.setParam(key, false);
}

}
int main(int argc,char ** argv) {
  ros::init(argc,argv,"lane_change_native_events");
  const int load_result = std::system(
    "rosrun autoware_behavior_path_lane_change_module load_ros1_parameters.py /lane_change_native_events "
    "--source \"$(rospack find autoware_behavior_path_lane_change_module)/config/"
    "lane_change.param.yaml\" "
    "--source \"$(rospack find autoware_test_utils)/config/test_common.param.yaml\"");
  if (load_result != 0) return EXIT_FAILURE;
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
