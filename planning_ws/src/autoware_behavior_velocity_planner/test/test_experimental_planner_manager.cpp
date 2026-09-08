#include <autoware/behavior_velocity_planner/experimental/planner_manager.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner::experimental
{
TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsNoDrivableLanePluginThroughRos1ExperimentalExport)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::NoDrivableLaneModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.predicted_objects);
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.no_ground_pointcloud);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsNoStoppingAreaRtcPluginAndPropagatesObjectInput)
{
  ros::NodeHandle node;
  node.setParam("wheel_radius", 0.39);
  node.setParam("wheel_width", 0.42);
  node.setParam("wheel_base", 2.74);
  node.setParam("wheel_tread", 1.63);
  node.setParam("front_overhang", 1.0);
  node.setParam("rear_overhang", 1.03);
  node.setParam("left_overhang", 0.1);
  node.setParam("right_overhang", 0.1);
  node.setParam("vehicle_height", 2.5);
  node.setParam("max_steer_angle", 0.70);
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::NoStoppingAreaModulePlugin"));
  EXPECT_TRUE(manager.getRequiredSubscriptions().predicted_objects);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsDefaultLaunchStopLineThroughExperimentalAdapter)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::StopLineModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.predicted_objects);
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.no_ground_pointcloud);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsDefaultLaunchMergeFromPrivatePlugin)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::MergeFromPrivateModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.predicted_objects);
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.occupancy_grid_map);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsCrosswalkAndPropagatesItsInputContract)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::CrosswalkModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_TRUE(requirements.predicted_objects);
  EXPECT_TRUE(requirements.traffic_signals);
  EXPECT_TRUE(requirements.occupancy_grid_map);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsDetectionAreaAndPropagatesItsInputContract)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::DetectionAreaModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_TRUE(requirements.no_ground_pointcloud);
  EXPECT_TRUE(requirements.predicted_objects);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsTrafficLightAndPropagatesItsInputContract)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::TrafficLightModulePlugin"));
  EXPECT_TRUE(manager.getRequiredSubscriptions().traffic_signals);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsVirtualTrafficLightRos1V2xPlugin)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::VirtualTrafficLightModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.predicted_objects);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsBlindSpotAndPropagatesItsInputContract)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::BlindSpotModulePlugin"));
  EXPECT_TRUE(manager.getRequiredSubscriptions().predicted_objects);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsIntersectionAndPropagatesItsSafetyInputs)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::IntersectionModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_TRUE(requirements.predicted_objects);
  EXPECT_TRUE(requirements.traffic_signals);
  EXPECT_TRUE(requirements.occupancy_grid_map);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsRoundaboutAndPropagatesItsInputContract)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::RoundaboutModulePlugin"));
  EXPECT_TRUE(manager.getRequiredSubscriptions().predicted_objects);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsSpeedBumpWithoutAdditionalInputs)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::SpeedBumpModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.predicted_objects);
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.occupancy_grid_map);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsWalkwayWithoutAdditionalInputs)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::WalkwayModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.predicted_objects);
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.occupancy_grid_map);
}

TEST(ExperimentalBehaviorVelocityPlannerManager, LoadsOcclusionSpotAndPropagatesItsInputContract)
{
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  ASSERT_TRUE(manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::experimental::OcclusionSpotModulePlugin"));
  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_TRUE(requirements.predicted_objects);
  EXPECT_TRUE(requirements.occupancy_grid_map);
}
}  // namespace autoware::behavior_velocity_planner::experimental

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "experimental_behavior_velocity_planner_manager_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
