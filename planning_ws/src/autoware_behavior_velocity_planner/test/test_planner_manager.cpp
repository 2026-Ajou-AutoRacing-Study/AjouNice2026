#include <autoware/behavior_velocity_planner/planner_manager.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{
TEST(BehaviorVelocityPlannerManager, EmptyPluginSetPreservesInputPathAndRequirements)
{
  BehaviorVelocityPlannerManager manager;
  auto planner_data = std::make_shared<PlannerData>();
  autoware_internal_planning_msgs::PathWithLaneId input;
  input.header.frame_id = "map";
  input.points.resize(2);
  input.points[0].lane_ids = {101};
  input.points[1].lane_ids = {202};

  const auto output = manager.planPathVelocity(planner_data, input);
  EXPECT_EQ(output.header.frame_id, "map");
  ASSERT_EQ(output.points.size(), 2U);
  EXPECT_EQ(output.points[0].lane_ids.front(), 101);
  EXPECT_EQ(output.points[1].lane_ids.front(), 202);

  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.predicted_objects);
  EXPECT_FALSE(requirements.occupancy_grid_map);
  EXPECT_FALSE(requirements.no_ground_pointcloud);
}

TEST(BehaviorVelocityPlannerManager, LoadsInstalledStopLinePluginThroughCommonInterfaceExport)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "behavior_velocity_planner_manager_test", ros::init_options::AnonymousName);
  }
  ros::NodeHandle node;
  BehaviorVelocityPlannerManager manager;
  manager.launchScenePlugin(node, "autoware::behavior_velocity_planner::StopLineModulePlugin");

  const auto requirements = manager.getRequiredSubscriptions();
  EXPECT_FALSE(requirements.traffic_signals);
  EXPECT_FALSE(requirements.predicted_objects);
  EXPECT_FALSE(requirements.occupancy_grid_map);
  EXPECT_FALSE(requirements.no_ground_pointcloud);
}

TEST(BehaviorVelocityPlannerManager, LoadsStandardNoStoppingAreaPluginAndPropagatesObjectInput)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "behavior_velocity_planner_no_stopping_manager_test", ros::init_options::AnonymousName);
  }
  ros::NodeHandle node;
  node.setParam("wheel_radius", 0.39); node.setParam("wheel_width", 0.42);
  node.setParam("wheel_base", 2.74); node.setParam("wheel_tread", 1.63);
  node.setParam("front_overhang", 1.0); node.setParam("rear_overhang", 1.03);
  node.setParam("left_overhang", 0.1); node.setParam("right_overhang", 0.1);
  node.setParam("vehicle_height", 2.5); node.setParam("max_steer_angle", 0.70);
  node.setParam("enable_all_modules_auto_mode", false);
  node.setParam("no_stopping_area/enable_rtc", true);
  BehaviorVelocityPlannerManager manager;
  manager.launchScenePlugin(
    node, "autoware::behavior_velocity_planner::NoStoppingAreaModulePlugin");
  EXPECT_TRUE(manager.getRequiredSubscriptions().predicted_objects);
}
}  // namespace autoware::behavior_velocity_planner
