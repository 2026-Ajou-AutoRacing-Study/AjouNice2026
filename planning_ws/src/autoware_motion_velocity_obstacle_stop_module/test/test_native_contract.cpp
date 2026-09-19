#include "obstacle_stop_module.hpp"
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>

TEST(MotionObstacleNative, InstalledPluginLifecycle)
{
  using namespace autoware::motion_velocity_planner;
  pluginlib::ClassLoader<PluginModuleInterface> loader(
    "autoware_motion_velocity_planner_common", "autoware::motion_velocity_planner::PluginModuleInterface");
  auto module=loader.createUniqueInstance("autoware::motion_velocity_planner::ObstacleStopModule");
  ros::NodeHandle node("/full_motion_obstacle_test");
  module->init(node,"native_obstacle_stop");
  EXPECT_EQ(module->getModuleName(),"native_obstacle_stop");
  EXPECT_EQ(module->getShortModuleName(),"obstacle_stop");
  const auto required=module->getRequiredSubscriptions();
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_TRUE(required.no_ground_pointcloud);
  EXPECT_FALSE(required.traffic_signals);
  EXPECT_FALSE(required.occupancy_grid_map);
  EXPECT_FALSE(required.lanelet_map);
  ParameterMap values;
  values.emplace("obstacle_stop.option.ignore_crossing_obstacle",XmlRpc::XmlRpcValue(true));
  values.emplace("obstacle_stop/option/suppress_sudden_stop",XmlRpc::XmlRpcValue(false));
  EXPECT_NO_THROW(module->updateParameters(values));
  EXPECT_NO_THROW(module->publishPlanningFactor());
}

TEST(MotionObstacleNative, MissingParametersFailInsteadOfSilentZeroDefaults)
{
  ros::NodeHandle empty("/full_motion_obstacle_missing_parameters");
  autoware::motion_velocity_planner::ObstacleStopModule module;
  EXPECT_ANY_THROW(module.init(empty,"obstacle_stop"));
  EXPECT_ANY_THROW(autoware::motion_velocity_planner::PlannerData{empty});
}

TEST(MotionObstacleNative, ParameterBackedPlannerDataPreservesConfiguration)
{
  ros::NodeHandle node("/full_motion_obstacle_test");
  const autoware::motion_velocity_planner::PlannerData data(node);
  double expected=0.0;
  ASSERT_TRUE(node.getParam("ego_nearest_dist_threshold",expected));
  EXPECT_DOUBLE_EQ(data.ego_nearest_dist_threshold,expected);
  ASSERT_TRUE(node.getParam("trajectory_polygon_collision_check/decimate_trajectory_step_length",expected));
  EXPECT_DOUBLE_EQ(data.trajectory_polygon_collision_check.decimate_trajectory_step_length,expected);
  ASSERT_TRUE(node.getParam("wheel_base",expected));
  EXPECT_DOUBLE_EQ(data.vehicle_info_.wheel_base_m,expected);
}
