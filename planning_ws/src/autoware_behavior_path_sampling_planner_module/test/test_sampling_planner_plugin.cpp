#include "autoware/behavior_path_sampling_planner_module/manager.hpp"

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

TEST(SamplingPlannerPlugin, LoadsManagerClass)
{
  pluginlib::ClassLoader<autoware::behavior_path_planner::SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");
  const auto instance = loader.createInstance(
    "autoware::behavior_path_planner::SamplingPlannerModuleManager");
  ASSERT_TRUE(instance);
  EXPECT_EQ(instance->name(), "sampling_planner");
}

TEST(SamplingPlannerPlugin, CompleteRos1ParametersInitializeAndCreateIdleScene)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char ** argv = nullptr;
    ros::init(argc, argv, "sampling_planner_manager_runtime");
  }
  ros::NodeHandle node("sampling_planner_manager_runtime");
  node.setParam("constraints/hard/max_curvature", 3.0);
  node.setParam("constraints/hard/min_curvature", -3.0);
  node.setParam("constraints/soft/lateral_deviation_weight", 1.0);
  node.setParam("constraints/soft/length_weight", 1.0);
  node.setParam("constraints/soft/curvature_weight", 2000.0);
  node.setParam("constraints/soft/weights", std::vector<double>{0.5, 1.0, 20.0});
  node.setParam("sampling/enable_frenet", true);
  node.setParam("sampling/enable_bezier", false);
  node.setParam("sampling/resolution", 0.5);
  node.setParam("sampling/previous_path_reuse_points_nb", 2);
  node.setParam("sampling/nb_target_lateral_positions", 0);
  node.setParam("sampling/target_lengths", std::vector<double>{20.0, 40.0});
  node.setParam("sampling/target_lateral_positions", std::vector<double>{-4.5, -2.5, 0.0, 2.5, 4.5});
  node.setParam("sampling/frenet/target_lateral_velocities", std::vector<double>{-0.5, 0.0, 0.5});
  node.setParam("sampling/frenet/target_lateral_accelerations", std::vector<double>{0.0});
  node.setParam("preprocessing/force_zero_initial_deviation", true);
  node.setParam("preprocessing/force_zero_initial_heading", true);
  node.setParam("preprocessing/smooth_reference_trajectory", false);
  node.setParam("wheel_radius", 0.5);
  node.setParam("wheel_width", 0.3);
  node.setParam("wheel_base", 2.7);
  node.setParam("wheel_tread", 1.6);
  node.setParam("front_overhang", 1.0);
  node.setParam("rear_overhang", 1.1);
  node.setParam("left_overhang", 0.2);
  node.setParam("right_overhang", 0.2);
  node.setParam("vehicle_height", 1.8);
  node.setParam("max_steer_angle", 0.6);

  pluginlib::ClassLoader<autoware::behavior_path_planner::SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");
  const auto instance = loader.createInstance(
    "autoware::behavior_path_planner::SamplingPlannerModuleManager");
  const auto manager = boost::dynamic_pointer_cast<
    autoware::behavior_path_planner::SamplingPlannerModuleManager>(instance);
  ASSERT_NE(manager, nullptr);
  ASSERT_NO_THROW(manager->init(&node));
  EXPECT_TRUE(manager->hasPlanningFactorInterface());
  EXPECT_EQ(manager->rtcInterfaceCount(), 1U);
  EXPECT_EQ(manager->markerInterfaceCount(), 1U);
  ASSERT_NO_THROW(manager->updateIdleModuleInstance());
  EXPECT_NE(manager->takeIdleModule(), nullptr);
}
