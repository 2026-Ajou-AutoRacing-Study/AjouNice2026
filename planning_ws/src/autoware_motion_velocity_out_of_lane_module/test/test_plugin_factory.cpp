#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>

#include <class_loader/class_loader.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <string>

TEST(OutOfLanePluginFactory, CreatesRegisteredRos1PluginFromCandidateDso)
{
  const std::string class_name = "autoware::motion_velocity_planner::OutOfLaneModule";
  class_loader::ClassLoader loader(OUT_OF_LANE_LIBRARY_PATH);
  const auto available =
    loader.getAvailableClasses<autoware::motion_velocity_planner::PluginModuleInterface>();
  EXPECT_NE(std::find(available.begin(), available.end(), class_name), available.end());
  const auto plugin =
    loader.createInstance<autoware::motion_velocity_planner::PluginModuleInterface>(class_name);
  ASSERT_TRUE(plugin);
}

TEST(OutOfLanePluginFactory, InitializesWithRos1ProjectedParameterNames)
{
  ros::NodeHandle node("~");
  node.setParam("out_of_lane/mode", "ttc");
  node.setParam("out_of_lane/skip_if_already_overlapping", false);
  node.setParam("out_of_lane/max_arc_length", 100.0);
  node.setParam("out_of_lane/threshold/time_threshold", 5.0);
  node.setParam("out_of_lane/ttc/threshold", 1.0);
  node.setParam("out_of_lane/ttc/release_threshold", 3.0);
  node.setParam("out_of_lane/objects/minimum_velocity", 0.5);
  node.setParam("out_of_lane/objects/predicted_path_min_confidence", 0.1);
  node.setParam("out_of_lane/objects/cut_predicted_paths_beyond_red_lights", true);
  node.setParam("out_of_lane/objects/ignore_behind_ego", false);
  node.setParam("out_of_lane/objects/validate_predicted_paths_on_lanelets", true);
  node.setParam("out_of_lane/objects/extra_width", 1.0);
  node.setParam("out_of_lane/action/use_map_stop_lines", true);
  node.setParam("out_of_lane/action/precision", 0.1);
  node.setParam("out_of_lane/action/longitudinal_distance_buffer", 1.5);
  node.setParam("out_of_lane/action/lateral_distance_buffer", 1.0);
  node.setParam("out_of_lane/action/min_on_duration", 0.3);
  node.setParam("out_of_lane/action/min_off_duration", 1.0);
  node.setParam("out_of_lane/action/update_distance_th", 5.0);
  node.setParam("out_of_lane/action/slowdown/velocity", 2.0);
  node.setParam("out_of_lane/action/stop/distance_threshold", 15.0);
  node.setParam("out_of_lane/ego/extra_front_offset", 0.0);
  node.setParam("out_of_lane/ego/extra_rear_offset", 0.0);
  node.setParam("out_of_lane/ego/extra_right_offset", 0.0);
  node.setParam("out_of_lane/ego/extra_left_offset", 0.0);
  node.setParam("vehicle_info/max_longitudinal_offset_m", 3.74);
  node.setParam("vehicle_info/rear_overhang_m", 1.03);
  node.setParam("vehicle_info/vehicle_width_m", 1.83);
  node.setParam("vehicle_info/max_steer_angle", 0.70);
  node.setParam("vehicle_info/wheel_base_m", 2.74);
  node.setParam("vehicle_info/wheel_tread_m", 1.63);
  node.setParam("vehicle_info/front_overhang_m", 1.0);
  node.setParam("vehicle_info/left_overhang_m", 0.1);
  node.setParam("vehicle_info/right_overhang_m", 0.1);
  node.setParam("vehicle_info/vehicle_height_m", 2.5);
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

  class_loader::ClassLoader loader(OUT_OF_LANE_LIBRARY_PATH);
  const auto plugin = loader.createInstance<autoware::motion_velocity_planner::PluginModuleInterface>(
    "autoware::motion_velocity_planner::OutOfLaneModule");
  ASSERT_TRUE(plugin);
  try {
    plugin->init(node, "out_of_lane");
  } catch (const std::exception & error) {
    FAIL() << "plugin init failed: " << error.what();
  }
  EXPECT_EQ(plugin->getModuleName(), "out_of_lane");
  EXPECT_TRUE(plugin->getRequiredSubscriptions().predicted_objects);
  EXPECT_TRUE(plugin->getRequiredSubscriptions().lanelet_map);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "out_of_lane_plugin_factory_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
