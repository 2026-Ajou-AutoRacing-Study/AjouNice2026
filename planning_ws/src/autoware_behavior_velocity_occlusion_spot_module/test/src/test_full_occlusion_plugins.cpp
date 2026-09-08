#include <autoware/behavior_velocity_occlusion_spot_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void check(const std::string & base, const std::string & prefix,
                                     const std::string & method)
{
  ros::NodeHandle node("/full_occlusion_spot_test");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_occlusion_spot_module)/config/ros1/occlusion_spot.yaml\" /full_occlusion_spot_test"), 0);
  node.setParam("wheel_radius", 0.39); node.setParam("wheel_width", 0.42);
  node.setParam("wheel_base", 2.74); node.setParam("wheel_tread", 1.63);
  node.setParam("front_overhang", 1.0); node.setParam("rear_overhang", 1.03);
  node.setParam("left_overhang", 0.1); node.setParam("right_overhang", 0.1);
  node.setParam("vehicle_height", 2.5); node.setParam("max_steer_angle", 0.70);
  node.setParam("occlusion_spot/detection_method", method);
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto plugin = loader.createUniqueInstance(prefix + "OcclusionSpotModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "occlusion_spot");
  const auto required = plugin->getRequiredSubscriptions();
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_TRUE(required.occupancy_grid_map);
  EXPECT_FALSE(required.traffic_signals);
}
TEST(FullOcclusionPlugins, NormalOccupancy) { check<bvp::PluginInterface>(
  "autoware::behavior_velocity_planner::PluginInterface", "autoware::behavior_velocity_planner::", "occupancy_grid"); }
TEST(FullOcclusionPlugins, NormalPredictedObjects) { check<bvp::PluginInterface>(
  "autoware::behavior_velocity_planner::PluginInterface", "autoware::behavior_velocity_planner::", "predicted_object"); }
TEST(FullOcclusionPlugins, ExperimentalOccupancy) { check<bvp::experimental::PluginInterface>(
  "autoware::behavior_velocity_planner::experimental::PluginInterface", "autoware::behavior_velocity_planner::experimental::", "occupancy_grid"); }
TEST(FullOcclusionPlugins, ExperimentalPredictedObjects) { check<bvp::experimental::PluginInterface>(
  "autoware::behavior_velocity_planner::experimental::PluginInterface", "autoware::behavior_velocity_planner::experimental::", "predicted_object"); }
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_occlusion_plugins");
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
