#include <autoware/behavior_velocity_no_stopping_area_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void check(const std::string & base, const std::string & prefix)
{
  ros::NodeHandle node("/full_no_stopping_area_test");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_no_stopping_area_module)/config/ros1/no_stopping_area.yaml\" /full_no_stopping_area_test"), 0);
  node.setParam("wheel_radius", 0.39); node.setParam("wheel_width", 0.42);
  node.setParam("wheel_base", 2.74); node.setParam("wheel_tread", 1.63);
  node.setParam("front_overhang", 1.0); node.setParam("rear_overhang", 1.03);
  node.setParam("left_overhang", 0.1); node.setParam("right_overhang", 0.1);
  node.setParam("vehicle_height", 2.5); node.setParam("max_steer_angle", 0.70);
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto plugin = loader.createUniqueInstance(prefix + "NoStoppingAreaModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "no_stopping_area");
  const auto required = plugin->getRequiredSubscriptions();
  EXPECT_FALSE(required.traffic_signals);
  EXPECT_TRUE(required.predicted_objects);  // Explicit required-input defect correction.
  EXPECT_FALSE(required.no_ground_pointcloud);
  EXPECT_FALSE(required.occupancy_grid_map);
}
TEST(FullNoStoppingPlugins, Normal) { check<bvp::PluginInterface>(
  "autoware::behavior_velocity_planner::PluginInterface", "autoware::behavior_velocity_planner::"); }
TEST(FullNoStoppingPlugins, Experimental) { check<bvp::experimental::PluginInterface>(
  "autoware::behavior_velocity_planner::experimental::PluginInterface", "autoware::behavior_velocity_planner::experimental::"); }
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_traffic_plugins");
  ros::NodeHandle owner;
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
