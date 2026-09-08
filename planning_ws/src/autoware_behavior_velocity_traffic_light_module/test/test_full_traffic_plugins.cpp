#include <autoware/behavior_velocity_traffic_light_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void check(const std::string & base, const std::string & prefix)
{
  ros::NodeHandle node("/full_traffic_light_test");
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto plugin = loader.createUniqueInstance(prefix + "TrafficLightModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "traffic_light");
  const auto required = plugin->getRequiredSubscriptions();
  EXPECT_TRUE(required.traffic_signals);
  EXPECT_FALSE(required.predicted_objects);
  EXPECT_FALSE(required.no_ground_pointcloud);
  EXPECT_FALSE(required.occupancy_grid_map);
}
TEST(FullTrafficPlugins, Normal) { check<bvp::PluginInterface>(
  "autoware::behavior_velocity_planner::PluginInterface", "autoware::behavior_velocity_planner::"); }
TEST(FullTrafficPlugins, Experimental) { check<bvp::experimental::PluginInterface>(
  "autoware::behavior_velocity_planner::experimental::PluginInterface", "autoware::behavior_velocity_planner::experimental::"); }
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_traffic_plugins");
  ros::NodeHandle owner;
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
