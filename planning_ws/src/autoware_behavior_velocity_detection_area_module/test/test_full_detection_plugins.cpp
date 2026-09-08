#include <autoware/behavior_velocity_detection_area_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void check(const std::string & base, const std::string & prefix, bool predicted)
{
  ros::NodeHandle node("/full_detection_area_test");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_detection_area_module)/config/ros1/detection_area.yaml\" /full_detection_area_test"), 0);
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto plugin = loader.createUniqueInstance(prefix + "DetectionAreaModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "detection_area");
  const auto required = plugin->getRequiredSubscriptions();
  // Both variants must collect objects for their object-detection fallback.
  EXPECT_EQ(required.predicted_objects, predicted);
  EXPECT_TRUE(required.no_ground_pointcloud);
  EXPECT_FALSE(required.occupancy_grid_map);
  EXPECT_FALSE(required.traffic_signals);
}
TEST(FullDetectionPlugins, Normal) { check<bvp::PluginInterface>(
  "autoware::behavior_velocity_planner::PluginInterface", "autoware::behavior_velocity_planner::", true); }
TEST(FullDetectionPlugins, Experimental) { check<bvp::experimental::PluginInterface>(
  "autoware::behavior_velocity_planner::experimental::PluginInterface", "autoware::behavior_velocity_planner::experimental::", true); }
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_detection_plugins");
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
