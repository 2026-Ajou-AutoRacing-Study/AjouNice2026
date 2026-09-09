#include <autoware/behavior_velocity_intersection_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void checkPlugin(const std::string & base, const std::string & name, bool intersection)
{
  ros::NodeHandle node("/full_intersection_test");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_planner)/config/ros1/behavior_velocity_planner.yaml\" /full_intersection_test"), 0);
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_intersection_module)/config/ros1/intersection.yaml\" /full_intersection_test"), 0);
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto plugin = loader.createUniqueInstance(name);
  ASSERT_TRUE(plugin);
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), intersection ? "intersection" : "merge_from_private");
  const auto required = plugin->getRequiredSubscriptions();
  EXPECT_EQ(required.predicted_objects, intersection);
  EXPECT_EQ(required.traffic_signals, intersection);
  EXPECT_EQ(required.occupancy_grid_map, intersection);
}
template<class Interface> void checkIntersectionPluginWithoutOcclusion(
  const std::string & base, const std::string & name)
{
  ros::NodeHandle node("/full_intersection_test_without_occlusion");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_planner)/config/ros1/behavior_velocity_planner.yaml\" /full_intersection_test_without_occlusion"), 0);
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_intersection_module)/config/ros1/intersection.yaml\" /full_intersection_test_without_occlusion"), 0);
  node.setParam("intersection/occlusion/enable", false);
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto plugin = loader.createUniqueInstance(name);
  ASSERT_TRUE(plugin);
  plugin->init(node);
  const auto required = plugin->getRequiredSubscriptions();
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_TRUE(required.traffic_signals);
  EXPECT_FALSE(required.occupancy_grid_map);
}
TEST(FullIntersection, NormalPlugin)
{
  checkPlugin<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface",
    "autoware::behavior_velocity_planner::IntersectionModulePlugin", true);
}
TEST(FullIntersection, ExperimentalPlugin)
{
  checkPlugin<bvp::experimental::PluginInterface>("autoware::behavior_velocity_planner::experimental::PluginInterface",
    "autoware::behavior_velocity_planner::experimental::IntersectionModulePlugin", true);
}
TEST(FullIntersection, NormalPluginDoesNotRequireOccupancyWhenOcclusionIsDisabled)
{
  checkIntersectionPluginWithoutOcclusion<bvp::PluginInterface>(
    "autoware::behavior_velocity_planner::PluginInterface",
    "autoware::behavior_velocity_planner::IntersectionModulePlugin");
}
TEST(FullIntersection, ExperimentalPluginDoesNotRequireOccupancyWhenOcclusionIsDisabled)
{
  checkIntersectionPluginWithoutOcclusion<bvp::experimental::PluginInterface>(
    "autoware::behavior_velocity_planner::experimental::PluginInterface",
    "autoware::behavior_velocity_planner::experimental::IntersectionModulePlugin");
}
TEST(FullIntersection, NormalPrivateRoadPlugin)
{
  checkPlugin<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface",
    "autoware::behavior_velocity_planner::MergeFromPrivateModulePlugin", false);
}
TEST(FullIntersection, ExperimentalPrivateRoadPlugin)
{
  checkPlugin<bvp::experimental::PluginInterface>("autoware::behavior_velocity_planner::experimental::PluginInterface",
    "autoware::behavior_velocity_planner::experimental::MergeFromPrivateModulePlugin", false);
}
TEST(FullIntersection, IntegerIdsRejectLossAndPreserveFullWidthStrings)
{
  ros::NodeHandle node("/full_intersection_test/id_test");
  XmlRpc::XmlRpcValue values;
  values.setSize(3);
  values[0] = 42;
  values[1] = std::string("9223372036854775807");
  values[2] = std::string("-9223372036854775808");
  node.setParam("ids", values);
  const auto parsed = bvp::intersection_ros1::require_parameter<std::vector<int64_t>>(node, "ids");
  EXPECT_EQ(parsed, (std::vector<int64_t>{42, INT64_MAX, INT64_MIN}));
  values[0] = 1.5;
  node.setParam("ids", values);
  EXPECT_THROW(bvp::intersection_ros1::require_parameter<std::vector<int64_t>>(node, "ids"), std::exception);
  values[0] = std::string("9223372036854775808");
  node.setParam("ids", values);
  EXPECT_THROW(bvp::intersection_ros1::require_parameter<std::vector<int64_t>>(node, "ids"), std::exception);
}
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_intersection_plugins");
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
