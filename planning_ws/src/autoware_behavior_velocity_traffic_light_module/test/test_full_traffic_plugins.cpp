#include <autoware/behavior_velocity_traffic_light_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
#include <stdexcept>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void check(const std::string & base, const std::string & prefix,
                                     double yellow_deceleration=8., bool obsolete_timed=false)
{
  ros::NodeHandle node("/full_traffic_light_test");
  node.deleteParam("/full_traffic_light_test");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_traffic_light_module)/config/ros1/traffic_light.yaml\" /full_traffic_light_test"), 0);
  node.setParam("traffic_light/signal_compliance_yellow_deceleration_mps2",yellow_deceleration);
  if (obsolete_timed) {
    // Historic snapshots may contain these retired values without a guard.
    // They must neither fail startup nor reactivate the old timing policy.
    node.setParam("traffic_light/signal_compliance_enable_timed_yellow",true);
    node.setParam("traffic_light/signal_compliance_min_yellow_sec",0.);
    node.setParam("traffic_light/signal_compliance_yellow_time_margin_sec",-100.);
  }
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
TEST(FullTrafficPlugins, RetiredTimedFieldsDoNotPreventEitherPluginStarting) {
  check<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface",
    "autoware::behavior_velocity_planner::",8.,true);
  check<bvp::experimental::PluginInterface>("autoware::behavior_velocity_planner::experimental::PluginInterface",
    "autoware::behavior_velocity_planner::experimental::",8.,true);
}
TEST(FullTrafficPlugins, InvalidYellowDecelerationRejectedByBothManagers) {
  for (const double invalid : {0.,-8.}) {
    EXPECT_THROW(check<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface",
      "autoware::behavior_velocity_planner::",invalid),std::invalid_argument);
    EXPECT_THROW(check<bvp::experimental::PluginInterface>("autoware::behavior_velocity_planner::experimental::PluginInterface",
      "autoware::behavior_velocity_planner::experimental::",invalid),std::invalid_argument);
  }
}
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_traffic_plugins");
  ros::NodeHandle owner;
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
