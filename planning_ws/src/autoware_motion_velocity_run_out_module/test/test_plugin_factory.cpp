#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>

#include <class_loader/class_loader.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <string>

TEST(RunOutPluginFactory, CreatesRegisteredRos1PluginFromCandidateDso)
{
  const std::string class_name = "autoware::motion_velocity_planner::RunOutModule";
  class_loader::ClassLoader loader(RUN_OUT_LIBRARY_PATH);
  const auto available =
    loader.getAvailableClasses<autoware::motion_velocity_planner::PluginModuleInterface>();
  EXPECT_NE(std::find(available.begin(), available.end(), class_name), available.end());
  const auto plugin =
    loader.createInstance<autoware::motion_velocity_planner::PluginModuleInterface>(class_name);
  ASSERT_TRUE(plugin);
  const auto required = plugin->getRequiredSubscriptions();
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_TRUE(required.lanelet_map);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "run_out_plugin_factory_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
