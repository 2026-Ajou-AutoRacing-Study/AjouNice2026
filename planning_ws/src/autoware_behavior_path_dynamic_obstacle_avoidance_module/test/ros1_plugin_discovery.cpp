#include <autoware/behavior_path_dynamic_obstacle_avoidance_module/manager.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

#include <exception>
#include <iostream>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_doa_plugin_discovery", ros::init_options::AnonymousName);
  try {
    pluginlib::ClassLoader<autoware::behavior_path_planner::SceneModuleManagerInterface> loader(
      "autoware_behavior_path_planner", "autoware::behavior_path_planner::SceneModuleManagerInterface");
    const auto instance = loader.createClassInstance(
      "autoware::behavior_path_planner::DynamicObstacleAvoidanceModuleManager");
    if (!instance) return 2;
    std::cout << "AW149_DOA_PLUGIN_DISCOVERY_PASS" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "AW149_DOA_PLUGIN_DISCOVERY_FAIL " << error.what() << std::endl;
    return 1;
  }
}
