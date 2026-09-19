#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "r025_obstacle_velocity_limiter_init_probe");
  const std::string node_namespace = argc > 1 ? argv[1] : "/r025_positive";
  try {
    ros::NodeHandle node(node_namespace);
    pluginlib::ClassLoader<autoware::motion_velocity_planner::PluginModuleInterface> loader(
      "autoware_motion_velocity_planner",
      "autoware::motion_velocity_planner::PluginModuleInterface");
    const auto plugin = loader.createInstance(
      "autoware::motion_velocity_planner::ObstacleVelocityLimiterModule");
    plugin->init(node, "obstacle_velocity_limiter");
    std::cout << "P3_R025_OBSTACLE_VELOCITY_LIMITER_INIT_PASS" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
