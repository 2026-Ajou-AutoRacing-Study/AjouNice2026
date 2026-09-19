#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <iostream>

int main()
{
  try {
    pluginlib::ClassLoader<autoware::motion_velocity_planner::PluginModuleInterface> loader(
      "autoware_motion_velocity_planner",
      "autoware::motion_velocity_planner::PluginModuleInterface");
    const auto plugin = loader.createInstance(
      "autoware::motion_velocity_planner::ObstacleVelocityLimiterModule");
    if (!plugin) return 2;
    std::cout << "P3_R025_OBSTACLE_VELOCITY_LIMITER_PLUGINLIB_PASS" << std::endl;
    return 0;
  } catch (const pluginlib::PluginlibException & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
