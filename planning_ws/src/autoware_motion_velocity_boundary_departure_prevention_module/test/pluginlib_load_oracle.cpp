#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

#include <exception>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bdp_pluginlib_load_oracle");
  constexpr auto kClass =
    "autoware::motion_velocity_planner::experimental::BoundaryDeparturePreventionModule";
  try {
    pluginlib::ClassLoader<autoware::motion_velocity_planner::PluginModuleInterface> loader(
      "autoware_motion_velocity_planner", "autoware::motion_velocity_planner::PluginModuleInterface");
    if (!loader.isClassAvailable(kClass)) {
      std::cerr << "BDP_PLUGINLIB_CLASS_UNAVAILABLE" << std::endl;
      return 2;
    }
    std::cout << "BDP_PLUGINLIB_RESOLVED_PATH " << loader.getClassLibraryPath(kClass) << std::endl;
    const auto module = loader.createInstance(kClass);
    ros::NodeHandle node;
    module->init(node, "boundary_departure_prevention");
    if (module->getModuleName() != "boundary_departure_prevention") {
      std::cerr << "BDP_PLUGINLIB_MODULE_NAME_MISMATCH" << std::endl;
      return 3;
    }
    std::cout << "BDP_PLUGINLIB_LOAD_AND_TRANSPORT_INIT_PASS" << std::endl;
  } catch (const std::exception & error) {
    std::cerr << "BDP_PLUGINLIB_LOAD_EXCEPTION " << error.what() << std::endl;
    return 4;
  }
  return 0;
}
