#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>

#include <pluginlib/class_loader.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <exception>
#include <iostream>
#include <vector>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "default_planner_plugin_oracle");
  ros::NodeHandle private_node("~");
  private_node.setParam("wheel_radius", 0.3);
  private_node.setParam("wheel_width", 0.2);
  private_node.setParam("wheel_base", 2.7);
  private_node.setParam("wheel_tread", 1.5);
  private_node.setParam("front_overhang", 0.9);
  private_node.setParam("rear_overhang", 1.0);
  private_node.setParam("left_overhang", 0.2);
  private_node.setParam("right_overhang", 0.2);
  private_node.setParam("vehicle_height", 1.6);
  private_node.setParam("max_steer_angle", 0.6);

  try {
    const auto package_path = ros::package::getPath("autoware_mission_planner_universe");
    if (package_path.empty()) {
      std::cerr << "plugin package is not discoverable" << std::endl;
      return 3;
    }
    pluginlib::ClassLoader<autoware::mission_planner_universe::PlannerPlugin> loader(
      "autoware_mission_planner_universe", "autoware::mission_planner_universe::PlannerPlugin",
      "plugin", {package_path + "/plugins/plugin_description.xml"});
    auto planner = loader.createInstance(
      "autoware::mission_planner_universe::lanelet2::DefaultPlanner");
    planner->initialize(&private_node);
    if (planner->ready()) {
      std::cerr << "DefaultPlanner reported ready before map input" << std::endl;
      return 2;
    }
    std::cout << "default_planner_plugin_oracle=PASS" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "plugin oracle error: " << error.what() << std::endl;
    return 1;
  }
}
