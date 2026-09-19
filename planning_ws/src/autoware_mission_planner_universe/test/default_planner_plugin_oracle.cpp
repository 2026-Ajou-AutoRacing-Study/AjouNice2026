#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>

#include <pluginlib/class_loader.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/topic.h>

#include <exception>
#include <cmath>
#include <iostream>
#include <set>
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
    // Optional real-map regression: test the plugin's emitted route, not a mock gateway.
    XmlRpc::XmlRpcValue cases;
    if (private_node.getParam("route_policy_cases", cases)) {
      const auto map = ros::topic::waitForMessage<autoware_map_msgs::LaneletMapBin>(
        "/map/vector_map", private_node, ros::Duration(30));
      if (!map) throw std::runtime_error("map input timed out");
      planner->initialize(&private_node, map);
      std::vector<int> excluded_ids;
      private_node.getParam("route_section_excluded_lanelet_ids", excluded_ids);
      const std::set<int> excluded(excluded_ids.begin(), excluded_ids.end());
      for (int i = 0; i < cases.size(); ++i) {
        autoware::mission_planner_universe::PlannerPlugin::RoutePoints points;
        auto & coordinates = cases[i]["points"];
        for (int j = 0; j < coordinates.size(); ++j) {
          geometry_msgs::Pose pose;
          pose.position.x = static_cast<double>(coordinates[j][0]);
          pose.position.y = static_cast<double>(coordinates[j][1]);
          pose.position.z = static_cast<double>(coordinates[j][2]);
          const double yaw = static_cast<double>(coordinates[j][3]);
          pose.orientation.z = std::sin(yaw / 2);
          pose.orientation.w = std::cos(yaw / 2);
          points.push_back(pose);
        }
        planner->clearRoute();
        const auto route = planner->plan(points);
        if (route.segments.empty()) throw std::runtime_error("empty policy test route");
        std::set<int> present;
        for (const auto & section : route.segments) {
          std::cout << "section " << section.preferred_primitive.id << ":";
          for (const auto & primitive : section.primitives) {
            if (excluded.count(primitive.id)) throw std::runtime_error("excluded lane emitted");
            present.insert(primitive.id);
            std::cout << " " << primitive.id;
          }
          std::cout << std::endl;
        }
        if (!present.count(201438)) throw std::runtime_error("required merge lane 201438 missing");
        std::cout << "route_policy_case=" << i << " PASS sections=" << route.segments.size()
                  << std::endl;
      }
    }
    std::cout << "default_planner_plugin_oracle=PASS" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "plugin oracle error: " << error.what() << std::endl;
    return 1;
  }
}
