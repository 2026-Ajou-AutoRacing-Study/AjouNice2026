#include <ros/ros.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>
int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_motion_obstacle_test");ros::NodeHandle owner;
  const std::vector<std::string> namespaces{
    "/full_motion_obstacle_test", "/full_motion_obstacle_outside_test"};
  const std::vector<std::string> configs{
    "$(rospack find autoware_motion_velocity_planner)/config/ros1/obstacle_stop_default.yaml",
    "$(rospack find autoware_motion_velocity_planner)/config/ros1/motion_velocity_planner.yaml",
    "$(rospack find autoware_velocity_smoother)/config/ros1/default_common.yaml",
    "$(rospack find autoware_velocity_smoother)/config/ros1/default_velocity_smoother.yaml",
    "$(rospack find autoware_velocity_smoother)/config/ros1/Analytical.yaml",
    "$(rospack find autoware_launch)/config/ros1/planning/scenario_planning/common/nearest_search.param.yaml"};
  for (const auto & ns : namespaces) {
    for (const auto & config : configs) {
      if (std::system(("rosparam load \""+config+"\" "+ns).c_str()) != 0) return 1;
    }
    ros::NodeHandle node(ns);
    for (const auto & entry : {
           std::pair{"wheel_radius", 0.383}, {"wheel_width", 0.235}, {"wheel_base", 2.79},
           {"wheel_tread", 1.64}, {"front_overhang", 1.0}, {"rear_overhang", 1.1},
           {"left_overhang", 0.128}, {"right_overhang", 0.128}, {"vehicle_height", 2.5},
           {"max_steer_angle", 0.70}}) {
      node.setParam(entry.first, entry.second);
    }
  }
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
