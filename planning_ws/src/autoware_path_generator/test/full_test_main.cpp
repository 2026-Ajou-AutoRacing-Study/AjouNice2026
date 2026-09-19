#include <ros/ros.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <utility>
int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_path_generator_test"); ros::NodeHandle owner;
  if (std::system(
        "rosparam load \"$(rospack find autoware_path_generator)/config/ros1/"
        "path_generator.yaml\" /full_path_generator_test") != 0) {
    return 1;
  }
  ros::NodeHandle node("/full_path_generator_test");
  for (const auto & entry : {
         std::pair{"wheel_radius", 0.383}, {"wheel_width", 0.235}, {"wheel_base", 2.79},
         {"wheel_tread", 1.64}, {"front_overhang", 1.0}, {"rear_overhang", 1.1},
         {"left_overhang", 0.128}, {"right_overhang", 0.128}, {"vehicle_height", 2.5},
         {"max_steer_angle", 0.70}}) {
    node.setParam(entry.first, entry.second);
  }
  testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS();
}
