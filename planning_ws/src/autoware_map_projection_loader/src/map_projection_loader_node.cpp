#include "autoware_map_projection_loader/map_projection_loader.hpp"

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "map_projection_loader");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string yaml_filename;
  std::string lanelet2_map_filename;
  pnh.param<std::string>("map_projector_info_path", yaml_filename, "");
  pnh.param<std::string>("lanelet2_map_path", lanelet2_map_filename, "");
  try {
    const auto msg = autoware::map_projection_loader::load_map_projector_info(
      yaml_filename, lanelet2_map_filename);
    const auto publisher = nh.advertise<autoware_map_msgs::MapProjectorInfo>(
      "/map/map_projector_info", 1, true);
    publisher.publish(msg);
    ros::spin();
  } catch (const std::exception & error) {
    ROS_FATAL_STREAM("map_projection_loader: " << error.what());
    return 1;
  }
  return 0;
}
