#include "pointcloud_map_loader/pointcloud_map_loader_node.hpp"
int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_pointcloud_map_loader");
  try {ros::NodeHandle node("~"); autoware::map_loader::PointCloudMapLoaderNode loader(node);ros::spin();}
  catch(const std::exception & error) {ROS_FATAL("%s",error.what());return 1;}return 0;
}
