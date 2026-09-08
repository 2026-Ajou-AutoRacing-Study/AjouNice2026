#include "autoware/map_loader/lanelet2_map_loader_node.hpp"
int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_lanelet2_map_loader");
  try {ros::NodeHandle node("~"); autoware::map_loader::Lanelet2MapLoaderNode loader(node);ros::spin();}
  catch(const std::exception & error) {ROS_FATAL("%s",error.what());return 1;}return 0;
}
