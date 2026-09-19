#include <autoware/path_generator/node.hpp>
int main(int argc,char ** argv) {
  ros::init(argc,argv,"path_generator");
  ros::NodeHandle node("~");
  autoware::path_generator::PathGenerator generator(node);
  ros::spin();
  return 0;
}
