#include "autoware_map_loader/pointcloud_map_loader.hpp"
int main(int argc, char ** argv) { ros::init(argc, argv, "pointcloud_map_loader"); ros::NodeHandle nh; ros::NodeHandle pnh("~"); autoware::map_loader::PointCloudMapLoader loader(nh, pnh); ros::spin(); return 0; }
