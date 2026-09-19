#include <autoware/costmap_generator/costmap_generator.hpp>
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "costmap_generator");
  ros::NodeHandle private_node("~");
  try {
    autoware::costmap_generator::CostmapGenerator generator(private_node);
    ros::spin();
  } catch (const std::exception & error) {
    ROS_FATAL("Costmap generator failed: %s", error.what());
    return 1;
  }
  return 0;
}
