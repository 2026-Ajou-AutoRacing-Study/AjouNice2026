#include <autoware_freespace_planner/freespace_planner_node.hpp>
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "freespace_planner");
  ros::NodeHandle node, private_node("~");
  try {
    autoware::freespace_planner::FreespacePlannerNode planner(node, private_node);
    ros::spin();
  } catch (const std::exception & error) {
    ROS_FATAL("Freespace planner failed: %s", error.what());
    return 1;
  }
  return 0;
}
