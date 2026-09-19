#include <aw149_surround_obstacle_checker/surround_obstacle_checker_ros1_node.hpp>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_surround_obstacle_checker");
  try {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    aw149::surround_obstacle_checker::SurroundObstacleCheckerRos1Node checker(node, private_node);
    ros::spin();
  } catch (const std::exception & exception) {
    ROS_FATAL("surround_obstacle_checker fails closed: %s", exception.what());
    return 2;
  }
  return 0;
}
