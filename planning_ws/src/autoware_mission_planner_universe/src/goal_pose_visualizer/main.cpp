#include "goal_pose_visualizer.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "goal_pose_visualizer");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  autoware::mission_planner_universe::GoalPoseVisualizer visualizer(node, private_node);
  ros::spin();
  return 0;
}
