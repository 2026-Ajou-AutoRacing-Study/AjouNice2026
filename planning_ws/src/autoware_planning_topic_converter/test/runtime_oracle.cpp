#include <autoware/planning_topic_converter/path_to_trajectory.hpp>

#include <cstdlib>
#include <iostream>

namespace
{
bool received = false;
autoware_planning_msgs::Trajectory received_trajectory;

void on_trajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  received_trajectory = *msg;
  received = true;
}

bool require(bool condition, const char * description)
{
  if (!condition) {
    std::cerr << "runtime oracle failed: " << description << std::endl;
  }
  return condition;
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_topic_converter_runtime_oracle");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  private_node.setParam("input_topic", "oracle_input_path");
  private_node.setParam("output_topic", "oracle_output_trajectory");
  autoware::planning_topic_converter::PathToTrajectory converter(node, private_node);
  ros::Publisher input = node.advertise<autoware_planning_msgs::Path>("oracle_input_path", 1);
  ros::Subscriber output = node.subscribe("oracle_output_trajectory", 1, on_trajectory);

  autoware_planning_msgs::Path path;
  path.header.frame_id = "map";
  autoware_planning_msgs::PathPoint first;
  first.pose.position.x = 1.0;
  first.pose.orientation.w = 1.0;
  first.longitudinal_velocity_mps = 2.5F;
  autoware_planning_msgs::PathPoint second;
  second.pose.position.x = 3.0;
  second.pose.orientation.w = 1.0;
  second.lateral_velocity_mps = 0.4F;
  second.heading_rate_rps = 0.2F;
  path.points.push_back(first);
  path.points.push_back(second);

  for (int i = 0; i < 100 && (input.getNumSubscribers() == 0 || output.getNumPublishers() == 0); ++i) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (!require(input.getNumSubscribers() == 1, "input subscription") ||
      !require(output.getNumPublishers() == 1, "output publisher")) {
    return EXIT_FAILURE;
  }
  input.publish(path);
  for (int i = 0; i < 100 && !received; ++i) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (!require(received, "trajectory receipt") ||
      !require(received_trajectory.header.frame_id == "map", "header propagation") ||
      !require(received_trajectory.points.size() == 2, "point count") ||
      !require(received_trajectory.points[0].pose.position.x == 1.0, "first pose") ||
      !require(received_trajectory.points[0].longitudinal_velocity_mps == 2.5F, "velocity") ||
      !require(received_trajectory.points[1].heading_rate_rps == 0.2F, "heading rate")) {
    return EXIT_FAILURE;
  }
  std::cout << "planning topic converter runtime oracle passed" << std::endl;
  return EXIT_SUCCESS;
}
