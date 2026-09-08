#include "invalid_trajectory_publisher.hpp"
int main(int argc,char **argv) {
  ros::init(argc,argv,"invalid_trajectory_publisher");
  autoware::planning_validator::full_source::InvalidTrajectoryPublisherNode node{ros::NodeHandle("~")};
  ros::spin();
  return 0;
}
