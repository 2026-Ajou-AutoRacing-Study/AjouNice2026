#ifndef AUTOWARE_PLANNING_VALIDATOR__INVALID_TRAJECTORY_PUBLISHER_HPP_
#define AUTOWARE_PLANNING_VALIDATOR__INVALID_TRAJECTORY_PUBLISHER_HPP_

#include <autoware_planning_msgs/Trajectory.h>
#include <ros/ros.h>

namespace autoware::planning_validator
{
class InvalidTrajectoryPublisher
{
public:
  explicit InvalidTrajectoryPublisher(ros::NodeHandle private_node);

private:
  void onTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & message);
  void onTimer(const ros::TimerEvent &);

  ros::NodeHandle private_node_;
  ros::Subscriber trajectory_subscriber_;
  ros::Publisher trajectory_publisher_;
  ros::Timer timer_;
  autoware_planning_msgs::TrajectoryConstPtr current_trajectory_;
};
}  // namespace autoware::planning_validator

#endif
