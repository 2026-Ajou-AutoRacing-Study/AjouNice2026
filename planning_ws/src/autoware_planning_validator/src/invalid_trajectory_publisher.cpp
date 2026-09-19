#include <autoware/planning_validator/invalid_trajectory_publisher.hpp>

namespace autoware::planning_validator
{
InvalidTrajectoryPublisher::InvalidTrajectoryPublisher(ros::NodeHandle private_node)
: private_node_(std::move(private_node))
{
  trajectory_subscriber_ = private_node_.subscribe(
    "input/trajectory", 1, &InvalidTrajectoryPublisher::onTrajectory, this);
  trajectory_publisher_ = private_node_.advertise<autoware_planning_msgs::Trajectory>(
    "output/trajectory", 1);
  timer_ = private_node_.createTimer(ros::Duration(0.1), &InvalidTrajectoryPublisher::onTimer, this);
}

void InvalidTrajectoryPublisher::onTrajectory(
  const autoware_planning_msgs::TrajectoryConstPtr & message)
{
  current_trajectory_ = message;
  trajectory_subscriber_.shutdown();
}

void InvalidTrajectoryPublisher::onTimer(const ros::TimerEvent &)
{
  if (!current_trajectory_ || current_trajectory_->points.empty()) return;
  auto output = *current_trajectory_;
  auto & position = output.points.back().pose.position;
  constexpr double kAddedValue = 1.0e3;
  position.x += kAddedValue;
  position.y += kAddedValue;
  position.z += kAddedValue;
  trajectory_publisher_.publish(output);
}
}  // namespace autoware::planning_validator
