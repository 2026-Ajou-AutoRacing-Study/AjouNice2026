#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/ros1_time.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>

#include <cmath>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <vector>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "autoware_motion_utils_runtime_oracle");
  ros::NodeHandle node;
  using autoware::motion_utils::duration_from_seconds;
  using autoware::motion_utils::duration_to_seconds;

  const auto duration = duration_from_seconds(1.25);
  if (std::abs(duration_to_seconds(duration) - 1.25) > 1e-9) {
    return 10;
  }

  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points.resize(2);
  trajectory.points[0].pose.orientation.w = 1.0;
  trajectory.points[1].pose.orientation.w = 1.0;
  trajectory.points[1].pose.position.x = 1.0;
  trajectory.points[0].longitudinal_velocity_mps = 4.0F;
  trajectory.points[1].longitudinal_velocity_mps = 0.0F;
  trajectory.points[0].time_from_start = duration_from_seconds(0.0);
  trajectory.points[1].time_from_start = duration_from_seconds(2.0);

  const auto resampled = autoware::motion_utils::resampleTrajectory(
    trajectory, std::vector<double>{0.0, 0.5, 1.0}, false, true, false);
  if (resampled.points.size() != 3U) {
    return 11;
  }
  if (std::abs(resampled.points[1].pose.position.x - 0.5) > 1e-6) {
    return 12;
  }
  if (std::abs(duration_to_seconds(resampled.points[1].time_from_start) - 1.0) > 1e-6) {
    return 13;
  }
  const auto stop_index = autoware::motion_utils::searchZeroVelocityIndex(resampled.points);
  if (!stop_index || stop_index.value() != 2U) {
    return 14;
  }

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  const auto markers = autoware::motion_utils::createStopVirtualWallMarker(
    pose, "runtime_oracle", ros::Time(3.0), 7);
  if (markers.markers.size() != 2U || markers.markers[0].header.stamp != ros::Time(3.0)) {
    return 15;
  }

  autoware::motion_utils::VehicleArrivalChecker arrival_checker(&node);
  auto odometry_publisher = node.advertise<nav_msgs::Odometry>("/localization/kinematic_state", 1, true);
  auto trajectory_publisher = node.advertise<autoware_planning_msgs::Trajectory>("/planning/trajectory", 1, true);
  const auto subscriber_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (
    ros::ok() && ros::WallTime::now() < subscriber_deadline &&
    (odometry_publisher.getNumSubscribers() == 0 || trajectory_publisher.getNumSubscribers() == 0)) {
    ros::spinOnce();
    ros::WallDuration(0.02).sleep();
  }
  if (odometry_publisher.getNumSubscribers() == 0 || trajectory_publisher.getNumSubscribers() == 0) {
    std::cerr << "VehicleArrivalChecker subscriptions were not established" << std::endl;
    return 16;
  }
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp = ros::Time::now();
  odometry.pose.pose.position.x = 1.0;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 0.0;
  trajectory_publisher.publish(trajectory);
  odometry_publisher.publish(odometry);
  const auto arrival_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::ok() && ros::WallTime::now() < arrival_deadline && !arrival_checker.isVehicleStoppedAtStopPoint(0.0)) {
    ros::spinOnce();
    ros::WallDuration(0.02).sleep();
  }
  if (!arrival_checker.isVehicleStoppedAtStopPoint(0.0)) {
    std::cerr << "VehicleArrivalChecker did not recognize a subscribed stopped vehicle at trajectory stop" << std::endl;
    return 17;
  }

  std::cout << "motion_utils runtime oracle passed with ROS subscriber delivery" << std::endl;
  return 0;
}
