#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_POINTCLOUD_TF_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_POINTCLOUD_TF_CORE_HPP_

#include <Eigen/Geometry>
#include <std_msgs/Header.h>
#include <tf2_ros/buffer.h>

#include <optional>

namespace autoware::planning_validator
{
// Source-shaped map<-pointcloud boundary from RearCollisionChecker::filter_pointcloud.
// A missing transform is not substituted with identity: callers must retain
// fail-closed checker status until this returns a value.
std::optional<Eigen::Affine3f> lookup_rear_pointcloud_to_map(
  tf2_ros::Buffer & buffer, const std_msgs::Header & pointcloud_header);
}  // namespace autoware::planning_validator

#endif
