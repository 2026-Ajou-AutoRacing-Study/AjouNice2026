#ifndef AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_POINTCLOUD_PREPROCESS_CORE_ROS1_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_POINTCLOUD_PREPROCESS_CORE_ROS1_HPP_

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

#include <optional>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
struct PointCloudPreprocessParameters
{
  double detection_range;
  double min_height;
  double height_buffer;
  double voxel_x;
  double voxel_y;
  double voxel_z;
  int voxel_min_size;
};

// Source-equivalent order: x crop, z crop, map-from-cloud TF, then voxel.
// Missing TF is deliberately a null result; the caller must invalidate the
// checker instead of treating an untransformed cloud as map data.
std::optional<sensor_msgs::PointCloud2> preprocess_pointcloud(
  const sensor_msgs::PointCloud2 & input, tf2_ros::Buffer & tf_buffer,
  const double vehicle_height, const PointCloudPreprocessParameters & parameters);
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1

#endif
