#include "autoware/planning_validator/rear_pointcloud_tf_core.hpp"

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>

namespace autoware::planning_validator
{
std::optional<Eigen::Affine3f> lookup_rear_pointcloud_to_map(
  tf2_ros::Buffer & buffer, const std_msgs::Header & pointcloud_header)
{
  try {
    const auto transform = buffer.lookupTransform(
      "map", pointcloud_header.frame_id, pointcloud_header.stamp, ros::Duration(0.1));
    return tf2::transformToEigen(transform.transform).cast<float>();
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}
}  // namespace autoware::planning_validator
