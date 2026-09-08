#pragma once

#include <autoware_utils_pcl/transforms.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace autoware::behavior_velocity_planner
{
inline pcl::PointCloud<pcl::PointXYZ>::Ptr transformNoGroundPointCloud(
  const sensor_msgs::PointCloud2 & message, const geometry_msgs::TransformStamped & transform)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  pcl::fromROSMsg(message, input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  if (input.empty()) {
    return output;
  }

  const Eigen::Translation3f translation(
    transform.transform.translation.x, transform.transform.translation.y,
    transform.transform.translation.z);
  const Eigen::Quaternionf rotation(
    transform.transform.rotation.w, transform.transform.rotation.x,
    transform.transform.rotation.y, transform.transform.rotation.z);
  autoware_utils_pcl::transform_pointcloud(input, *output, translation * rotation);
  return output;
}
}  // namespace autoware::behavior_velocity_planner
