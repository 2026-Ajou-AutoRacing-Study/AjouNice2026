#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_POINTCLOUD_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_POINTCLOUD_CORE_HPP_

#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace autoware::planning_validator
{
struct RearPointCloudCoreParameters
{
  double crop_x_min{-100.0};
  double crop_x_max{30.0};
  double crop_z_min{0.3};
  double crop_z_max_offset{-1.0};
  double voxel_x{0.1};
  double voxel_y{0.1};
  double voxel_z{0.5};
  double cluster_tolerance{0.15};
  double cluster_min_height{0.1};
  int cluster_min_size{5};
  int cluster_max_size{10000};
};

using RearPointCloud = pcl::PointCloud<pcl::PointXYZ>;

// Source-faithful preprocessing order for the rear collision checker:
// crop x, crop z using vehicle height, transform to map, then voxelize.
RearPointCloud::Ptr preprocess_rear_pointcloud(
  const sensor_msgs::PointCloud2 & input, const RearPointCloudCoreParameters & parameters,
  double vehicle_height_m, const Eigen::Affine3f & source_to_map);

// Keeps only 2D convex-hull points from clusters which exceed the source
// minimum height relative to the ego base z.
RearPointCloud::Ptr cluster_rear_pointcloud(
  const RearPointCloud::Ptr & input, const RearPointCloudCoreParameters & parameters,
  double ego_base_z);
}  // namespace autoware::planning_validator

#endif
