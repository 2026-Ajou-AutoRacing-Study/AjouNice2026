#include "autoware/planning_validator/rear_pointcloud_core.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

namespace autoware::planning_validator
{
RearPointCloud::Ptr preprocess_rear_pointcloud(
  const sensor_msgs::PointCloud2 & input, const RearPointCloudCoreParameters & parameters,
  const double vehicle_height_m, const Eigen::Affine3f & source_to_map)
{
  auto output = boost::make_shared<RearPointCloud>();
  if (input.data.empty()) return output;

  pcl::fromROSMsg(input, *output);
  pcl::PassThrough<pcl::PointXYZ> x_filter;
  x_filter.setInputCloud(output);
  x_filter.setFilterFieldName("x");
  x_filter.setFilterLimits(parameters.crop_x_min, parameters.crop_x_max);
  x_filter.filter(*output);
  if (output->empty()) return output;

  pcl::PassThrough<pcl::PointXYZ> z_filter;
  z_filter.setInputCloud(output);
  z_filter.setFilterFieldName("z");
  z_filter.setFilterLimits(parameters.crop_z_min, vehicle_height_m + parameters.crop_z_max_offset);
  z_filter.filter(*output);
  if (output->empty()) return output;

  pcl::transformPointCloud(*output, *output, source_to_map);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(output);
  voxel_filter.setLeafSize(parameters.voxel_x, parameters.voxel_y, parameters.voxel_z);
  voxel_filter.filter(*output);
  return output;
}

RearPointCloud::Ptr cluster_rear_pointcloud(
  const RearPointCloud::Ptr & input, const RearPointCloudCoreParameters & parameters,
  const double ego_base_z)
{
  auto hull_points = boost::make_shared<RearPointCloud>();
  if (input->empty()) return hull_points;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
  extraction.setClusterTolerance(parameters.cluster_tolerance);
  extraction.setMinClusterSize(parameters.cluster_min_size);
  extraction.setMaxClusterSize(parameters.cluster_max_size);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(input);
  std::vector<pcl::PointIndices> cluster_indices;
  extraction.extract(cluster_indices);

  for (const auto & indices : cluster_indices) {
    auto cluster = boost::make_shared<RearPointCloud>();
    bool exceeds_height = false;
    for (const auto index : indices.indices) {
      const auto & point = (*input)[index];
      exceeds_height = exceeds_height ||
        (point.z - ego_base_z) > parameters.cluster_min_height;
      cluster->push_back(point);
    }
    if (!exceeds_height) continue;
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    RearPointCloud surface_hull;
    hull.reconstruct(surface_hull);
    *hull_points += surface_hull;
  }
  return hull_points;
}
}  // namespace autoware::planning_validator
