#include "autoware/planning_validator_intersection_collision_checker/pointcloud_cluster_core_ros1.hpp"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
pcl::PointCloud<pcl::PointXYZ> cluster_points_above_height(
  const pcl::PointCloud<pcl::PointXYZ> & input, const double ego_base_z,
  const ClusterParameters & parameters)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  if (input.empty()) return output;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input.makeShared());
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
  extraction.setClusterTolerance(parameters.tolerance);
  extraction.setMinClusterSize(parameters.min_size);
  extraction.setMaxClusterSize(parameters.max_size);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(input.makeShared());
  std::vector<pcl::PointIndices> indices;
  extraction.extract(indices);
  for (const auto & cluster_indices : indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    bool above_height = false;
    for (const auto index : cluster_indices.indices) {
      const auto & point = input[static_cast<size_t>(index)];
      above_height |= point.z - ego_base_z > parameters.min_height;
      cluster->push_back(point);
    }
    if (!above_height) continue;
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    pcl::PointCloud<pcl::PointXYZ> surface_hull;
    hull.reconstruct(surface_hull);
    output += surface_hull;
  }
  return output;
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
