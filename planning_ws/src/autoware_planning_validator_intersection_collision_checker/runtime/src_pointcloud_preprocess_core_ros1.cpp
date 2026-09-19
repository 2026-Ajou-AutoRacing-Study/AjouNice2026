#include <autoware/planning_validator_intersection_collision_checker/pointcloud_preprocess_core_ros1.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
std::optional<sensor_msgs::PointCloud2> preprocess_pointcloud(
  const sensor_msgs::PointCloud2 & input, tf2_ros::Buffer & tf_buffer,
  const double vehicle_height, const PointCloudPreprocessParameters & p)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(input, cloud);
  pcl::PassThrough<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud.makeShared());
  crop.setFilterFieldName("x");
  crop.setFilterLimits(-p.detection_range, p.detection_range);
  crop.filter(cloud);
  crop.setInputCloud(cloud.makeShared());
  crop.setFilterFieldName("z");
  crop.setFilterLimits(p.min_height, vehicle_height + p.height_buffer);
  crop.filter(cloud);
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform("map", input.header.frame_id, input.header.stamp, ros::Duration(0.1));
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
  pcl::transformPointCloud(cloud, cloud, tf2::transformToEigen(transform.transform).cast<float>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud.makeShared());
  voxel.setLeafSize(p.voxel_x, p.voxel_y, p.voxel_z);
  voxel.setMinimumPointsNumberPerVoxel(p.voxel_min_size);
  voxel.filter(cloud);
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header = input.header;
  output.header.frame_id = "map";
  return output;
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
