#include "autoware/planning_validator/rear_pointcloud_core.hpp"

#include <gtest/gtest.h>

#include <pcl_conversions/pcl_conversions.h>

namespace
{
sensor_msgs::PointCloud2 source_cloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({0.00F, 0.00F, 0.50F});
  cloud.push_back({0.05F, 0.00F, 0.60F});
  cloud.push_back({0.00F, 0.05F, 0.60F});
  cloud.push_back({0.05F, 0.05F, 0.60F});
  cloud.push_back({0.02F, 0.02F, 0.65F});
  cloud.push_back({50.0F, 0.00F, 0.50F});  // outside source x crop
  cloud.push_back({0.00F, 0.00F, 2.00F});  // outside source z crop
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  return message;
}
}  // namespace

TEST(RearPointCloudCore, AppliesSourceCropTransformVoxelAndClusterOrder)
{
  autoware::planning_validator::RearPointCloudCoreParameters parameters;
  parameters.voxel_x = 0.01;
  parameters.voxel_y = 0.01;
  parameters.voxel_z = 0.01;
  parameters.cluster_tolerance = 0.2;
  parameters.cluster_min_size = 3;
  Eigen::Affine3f source_to_map = Eigen::Affine3f::Identity();
  source_to_map.translate(Eigen::Vector3f(10.0F, 0.0F, 0.0F));
  const auto filtered = autoware::planning_validator::preprocess_rear_pointcloud(
    source_cloud(), parameters, 2.0, source_to_map);
  ASSERT_EQ(filtered->size(), 5U);
  EXPECT_NEAR(filtered->front().x, 10.0F, 1.0e-6F);
  const auto hulls = autoware::planning_validator::cluster_rear_pointcloud(filtered, parameters, 0.0);
  EXPECT_FALSE(hulls->empty());
}

TEST(RearPointCloudCore, RejectsClustersBelowSourceHeightThreshold)
{
  autoware::planning_validator::RearPointCloudCoreParameters parameters;
  parameters.cluster_tolerance = 0.2;
  parameters.cluster_min_size = 3;
  auto cloud = boost::make_shared<autoware::planning_validator::RearPointCloud>();
  cloud->push_back({0.0F, 0.0F, 0.05F});
  cloud->push_back({0.1F, 0.0F, 0.05F});
  cloud->push_back({0.0F, 0.1F, 0.05F});
  EXPECT_TRUE(autoware::planning_validator::cluster_rear_pointcloud(cloud, parameters, 0.0)->empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
