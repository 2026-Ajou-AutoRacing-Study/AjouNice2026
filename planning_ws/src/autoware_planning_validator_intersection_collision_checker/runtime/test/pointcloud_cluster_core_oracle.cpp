#include "autoware/planning_validator_intersection_collision_checker/pointcloud_cluster_core_ros1.hpp"

#include <gtest/gtest.h>

namespace icc = autoware::planning_validator::intersection_collision_checker_ros1;

TEST(PointcloudClusterCore, RetainsOnlyHeightQualifiedEuclideanClusterHull)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  // A valid, high cluster near x=0.
  input.push_back({0.0F, 0.0F, 1.0F});
  input.push_back({0.1F, 0.0F, 1.0F});
  input.push_back({0.0F, 0.1F, 1.0F});
  // A same-size low cluster far away; source policy must reject it.
  input.push_back({5.0F, 0.0F, 0.1F});
  input.push_back({5.1F, 0.0F, 0.1F});
  input.push_back({5.0F, 0.1F, 0.1F});
  const auto output = icc::cluster_points_above_height(input, 0.0, {0.3, 0.5, 3, 100});
  ASSERT_FALSE(output.empty());
  for (const auto & point : output) {
    EXPECT_LT(point.x, 1.0F);
    EXPECT_GT(point.z, 0.5F);
  }
}

TEST(PointcloudClusterCore, RejectsClustersOutsideConfiguredSize)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  input.push_back({0.0F, 0.0F, 1.0F});
  input.push_back({0.1F, 0.0F, 1.0F});
  const auto output = icc::cluster_points_above_height(input, 0.0, {0.3, 0.5, 3, 100});
  EXPECT_TRUE(output.empty());
}
