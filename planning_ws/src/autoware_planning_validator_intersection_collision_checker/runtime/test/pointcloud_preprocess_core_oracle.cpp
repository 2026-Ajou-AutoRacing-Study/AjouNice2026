#include <autoware/planning_validator_intersection_collision_checker/pointcloud_preprocess_core_ros1.hpp>

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace icc = autoware::planning_validator::intersection_collision_checker_ros1;

namespace
{
sensor_msgs::PointCloud2 cloud()
{
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "lidar";
  msg.header.stamp = ros::Time(12.0);
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3U);
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (const auto & point : {std::array<float, 3>{1.0F, 0.0F, 1.0F}, {100.0F, 0.0F, 1.0F}, {2.0F, 0.0F, 0.1F}}) {
    *x = point[0]; *y = point[1]; *z = point[2]; ++x; ++y; ++z;
  }
  return msg;
}
icc::PointCloudPreprocessParameters parameters() { return {10.0, 0.5, 0.5, 0.2, 0.2, 0.2, 1}; }
}

TEST(PointCloudPreprocessCoreRos1, MissingTransformFailsClosed)
{
  tf2_ros::Buffer buffer;
  EXPECT_FALSE(icc::preprocess_pointcloud(cloud(), buffer, 2.0, parameters()).has_value());
}

TEST(PointCloudPreprocessCoreRos1, CropsTransformsAndVoxels)
{
  tf2_ros::Buffer buffer;
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time(12.0);
  transform.header.frame_id = "map";
  transform.child_frame_id = "lidar";
  transform.transform.translation.x = 10.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "icc-r004-oracle", true));
  const auto result = icc::preprocess_pointcloud(cloud(), buffer, 2.0, parameters());
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.frame_id, "map");
  EXPECT_EQ(result->width, 1U);
  sensor_msgs::PointCloud2ConstIterator<float> x(*result, "x");
  EXPECT_NEAR(*x, 11.0F, 1e-5F);
}
