#include "autoware/planning_validator/rear_pointcloud_tf_core.hpp"

#include <gtest/gtest.h>

#include <geometry_msgs/TransformStamped.h>

namespace
{
TEST(RearPointcloudTfCore, ReturnsMapToCloudAffineFromBuffer)
{
  tf2_ros::Buffer buffer;
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "lidar";
  transform.header.stamp = ros::Time(10.0);
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = -3.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "rear_pointcloud_tf_core_oracle", true));
  std_msgs::Header header;
  header.frame_id = "lidar";
  header.stamp = transform.header.stamp;
  const auto affine = autoware::planning_validator::lookup_rear_pointcloud_to_map(buffer, header);
  ASSERT_TRUE(affine.has_value());
  EXPECT_FLOAT_EQ(affine->translation().x(), 2.0F);
  EXPECT_FLOAT_EQ(affine->translation().y(), -3.0F);
}

TEST(RearPointcloudTfCore, FailsClosedWhenTransformIsUnavailable)
{
  tf2_ros::Buffer buffer;
  buffer.setUsingDedicatedThread(true);
  std_msgs::Header header;
  header.frame_id = "unavailable_lidar";
  header.stamp = ros::Time(10.0);
  EXPECT_FALSE(autoware::planning_validator::lookup_rear_pointcloud_to_map(buffer, header).has_value());
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rear_pointcloud_tf_core_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
