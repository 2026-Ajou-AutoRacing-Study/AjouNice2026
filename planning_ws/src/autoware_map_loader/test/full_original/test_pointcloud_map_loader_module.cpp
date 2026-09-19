#include "native_test_support.hpp"
// Copyright 2023 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pointcloud_map_loader/pointcloud_map_loader_module.hpp"
#include "pointcloud_map_loader/utils.hpp"

#include "native_test_support.hpp"

#include <sensor_msgs/PointCloud2.h>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

class TestPointcloudMapLoaderModule : public ::testing::Test
{
protected:
  std::shared_ptr<autoware::map_loader::map_native::Node> node;
  std::string temp_pcd_path;

  void SetUp() override
  {
    node = std::make_shared<autoware::map_loader::map_native::Node>(ros::NodeHandle("/test_pointcloud_map_loader_module"));

    // Create a temporary PCD file with dummy point cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = static_cast<float>(i);
      cloud.points[i].y = static_cast<float>(i * 2);
      cloud.points[i].z = static_cast<float>(i * 3);
    }

    temp_pcd_path = owned_map_test_file("test_pointcloud_map_loader_module.pcd");
    pcl::io::savePCDFileASCII(temp_pcd_path, cloud);
  }


};

TEST_F(TestPointcloudMapLoaderModule, LoadPCDFilesNoDownsampleTest)
{
  using namespace std::literals::chrono_literals;

  // Prepare PCD paths
  std::vector<std::string> pcd_paths = {temp_pcd_path};

  // Create PointcloudMapLoaderModule instance
  autoware::map_loader::PointcloudMapLoaderModule loader(
    node.get(), pcd_paths, "pointcloud_map_no_downsample", false);

  // Create a subscriber to the published pointcloud topic
  auto pointcloud_received = std::make_shared<bool>(false);
  auto pointcloud_msg = std::make_shared<sensor_msgs::PointCloud2>();


  auto pointcloud_sub = node->subscribe<sensor_msgs::PointCloud2>(
    "pointcloud_map_no_downsample", 10,
    [pointcloud_received, pointcloud_msg](const sensor_msgs::PointCloud2::ConstPtr msg) {
      *pointcloud_received = true;
      *pointcloud_msg = *msg;
    });

  // Spin until pointcloud is received or timeout occurs
  auto start_time = node->now();
  while (!*pointcloud_received && (node->now() - start_time).toSec() < 3) {
    ros::spinOnce();ros::WallDuration(0.01).sleep();
  }

  // Check if the point cloud is received and the content is as expected
  ASSERT_TRUE(*pointcloud_received);

  // Convert the received point cloud to pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ> received_cloud;
  pcl::fromROSMsg(*pointcloud_msg, received_cloud);

  // Check if the received point cloud matches the content of the temporary PCD file
  ASSERT_EQ(static_cast<int>(received_cloud.width), 5);
  ASSERT_EQ(static_cast<int>(received_cloud.height), 1);

  for (size_t i = 0; i < received_cloud.points.size(); ++i) {
    EXPECT_FLOAT_EQ(received_cloud.points[i].x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(received_cloud.points[i].y, static_cast<float>(i * 2));
    EXPECT_FLOAT_EQ(received_cloud.points[i].z, static_cast<float>(i * 3));
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"original_map_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
