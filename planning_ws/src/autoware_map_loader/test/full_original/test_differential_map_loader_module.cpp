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

#include "pointcloud_map_loader/differential_map_loader_module.hpp"

#include "native_test_support.hpp"

#include <autoware_map_msgs/GetDifferentialPointCloudMap.h>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include <map>
#include <memory>
#include <string>

using autoware::map_loader::DifferentialMapLoaderModule;
using autoware_map_msgs::GetDifferentialPointCloudMap;

class TestDifferentialMapLoaderModule : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node
    node_ = std::make_shared<autoware::map_loader::map_native::Node>(ros::NodeHandle("/test_differential_map_loader_module"));

    // Generate a sample dummy pointcloud and save it to a file
    pcl::PointCloud<pcl::PointXYZ> dummy_cloud;
    dummy_cloud.width = 3;
    dummy_cloud.height = 1;
    dummy_cloud.points.resize(dummy_cloud.width * dummy_cloud.height);
    dummy_cloud.points[0] = pcl::PointXYZ(-1.0, -1.0, -1.0);
    dummy_cloud.points[1] = pcl::PointXYZ(0.0, 0.0, 0.0);
    dummy_cloud.points[2] = pcl::PointXYZ(1.0, 1.0, 1.0);
    pcl::io::savePCDFileASCII(owned_map_test_file("dummy.pcd"), dummy_cloud);

    // Generate a sample dummy pointcloud metadata dictionary
    std::map<std::string, autoware::map_loader::PCDFileMetadata> dummy_metadata_dict;
    autoware::map_loader::PCDFileMetadata dummy_metadata;
    dummy_metadata.min = pcl::PointXYZ(-1.0, -1.0, -1.0);
    dummy_metadata.max = pcl::PointXYZ(1.0, 1.0, 1.0);
    dummy_metadata_dict[owned_map_test_file("dummy.pcd")] = dummy_metadata;

    // Initialize the DifferentialMapLoaderModule with the dummy metadata dictionary
    module_ = std::make_shared<DifferentialMapLoaderModule>(node_.get(), dummy_metadata_dict);

    // Create a client for the GetDifferentialPointCloudMap service
    client_ =
      node_->serviceClient<GetDifferentialPointCloudMap>("service/get_differential_pcd_map");
  }



  std::shared_ptr<autoware::map_loader::map_native::Node> node_;
  std::shared_ptr<DifferentialMapLoaderModule> module_;
  ros::ServiceClient client_;
};

TEST_F(TestDifferentialMapLoaderModule, LoadDifferentialPCDFiles)
{
  // Wait for the GetDifferentialPointCloudMap service to be available
  ASSERT_TRUE(client_.waitForExistence(ros::Duration(3)));

  // Prepare a request for the service
  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();
  request->area.center_x = 0;
  request->area.center_y = 0;
  request->area.radius = 2;
  request->cached_ids.clear();

  // Call the service
  GetDifferentialPointCloudMap service;
  service.request=*request;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ASSERT_TRUE(client_.call(service));
  auto result=std::make_shared<GetDifferentialPointCloudMap::Response>(service.response);
  ASSERT_EQ(static_cast<int>(result->new_pointcloud_with_ids.size()), 1);
  EXPECT_EQ(result->new_pointcloud_with_ids[0].cell_id, owned_map_test_file("dummy.pcd"));
  EXPECT_EQ(static_cast<int>(result->ids_to_remove.size()), 0);
}

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"original_map_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
