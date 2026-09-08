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

#include "pointcloud_map_loader/partial_map_loader_module.hpp"

#include "native_test_support.hpp"

#include <autoware_map_msgs/GetPartialPointCloudMap.h>

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

using autoware::map_loader::PartialMapLoaderModule;
using autoware_map_msgs::GetPartialPointCloudMap;

class TestPartialMapLoaderModule : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node
    node_ = std::make_shared<autoware::map_loader::map_native::Node>(ros::NodeHandle("/test_partial_map_loader_module"));

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

    // Initialize the PartialMapLoaderModule with the dummy metadata dictionary
    module_ = std::make_shared<PartialMapLoaderModule>(node_.get(), dummy_metadata_dict);

    // Create a client for the GetPartialPointCloudMap service
    client_ = node_->serviceClient<GetPartialPointCloudMap>("service/get_partial_pcd_map");
  }



  std::shared_ptr<autoware::map_loader::map_native::Node> node_;
  std::shared_ptr<PartialMapLoaderModule> module_;
  ros::ServiceClient client_;
};

TEST_F(TestPartialMapLoaderModule, LoadPartialPCDFiles)
{
  // Wait for the GetPartialPointCloudMap service to be available
  ASSERT_TRUE(client_.waitForExistence(ros::Duration(3)));

  // Prepare a request for the service
  auto request = std::make_shared<GetPartialPointCloudMap::Request>();
  request->area.center_x = 0;
  request->area.center_y = 0;
  request->area.radius = 2;

  // Call the service
  GetPartialPointCloudMap service;
  service.request=*request;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ASSERT_TRUE(client_.call(service));
  auto result=std::make_shared<GetPartialPointCloudMap::Response>(service.response);
  ASSERT_EQ(static_cast<int>(result->new_pointcloud_with_ids.size()), 1);
  EXPECT_EQ(result->new_pointcloud_with_ids[0].cell_id, owned_map_test_file("dummy.pcd"));
}

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"original_map_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
