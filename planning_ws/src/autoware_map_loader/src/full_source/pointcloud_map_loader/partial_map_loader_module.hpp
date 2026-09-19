// Copyright 2022 The Autoware Contributors
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

#ifndef POINTCLOUD_MAP_LOADER__PARTIAL_MAP_LOADER_MODULE_HPP_
#define POINTCLOUD_MAP_LOADER__PARTIAL_MAP_LOADER_MODULE_HPP_

#include "utils.hpp"

#include <autoware/map_loader/native_node.hpp>

#include <autoware_map_msgs/GetPartialPointCloudMap.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
class PartialMapLoaderModule
{
  using GetPartialPointCloudMap = autoware_map_msgs::GetPartialPointCloudMap;

public:
  explicit PartialMapLoaderModule(
    map_native::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict);

private:
  std::string logger_;

  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict_;
  ros::ServiceServer get_partial_pcd_maps_service_;

  [[nodiscard]] bool on_service_get_partial_point_cloud_map(
    std::shared_ptr<GetPartialPointCloudMap::Request> req,
    std::shared_ptr<GetPartialPointCloudMap::Response> res) const;
  void partial_area_load(
    const autoware_map_msgs::AreaInfo & area,
    const std::shared_ptr<GetPartialPointCloudMap::Response> & response) const;
  [[nodiscard]] autoware_map_msgs::PointCloudMapCellWithID load_point_cloud_map_cell_with_id(
    const std::string & path, const std::string & map_id) const;
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__PARTIAL_MAP_LOADER_MODULE_HPP_
