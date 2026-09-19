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

#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_

#include <autoware/map_loader/native_node.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <boost/optional.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

namespace autoware::map_loader
{
class PointcloudMapLoaderModule
{
public:
  explicit PointcloudMapLoaderModule(
    map_native::Node * node, const std::vector<std::string> & pcd_paths,
    const std::string & publisher_name, const bool use_downsample);

private:
  std::string logger_;
  std::shared_ptr<ros::Publisher> pub_pointcloud_map_;

  [[nodiscard]] sensor_msgs::PointCloud2 load_pcd_files(
    const std::vector<std::string> & pcd_paths, const boost::optional<float> leaf_size) const;
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_
