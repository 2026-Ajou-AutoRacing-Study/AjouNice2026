// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#define AUTOWARE__MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_

#include <autoware_lanelet2_extension/version.hpp>
#include <autoware/map_loader/native_node.hpp>

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_map_msgs/MapProjectorInfo.h>

#include <lanelet2_projection/UTM.h>

#include <memory>
#include <string>

namespace autoware::map_loader
{
class Lanelet2MapLoaderNode : public map_native::Node
{
public:
  static constexpr lanelet::autoware::Version version = lanelet::autoware::version;

public:
  explicit Lanelet2MapLoaderNode(ros::NodeHandle private_node);

  static lanelet::LaneletMapPtr load_map(
    const std::string & lanelet2_filename,
    const autoware_map_msgs::MapProjectorInfo & projector_info);
  static autoware_map_msgs::LaneletMapBin create_map_bin_msg(
    const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename,
    const ros::Time & now);

private:
  void on_map_projector_info(const autoware_map_msgs::MapProjectorInfo::ConstPtr msg);

  ros::Subscriber sub_map_projector_info_;
  std::shared_ptr<ros::Publisher> pub_map_bin_;
};
}  // namespace autoware::map_loader

#endif  // AUTOWARE__MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
