// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__MAP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__MAP_HPP_

#include <autoware/component_interface_specs/utils.hpp>

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_map_msgs/MapProjectorInfo.h>
#include <sensor_msgs/PointCloud2.h>

namespace autoware::component_interface_specs::map
{

struct MapProjectorInfo
{
  using Message = autoware_map_msgs::MapProjectorInfo;
  static constexpr char name[] = "/map/map_projector_info";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::TransientLocal;
};

struct PointCloudMap
{
  using Message = sensor_msgs::PointCloud2;
  static constexpr char name[] = "/map/point_cloud_map";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::TransientLocal;
};

struct VectorMap
{
  using Message = autoware_map_msgs::LaneletMapBin;
  static constexpr char name[] = "/map/vector_map";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::TransientLocal;
};

}  // namespace autoware::component_interface_specs::map

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__MAP_HPP_
