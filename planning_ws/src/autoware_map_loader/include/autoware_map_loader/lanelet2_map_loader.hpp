#ifndef AUTOWARE_MAP_LOADER_LANELET2_MAP_LOADER_HPP_
#define AUTOWARE_MAP_LOADER_LANELET2_MAP_LOADER_HPP_

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_map_msgs/MapProjectorInfo.h>
#include <lanelet2_core/LaneletMap.h>

#include <string>

namespace autoware::map_loader
{
lanelet::LaneletMapPtr load_lanelet2_map(
  const std::string & lanelet2_filename, const autoware_map_msgs::MapProjectorInfo & projector_info);
autoware_map_msgs::LaneletMapBin create_map_bin_msg(
  const lanelet::LaneletMapPtr & map, const std::string & lanelet2_filename);
}  // namespace autoware::map_loader

#endif
