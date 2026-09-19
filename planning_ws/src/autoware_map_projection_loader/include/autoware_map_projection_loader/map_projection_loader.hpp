#ifndef AUTOWARE_MAP_PROJECTION_LOADER_MAP_PROJECTION_LOADER_HPP_
#define AUTOWARE_MAP_PROJECTION_LOADER_MAP_PROJECTION_LOADER_HPP_

#include <autoware_map_msgs/MapProjectorInfo.h>

#include <string>

namespace autoware::map_projection_loader
{
autoware_map_msgs::MapProjectorInfo load_info_from_yaml(const std::string & filename);
autoware_map_msgs::MapProjectorInfo load_info_from_lanelet2_map(const std::string & filename);
autoware_map_msgs::MapProjectorInfo load_map_projector_info(
  const std::string & yaml_filename, const std::string & lanelet2_map_filename);
}  // namespace autoware::map_projection_loader

#endif
