#include "autoware_map_projection_loader/map_projection_loader.hpp"

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>

#include <sstream>
#include <stdexcept>

namespace autoware::map_projection_loader
{
autoware_map_msgs::MapProjectorInfo load_info_from_lanelet2_map(const std::string & filename)
{
  lanelet::ErrorMessages errors;
  lanelet::projection::MGRSProjector projector;
  const auto map = lanelet::load(filename, projector, &errors);
  if (!errors.empty()) {
    std::stringstream ss;
    ss << "Error occurred while loading lanelet2 map:\n";
    for (const auto & error : errors) ss << "- " << error << "\n";
    throw std::runtime_error(ss.str());
  }
  bool is_local = true;
  for (const auto & point : map->pointLayer) {
    const auto gps_point = projector.reverse(point);
    if (gps_point.lat != 0.0 || gps_point.lon != 0.0) {
      is_local = false;
      break;
    }
  }
  autoware_map_msgs::MapProjectorInfo msg;
  if (is_local) {
    msg.projector_type = "Local";
  } else {
    msg.projector_type = "MGRS";
    msg.mgrs_grid = projector.getProjectedMGRSGrid();
  }
  msg.vertical_datum = "WGS84";
  return msg;
}
}  // namespace autoware::map_projection_loader
