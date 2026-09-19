#include "autoware_map_loader/lanelet2_map_loader.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <stdexcept>

namespace autoware::map_loader
{
namespace
{
class LocalProjector : public lanelet::Projector
{
public:
  LocalProjector() : Projector(lanelet::Origin(lanelet::GPSPoint{})) {}
  lanelet::BasicPoint3d forward(const lanelet::GPSPoint & gps) const override
  {
    return {0.0, 0.0, gps.ele};
  }
  lanelet::GPSPoint reverse(const lanelet::BasicPoint3d & point) const override
  {
    return {0.0, 0.0, point.z()};
  }
};
}

lanelet::LaneletMapPtr load_lanelet2_map(
  const std::string & lanelet2_filename, const autoware_map_msgs::MapProjectorInfo & projector_info)
{
  lanelet::ErrorMessages errors;
  if (projector_info.projector_type != "Local") {
    std::unique_ptr<lanelet::Projector> projector;
    const lanelet::GPSPoint origin_point{
      projector_info.map_origin.latitude, projector_info.map_origin.longitude,
      projector_info.map_origin.altitude};
    const lanelet::Origin origin{origin_point};
    if (projector_info.projector_type == "MGRS") {
      auto mgrs = std::make_unique<lanelet::projection::MGRSProjector>();
      mgrs->setMGRSCode(projector_info.mgrs_grid);
      projector = std::move(mgrs);
    } else if (projector_info.projector_type == "LocalCartesianUTM") {
      projector = std::make_unique<lanelet::projection::UtmProjector>(origin);
    } else if (projector_info.projector_type == "LocalCartesian") {
      projector = std::make_unique<lanelet::projection::LocalCartesianProjector>(origin);
    } else if (projector_info.projector_type == "TransverseMercator") {
      projector = std::make_unique<lanelet::projection::TransverseMercatorProjector>(
        origin, projector_info.scale_factor);
    } else {
      throw std::invalid_argument("Invalid map projector type: " + projector_info.projector_type);
    }
    auto map = lanelet::load(lanelet2_filename, *projector, &errors);
    if (errors.empty()) return lanelet::LaneletMapPtr(map.release());
  } else {
    const LocalProjector projector;
    auto map = lanelet::load(lanelet2_filename, projector, &errors);
    if (!errors.empty()) return nullptr;
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) point.x() = point.attribute("local_x").asDouble().value();
      if (point.hasAttribute("local_y")) point.y() = point.attribute("local_y").asDouble().value();
    }
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }
    return lanelet::LaneletMapPtr(map.release());
  }
  return nullptr;
}

autoware_map_msgs::LaneletMapBin create_map_bin_msg(
  const lanelet::LaneletMapPtr & map, const std::string & lanelet2_filename)
{
  std::string format_version;
  std::string map_version;
  lanelet::io_handlers::AutowareOsmParser::parseVersions(lanelet2_filename, &format_version, &map_version);
  auto msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.version_map_format = format_version;
  msg.version_map = map_version;
  return msg;
}
}  // namespace autoware::map_loader
