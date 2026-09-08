#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>

#include <autoware_planning_msgs/LaneletRoute.h>
#include <lanelet2_io/Io.h>
#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: lanelet2_extension_oracle <lanelet_map.osm>" << std::endl;
    return 64;
  }
  ros::init(argc, argv, "lanelet2_extension_oracle");
  lanelet::ErrorMessages errors;
  lanelet::projection::MGRSProjector projector;
  auto parsed_map = lanelet::load(
    argv[1], lanelet::io_handlers::AutowareOsmParser::name(), projector, &errors);
  lanelet::LaneletMapPtr map(std::move(parsed_map));
  if (!map || map->laneletLayer.empty()) {
    std::cerr << "Autoware OSM parser returned no lanelets" << std::endl;
    return 1;
  }

  std::string format_version;
  std::string map_version;
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    argv[1], &format_version, &map_version);

  autoware_planning_msgs::LaneletRoute route;
  autoware_planning_msgs::LaneletSegment segment;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = map->laneletLayer.begin()->id();
  primitive.primitive_type = "lanelet";
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  route.segments.push_back(segment);
  if (!lanelet::utils::route::isRouteValid(route, map, false)) {
    std::cerr << "route checker rejected a parsed lanelet primitive" << std::endl;
    return 2;
  }

  route.segments.front().primitives.front().id = -1;
  if (lanelet::utils::route::isRouteValid(route, map, false)) {
    std::cerr << "route checker accepted an unknown route primitive" << std::endl;
    return 3;
  }
  std::cout << "lanelet2_extension_oracle=PASS lanelets=" << map->laneletLayer.size()
            << " format_version=" << format_version << " map_version=" << map_version << std::endl;
  return 0;
}
