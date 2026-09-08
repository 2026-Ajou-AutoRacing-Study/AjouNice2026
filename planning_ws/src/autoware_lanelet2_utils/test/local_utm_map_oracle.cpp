#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/map_handler.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdlib>
#include <iostream>

int main(int argc, char ** argv)
{
  if (argc < 6) {
    std::cerr << "usage: lanelet2_utils_local_utm_map_oracle <map.osm> <lat> <lon> <alt> <lanelet-id> [...]" << std::endl;
    return 64;
  }
  const auto map = autoware::experimental::lanelet2_utils::load_local_cartesian_utm_map(
    argv[1], std::strtod(argv[2], nullptr), std::strtod(argv[3], nullptr),
    std::strtod(argv[4], nullptr));
  if (!map || map->laneletLayer.empty()) {
    std::cerr << "LocalCartesianUTM map loader returned no lanelets" << std::endl;
    return 1;
  }
  for (int index = 5; index < argc; ++index) {
    const auto id = static_cast<lanelet::Id>(std::strtoll(argv[index], nullptr, 10));
    if (!map->laneletLayer.exists(id)) {
      std::cerr << "required lanelet absent: " << id << std::endl;
      return 2;
    }
  }
  const auto message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  if (message.data.empty()) {
    std::cerr << "LaneletMapBin serialization produced no data" << std::endl;
    return 3;
  }
  const auto handler = autoware::experimental::lanelet2_utils::MapHandler::create(message);
  if (!handler || !handler->lanelet_map_ptr() || !handler->routing_graph_ptr()) {
    std::cerr << "MapHandler could not construct routing state from LocalCartesianUTM map" << std::endl;
    return 4;
  }
  std::cout << "lanelet2_utils_local_utm_map_oracle=PASS lanelets="
            << map->laneletLayer.size() << " bytes=" << message.data.size() << std::endl;
  return 0;
}
