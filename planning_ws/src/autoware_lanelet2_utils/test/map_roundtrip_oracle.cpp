#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/map_handler.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <iostream>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: lanelet2_utils_map_roundtrip_oracle <map.osm>" << std::endl;
    return 64;
  }
  const auto original = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(argv[1]);
  if (!original || original->laneletLayer.empty()) {
    std::cerr << "MGRS map loader returned no lanelets" << std::endl;
    return 1;
  }
  const auto message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(original);
  if (message.data.empty()) {
    std::cerr << "LaneletMapBin serialization produced no data" << std::endl;
    return 2;
  }
  const auto restored = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(message);
  if (!restored || restored->laneletLayer.size() != original->laneletLayer.size()) {
    std::cerr << "LaneletMapBin roundtrip changed lanelet cardinality" << std::endl;
    return 3;
  }
  const auto handler = autoware::experimental::lanelet2_utils::MapHandler::create(message);
  if (!handler || !handler->lanelet_map_ptr() || !handler->routing_graph_ptr()) {
    std::cerr << "MapHandler could not construct routing state from LaneletMapBin" << std::endl;
    return 4;
  }
  std::cout << "lanelet2_utils_map_roundtrip_oracle=PASS lanelets="
            << restored->laneletLayer.size() << " bytes=" << message.data.size() << std::endl;
  return 0;
}
