#include "autoware_map_projection_loader/map_projection_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

namespace autoware::map_projection_loader
{
namespace
{
constexpr char kLocal[] = "Local";
constexpr char kLocalCartesianUtm[] = "LocalCartesianUTM";
constexpr char kLocalCartesian[] = "LocalCartesian";
constexpr char kMgrs[] = "MGRS";
constexpr char kTransverseMercator[] = "TransverseMercator";
}  // namespace

autoware_map_msgs::MapProjectorInfo load_info_from_yaml(const std::string & filename)
{
  const YAML::Node data = YAML::LoadFile(filename);
  autoware_map_msgs::MapProjectorInfo msg;
  msg.projector_type = data["projector_type"].as<std::string>();
  if (msg.projector_type == kMgrs) {
    msg.vertical_datum = data["vertical_datum"].as<std::string>();
    msg.mgrs_grid = data["mgrs_grid"].as<std::string>();
  } else if (
    msg.projector_type == kLocalCartesianUtm ||
    msg.projector_type == kLocalCartesian ||
    msg.projector_type == kTransverseMercator) {
    msg.vertical_datum = data["vertical_datum"].as<std::string>();
    msg.map_origin.latitude = data["map_origin"]["latitude"].as<double>();
    msg.map_origin.longitude = data["map_origin"]["longitude"].as<double>();
    msg.map_origin.altitude = 0.0;
  } else if (msg.projector_type == kLocal) {
    // The local projector has no additional YAML fields.
  } else if (msg.projector_type == "local") {
    msg.projector_type = kLocal;
  } else {
    throw std::runtime_error("Invalid map projector type. Currently supported types: MGRS, LocalCartesian, LocalCartesianUTM, TransverseMercator, and local");
  }

  constexpr float kUtmScale = 0.9996F;
  constexpr float kLocalScale = 1.0F;
  if (msg.projector_type == kTransverseMercator) {
    msg.scale_factor = data["scale_factor"] ? data["scale_factor"].as<float>() : kUtmScale;
  } else if (
    msg.projector_type == kMgrs || msg.projector_type == kLocalCartesianUtm) {
    msg.scale_factor = kUtmScale;
  } else {
    msg.scale_factor = kLocalScale;
  }
  if (msg.scale_factor <= 0.0F) {
    throw std::runtime_error("Invalid scale factor. The scale factor must be a value greater than 0.");
  }
  return msg;
}

autoware_map_msgs::MapProjectorInfo load_map_projector_info(
  const std::string & yaml_filename, const std::string & lanelet2_map_filename)
{
  if (std::filesystem::exists(yaml_filename)) {
    return load_info_from_yaml(yaml_filename);
  }
  if (std::filesystem::exists(lanelet2_map_filename)) {
    return load_info_from_lanelet2_map(lanelet2_map_filename);
  }
  throw std::runtime_error("No map projector info files found. Please provide either map_projector_info.yaml or lanelet2_map.osm");
}
}  // namespace autoware::map_projection_loader
