#include "autoware/map_loader/speed_limit_overrides.hpp"
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace autoware::map_loader
{
namespace
{
std::string sha256(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot read base map: " + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("Cannot initialize map SHA256");
  }
  std::array<char, 65536> buffer;
  while (input) {
    input.read(buffer.data(), buffer.size());
    if (EVP_DigestUpdate(ctx.get(), buffer.data(), input.gcount()) != 1) {
      throw std::runtime_error("Cannot hash base map");
    }
  }
  if (!input.eof()) throw std::runtime_error("Cannot read complete base map");
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest;
  unsigned int size = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest.data(), &size) != 1) {
    throw std::runtime_error("Cannot finalize map SHA256");
  }
  std::ostringstream output;
  for (unsigned int i=0; i<size; ++i) {
    output << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
  }
  return output.str();
}
}

void apply_speed_limit_overrides(
  const lanelet::LaneletMapPtr & map, const std::string & map_file,
  const std::string & overrides_file)
{
  if (overrides_file.empty()) return;
  const auto config = YAML::LoadFile(overrides_file);
  if (config["schema_version"].as<int>() != 1 ||
      config["map_sha256"].as<std::string>() != sha256(map_file)) {
    throw std::runtime_error("Lanelet speed limits do not match the base map SHA256");
  }
  const auto limits = config["lanelet_speed_limits"];
  if (!limits.IsMap()) throw std::runtime_error("lanelet_speed_limits must be a map");
  const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  std::vector<std::pair<lanelet::Id, std::string>> validated;
  for (const auto & item : limits) {
    const auto id = item.first.as<lanelet::Id>();
    const double kmh = item.second.as<double>();
    if (!map->laneletLayer.exists(id) || !std::isfinite(kmh) || kmh <= 0 || kmh > 150) {
      throw std::runtime_error("Invalid lanelet speed cap: " + std::to_string(id));
    }
    const auto lane = map->laneletLayer.get(id);
    if (!rules->canPass(lane) || lane.attributeOr("subtype", std::string()) != "road") {
      throw std::runtime_error("Speed cap target is not a vehicle road: " + std::to_string(id));
    }
    const double original_kmh = rules->speedLimit(lane).speedLimit.value() * 3.6;
    std::ostringstream value;
    value << std::setprecision(12) << std::min(kmh, original_kmh);
    validated.emplace_back(id, value.str());
  }
  for (const auto & item : validated) {
    map->laneletLayer.get(item.first).attributes()["speed_limit"] = item.second;
  }
}
}
