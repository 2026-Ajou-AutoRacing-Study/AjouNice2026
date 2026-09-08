#include "autoware_map_projection_loader/map_projection_loader.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <string>

namespace
{
std::string temp_file(const std::string & suffix)
{
  return "/tmp/autoware_map_projection_loader_" + suffix;
}
void write_file(const std::string & path, const std::string & contents)
{
  std::ofstream file(path);
  ASSERT_TRUE(file.good());
  file << contents;
}

std::string osm_with_nodes(const std::string & nodes)
{
  return "<?xml version=\"1.0\"?>\n<osm version=\"0.6\" generator=\"lanelet2\">\n" + nodes + "</osm>\n";
}
}  // namespace

TEST(MapProjectionLoader, LoadsMGRSFromYaml)
{
  const auto path = temp_file("mgrs.yaml");
  write_file(path, "projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n");
  const auto actual = autoware::map_projection_loader::load_info_from_yaml(path);
  EXPECT_EQ(actual.projector_type, "MGRS");
  EXPECT_EQ(actual.vertical_datum, "WGS84");
  EXPECT_EQ(actual.mgrs_grid, "54SUE");
  EXPECT_FLOAT_EQ(actual.scale_factor, 0.9996F);
}

TEST(MapProjectionLoader, LoadsLocalAndLegacyLocalFromYaml)
{
  const auto local = temp_file("local.yaml");
  const auto legacy = temp_file("legacy.yaml");
  write_file(local, "projector_type: Local\n");
  write_file(legacy, "projector_type: local\n");
  EXPECT_EQ(autoware::map_projection_loader::load_info_from_yaml(local).projector_type, "Local");
  EXPECT_EQ(autoware::map_projection_loader::load_info_from_yaml(legacy).projector_type, "Local");
}

TEST(MapProjectionLoader, LoadsCartesianAndTransverseMercatorFromYaml)
{
  const auto cartesian = temp_file("cartesian.yaml");
  const auto transverse = temp_file("transverse.yaml");
  write_file(cartesian, "projector_type: LocalCartesianUTM\nvertical_datum: WGS84\nmap_origin: {latitude: 35.6762, longitude: 139.6503}\n");
  write_file(transverse, "projector_type: TransverseMercator\nvertical_datum: WGS84\nmap_origin: {latitude: 35.6762, longitude: 139.6503}\nscale_factor: 0.997\n");
  const auto cartesian_actual = autoware::map_projection_loader::load_info_from_yaml(cartesian);
  const auto transverse_actual = autoware::map_projection_loader::load_info_from_yaml(transverse);
  EXPECT_DOUBLE_EQ(cartesian_actual.map_origin.latitude, 35.6762);
  EXPECT_FLOAT_EQ(cartesian_actual.scale_factor, 0.9996F);
  EXPECT_DOUBLE_EQ(transverse_actual.map_origin.longitude, 139.6503);
  EXPECT_FLOAT_EQ(transverse_actual.scale_factor, 0.997F);
}

TEST(MapProjectionLoader, RejectsInvalidProjectorAndScale)
{
  const auto invalid_type = temp_file("invalid_type.yaml");
  const auto invalid_scale = temp_file("invalid_scale.yaml");
  write_file(invalid_type, "projector_type: Invalid\n");
  write_file(invalid_scale, "projector_type: TransverseMercator\nvertical_datum: WGS84\nmap_origin: {latitude: 0.0, longitude: 0.0}\nscale_factor: 0.0\n");
  EXPECT_THROW(autoware::map_projection_loader::load_info_from_yaml(invalid_type), std::runtime_error);
  EXPECT_THROW(autoware::map_projection_loader::load_info_from_yaml(invalid_scale), std::runtime_error);
}

TEST(MapProjectionLoader, UsesYamlBeforeLaneletFallbackAndRejectsAbsentInputs)
{
  const auto yaml = temp_file("priority.yaml");
  write_file(yaml, "projector_type: Local\n");
  EXPECT_EQ(autoware::map_projection_loader::load_map_projector_info(yaml, "/missing.osm").projector_type, "Local");
  EXPECT_THROW(autoware::map_projection_loader::load_map_projector_info("/missing.yaml", "/missing.osm"), std::runtime_error);
}

TEST(MapProjectionLoader, LoadsMgrsGridFromLanelet2Fallback)
{
  const auto map = temp_file("mgrs.osm");
  write_file(map, osm_with_nodes("  <node id=\"1\" lat=\"35.62554985786315\" lon=\"139.78246505089274\"/>\n"));
  const auto actual = autoware::map_projection_loader::load_map_projector_info("/missing.yaml", map);
  EXPECT_EQ(actual.projector_type, "MGRS");
  EXPECT_EQ(actual.mgrs_grid, "54SUE");
  EXPECT_EQ(actual.vertical_datum, "WGS84");
}

TEST(MapProjectionLoader, LoadsLocalGridFromLanelet2Fallback)
{
  const auto map = temp_file("local.osm");
  write_file(map, osm_with_nodes(
    "  <node id=\"1\" lat=\"\" lon=\"\"/>\n"
    "  <node id=\"2\" lat=\"\" lon=\"\"/>\n"));
  const auto actual = autoware::map_projection_loader::load_map_projector_info("/missing.yaml", map);
  EXPECT_EQ(actual.projector_type, "Local");
  EXPECT_EQ(actual.vertical_datum, "WGS84");
}

TEST(MapProjectionLoader, TreatsMixedZeroAndNonzeroLanelet2MapAsMgrs)
{
  const auto map = temp_file("mixed.osm");
  write_file(map, osm_with_nodes(
    "  <node id=\"1\" lat=\"0.0\" lon=\"0.0\"/>\n"
    "  <node id=\"2\" lat=\"0.00001\" lon=\"0.00001\"/>\n"
    "  <node id=\"3\" lat=\"0.00002\" lon=\"0.00002\"/>\n"));
  const auto actual = autoware::map_projection_loader::load_map_projector_info("/missing.yaml", map);
  EXPECT_EQ(actual.projector_type, "MGRS");
  EXPECT_EQ(actual.vertical_datum, "WGS84");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
