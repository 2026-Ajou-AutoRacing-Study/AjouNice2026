#include "autoware/map_loader/speed_limit_overrides.hpp"
#include <autoware/lanelet2_utils/conversion.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <fstream>
#include <string>
#include <unistd.h>

namespace
{
class SpeedLimitOverrides : public ::testing::Test
{
protected:
  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
  std::string file;
  const std::string source = std::string(AUTOWARE_MAP_LOADER_TEST_DATA_DIR) + "/lanelet2_map.osm";
  const std::string hash = "1469dd0c56d72d0d460eff1c1a091761a01094bb83c046a50d63dac120ffafad";
  void SetUp() override
  {
    char pattern[] = "/tmp/lanelet_speed_test_XXXXXX";
    const int fd = mkstemp(pattern);
    ASSERT_GE(fd, 0);
    close(fd);
    file = pattern;
    for (int i=0; i<2; ++i) {
      lanelet::LineString3d left(10+i*10, {
        lanelet::Point3d(1+i*10, 0, double(i*4+2), 0),
        lanelet::Point3d(2+i*10, 20, double(i*4+2), 0)});
      lanelet::LineString3d right(11+i*10, {
        lanelet::Point3d(3+i*10, 0, double(i*4), 0),
        lanelet::Point3d(4+i*10, 20, double(i*4), 0)});
      lanelet::Lanelet lane(100+i, left, right);
      lane.attributes()["subtype"] = "road";
      lane.attributes()["location"] = "urban";
      lane.attributes()["one_way"] = "yes";
      lane.attributes()["speed_limit"] = i == 0 ? "150" : "30";
      map->add(lane);
    }
  }
  void TearDown() override {unlink(file.c_str());}
  void write(const std::string & limits, const std::string & fingerprint = "")
  {
    std::ofstream stream(file);
    stream << "schema_version: 1\nmap_sha256: " << (fingerprint.empty() ? hash : fingerprint)
           << "\nlanelet_speed_limits: " << limits << "\n";
  }
  double speed(const lanelet::LaneletMapConstPtr & current, lanelet::Id id)
  {
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    return rules->speedLimit(current->laneletLayer.get(id)).speedLimit.value();
  }
};
TEST_F(SpeedLimitOverrides, SerializesFiftyKmhAndPreservesLowerExistingLimit)
{
  write("{'100': 50.0, '101': 50.0}");
  autoware::map_loader::apply_speed_limit_overrides(map, source, file);
  EXPECT_NEAR(speed(map, 100), 50.0/3.6, 1e-8);
  EXPECT_NEAR(speed(map, 101), 30.0/3.6, 1e-8);
  const auto binary = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  const auto restored = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(binary);
  EXPECT_NEAR(speed(restored, 100), 50.0/3.6, 1e-8);
  EXPECT_NEAR(speed(restored, 101), 30.0/3.6, 1e-8);
}
TEST_F(SpeedLimitOverrides, InvalidLaterEntryDoesNotPartiallyApply)
{
  write("{'100': 50.0, '999': 50.0}");
  EXPECT_THROW(autoware::map_loader::apply_speed_limit_overrides(map, source, file), std::runtime_error);
  EXPECT_NEAR(speed(map, 100), 150.0/3.6, 1e-8);
}
TEST_F(SpeedLimitOverrides, RejectsWrongBaseMapHashAndInvalidSpeeds)
{
  write("{'100': 50.0}", std::string(64, '0'));
  EXPECT_THROW(autoware::map_loader::apply_speed_limit_overrides(map, source, file), std::runtime_error);
  for (const auto & value : {"0", "-1", "151", ".nan", ".inf"}) {
    write(std::string("{'100': ") + value + "}");
    EXPECT_THROW(autoware::map_loader::apply_speed_limit_overrides(map, source, file), std::runtime_error);
  }
  EXPECT_NEAR(speed(map, 100), 150.0/3.6, 1e-8);
}
TEST_F(SpeedLimitOverrides, EmptySelectionsAndOptOutPreserveSpeeds)
{
  write("{}");
  autoware::map_loader::apply_speed_limit_overrides(map, source, file);
  autoware::map_loader::apply_speed_limit_overrides(map, source, "");
  EXPECT_NEAR(speed(map, 100), 150.0/3.6, 1e-8);
}
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
