#include "autoware_map_loader/lanelet2_map_loader.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(Lanelet2MapLoader, RejectsMissingMap)
{
  autoware_map_msgs::MapProjectorInfo projector;
  projector.projector_type = "Local";
  EXPECT_THROW(
    autoware::map_loader::load_lanelet2_map("/definitely/missing/map.osm", projector),
    std::runtime_error);
}

TEST(Lanelet2MapLoader, LoadsLocalFixtureAndSerializesMapMetadata)
{
  autoware_map_msgs::MapProjectorInfo projector;
  projector.projector_type = "Local";
  const std::string map_path = std::string(AUTOWARE_MAP_LOADER_TEST_DATA_DIR) + "/lanelet2_map.osm";

  const auto map = autoware::map_loader::load_lanelet2_map(map_path, projector);
  ASSERT_NE(map, nullptr);
  EXPECT_FALSE(map->laneletLayer.empty());

  const auto map_message = autoware::map_loader::create_map_bin_msg(map, map_path);
  EXPECT_EQ(map_message.header.frame_id, "map");
  EXPECT_EQ(map_message.version_map_format, "1");
  EXPECT_EQ(map_message.version_map, "2");
  EXPECT_FALSE(map_message.data.empty());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_lanelet2_map_loader");
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
