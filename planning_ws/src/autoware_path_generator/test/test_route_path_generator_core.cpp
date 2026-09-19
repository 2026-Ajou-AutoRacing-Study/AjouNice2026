#include <autoware/path_generator/route_path_generator_core.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>

#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::LaneletRoute route()
{
  autoware_planning_msgs::LaneletRoute result;
  for (const auto & item : std::vector<std::pair<std::vector<lanelet::Id>, lanelet::Id>>{
         {{2239, 2240, 2241, 2242}, 2240}, {{2301, 2302, 2300, 2299}, 2302},
         {{2244, 2245, 2246, 2247}, 2245}, {{2265, 2261, 2262, 2263}, 2261}}) {
    autoware_planning_msgs::LaneletSegment segment;
    segment.preferred_primitive.id = item.second;
    segment.preferred_primitive.primitive_type = "road";
    for (const auto id : item.first) { autoware_planning_msgs::LaneletPrimitive p; p.id = id; p.primitive_type = "road"; segment.primitives.push_back(p); }
    result.segments.push_back(segment);
  }
  result.header.frame_id = "map";
  return result;
}
geometry_msgs::Pose pose()
{
  geometry_msgs::Pose p; p.position.x = 122.81823934652175; p.position.y = 264.7931657588342; p.position.z = 100.0; p.orientation.z = -0.6922317008371615; p.orientation.w = 0.7216753233664642; return p;
}
}  // namespace

TEST(RoutePathGeneratorCore, BuildsReferencePathFromMapRouteAndPose)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_PATH_GENERATOR_TEST_MAP);
  ASSERT_TRUE(map);
  const auto output = autoware::path_generator::generateRoutePath(
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map), route(), pose(), pose(), {3.0, 1.046, 30.0, 5.0});
  ASSERT_TRUE(output);
  EXPECT_EQ(output->header.frame_id, "map");
  EXPECT_FALSE(output->points.empty());
  EXPECT_FALSE(output->points.front().lane_ids.empty());
}

TEST(RoutePathGeneratorCore, FailsClosedForEmptyRoute)
{
  autoware_map_msgs::LaneletMapBin map;
  autoware_planning_msgs::LaneletRoute route;
  geometry_msgs::Pose pose; pose.orientation.w = 1.0;
  EXPECT_FALSE(autoware::path_generator::generateRoutePath(map, route, pose, pose));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
