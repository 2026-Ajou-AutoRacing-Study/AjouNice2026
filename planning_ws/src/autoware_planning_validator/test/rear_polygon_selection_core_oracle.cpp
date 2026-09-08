#include "autoware/planning_validator/rear_polygon_selection_core.hpp"

#include <gtest/gtest.h>

TEST(RearPolygonSelectionCore, UsesCirclePrefilterThenStrictPolygonContainment)
{
  using autoware::planning_validator::RearDetectionPolygon;
  using autoware::planning_validator::RearPointCloud;
  RearDetectionPolygon square{{0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F},
    {2.0F, 2.0F, 0.0F}, {0.0F, 2.0F, 0.0F}};
  RearPointCloud cloud;
  cloud.push_back({1.0F, 1.0F, 1.0F});     // inside
  cloud.push_back({2.0F, 1.0F, 1.0F});     // boundary: source uses within => exclude
  cloud.push_back({1.9F, 1.9F, 1.0F});     // circle and polygon inside
  cloud.push_back({2.1F, 1.0F, 1.0F});     // circle candidate but outside polygon
  cloud.push_back({10.0F, 10.0F, 1.0F});   // rejected by circle

  const auto result = autoware::planning_validator::select_rear_points_in_detection_polygons(
    {square}, cloud);
  ASSERT_EQ(result->size(), 2U);
  EXPECT_FLOAT_EQ((*result)[0].x, 1.0F);
  EXPECT_FLOAT_EQ((*result)[1].x, 1.9F);
}

TEST(RearPolygonSelectionCore, IgnoresDegeneratePolygons)
{
  using autoware::planning_validator::RearDetectionPolygon;
  using autoware::planning_validator::RearPointCloud;
  RearPointCloud cloud;
  cloud.push_back({0.0F, 0.0F, 0.0F});
  EXPECT_TRUE(autoware::planning_validator::select_rear_points_in_detection_polygons(
    {RearDetectionPolygon{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}}}, cloud)->empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
