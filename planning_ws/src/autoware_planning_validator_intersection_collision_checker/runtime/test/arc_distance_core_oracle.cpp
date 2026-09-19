#include "autoware/planning_validator_intersection_collision_checker/arc_distance_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <algorithm>

namespace
{
pcl::PointXYZ point_for(const lanelet::ConstPoint3d & point)
{
  pcl::PointXYZ rendered;
  rendered.x = static_cast<float>(point.x());
  rendered.y = static_cast<float>(point.y());
  rendered.z = static_cast<float>(point.z());
  return rendered;
}
}  // namespace

TEST(ArcDistanceCore, SelectsNearestPointBeforeOverlapAndRejectsPassedPoint)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  ASSERT_GE(lane.centerline().size(), 3U);
  const auto & start = lane.centerline().front();
  const auto & middle = lane.centerline()[lane.centerline().size() / 2U];
  const auto & end = lane.centerline().back();
  geometry_msgs::Pose overlap;
  overlap.position.x = static_cast<double>(static_cast<float>(end.x()));
  overlap.position.y = static_cast<double>(static_cast<float>(end.y()));
  overlap.position.z = static_cast<double>(static_cast<float>(end.z()));
  overlap.orientation.w = 1.0;
  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back(point_for(start));
  points.push_back(point_for(middle));
  points.push_back(point_for(end));
  const lanelet::ConstLanelets lanelets{lane};
  const auto selected = autoware::planning_validator::intersection_collision_checker_ros1::nearest_approaching_point(points, lanelets, overlap);
  pcl::PointCloud<pcl::PointXYZ> start_only;
  start_only.push_back(point_for(start));
  pcl::PointCloud<pcl::PointXYZ> middle_only;
  middle_only.push_back(point_for(middle));
  pcl::PointCloud<pcl::PointXYZ> overlap_only;
  overlap_only.push_back(point_for(end));
  const auto start_candidate = autoware::planning_validator::intersection_collision_checker_ros1::nearest_approaching_point(start_only, lanelets, overlap);
  const auto middle_candidate = autoware::planning_validator::intersection_collision_checker_ros1::nearest_approaching_point(middle_only, lanelets, overlap);
  const auto passed_candidate = autoware::planning_validator::intersection_collision_checker_ros1::nearest_approaching_point(overlap_only, lanelets, overlap);
  ASSERT_TRUE(selected.has_value());
  ASSERT_TRUE(start_candidate.has_value());
  EXPECT_FALSE(passed_candidate.has_value());
  const double expected = middle_candidate.has_value()
    ? std::min(start_candidate->distance_to_overlap, middle_candidate->distance_to_overlap)
    : start_candidate->distance_to_overlap;
  EXPECT_NEAR(selected->distance_to_overlap, expected, 1e-6);
  EXPECT_GT(selected->distance_to_overlap, 0.0);
}
