// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <gtest/gtest.h>

namespace
{
using autoware_internal_planning_msgs::PathPointWithLaneId;
using autoware_internal_planning_msgs::PathWithLaneId;

PathWithLaneId makePath(const size_t count, const double spacing, const double velocity = 1.0)
{
  PathWithLaneId path;
  for (size_t index = 0; index < count; ++index) {
    PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(index) * spacing;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = velocity;
    point.lane_ids = {100 + static_cast<int64_t>(index)};
    path.points.push_back(point);
  }
  return path;
}

TEST(BehaviorPathPlanningPathUtilTest, ArcLengthAndIndexTraversal)
{
  using autoware::behavior_path_planner::utils::calcPathArcLengthArray;
  using autoware::behavior_path_planner::utils::getIdxByArclength;
  const auto path = makePath(10, 1.0);
  const auto arc = calcPathArcLengthArray(path);
  ASSERT_EQ(arc.size(), path.points.size());
  for (size_t index = 0; index < arc.size(); ++index) EXPECT_DOUBLE_EQ(arc.at(index), index);
  EXPECT_EQ(getIdxByArclength(path, 0, -1.0), 0U);
  EXPECT_EQ(getIdxByArclength(path, 5, -2.0), 2U);
  EXPECT_EQ(getIdxByArclength(path, 3, 4.0), 8U);
  EXPECT_THROW(getIdxByArclength(PathWithLaneId{}, 0, 1.0), std::runtime_error);
}

TEST(BehaviorPathPlanningPathUtilTest, ClipsAndDividesAtReversal)
{
  using autoware::behavior_path_planner::utils::clipPathLength;
  using autoware::behavior_path_planner::utils::dividePath;
  using autoware::behavior_path_planner::utils::getReversingIndices;
  auto path = makePath(10, 1.0);
  clipPathLength(path, 5, 10.0, 1.0);
  ASSERT_EQ(path.points.size(), 7U);
  EXPECT_DOUBLE_EQ(path.points.front().point.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(path.points.back().point.pose.position.x, 9.0);

  auto reversing = makePath(10, 1.0);
  reversing.points.at(7).point.longitudinal_velocity_mps = -1.0;
  const auto indices = getReversingIndices(reversing);
  ASSERT_EQ(indices.size(), 2U);
  EXPECT_EQ(indices.at(0), 6U);
  EXPECT_EQ(indices.at(1), 7U);
  const auto divided = dividePath(reversing, indices);
  ASSERT_EQ(divided.size(), 3U);
  EXPECT_EQ(divided.at(0).points.size(), 7U);
  EXPECT_EQ(divided.at(1).points.size(), 2U);
  EXPECT_EQ(divided.at(2).points.size(), 3U);
}

TEST(BehaviorPathPlanningPathUtilTest, CombinesWithoutBoundaryDuplicate)
{
  using autoware::behavior_path_planner::utils::combinePath;
  auto first = makePath(4, 0.5);
  auto second = makePath(20, 0.25);
  for (auto & point : second.points) point.point.pose.position.x += 1.5;
  const auto combined = combinePath(first, second);
  ASSERT_EQ(combined.points.size(), 23U);
  for (size_t index = 0; index < first.points.size(); ++index) {
    EXPECT_DOUBLE_EQ(combined.points.at(index).point.pose.position.x, index * 0.5);
  }
  EXPECT_DOUBLE_EQ(combined.points.at(4).point.pose.position.x, 1.75);
  EXPECT_DOUBLE_EQ(combined.points.back().point.pose.position.x, 6.25);
}
TEST(BehaviorPathPlanningPathUtilTest, NonFiniteResamplingFailsWithoutUnboundedLoop)
{
  using autoware::behavior_path_planner::utils::resamplePathWithSpline;
  auto path = makePath(4, 1.0);
  path.points.at(2).point.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(resamplePathWithSpline(path, 1.0), std::invalid_argument);
  EXPECT_THROW(resamplePathWithSpline(makePath(4, 1.0),
    std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(BehaviorPathPlanningPathUtilTest, CrosswalkDistanceUsesActualConflictGraph)
{
  lanelet::Lanelet road(100,
    lanelet::LineString3d(101, {lanelet::Point3d(102, 0, 2, 0), lanelet::Point3d(103, 20, 2, 0)}),
    lanelet::LineString3d(104, {lanelet::Point3d(105, 0, -2, 0), lanelet::Point3d(106, 20, -2, 0)}));
  road.attributes()["subtype"] = "road";
  road.attributes()["location"] = "urban";
  lanelet::Lanelet crossing(200,
    lanelet::LineString3d(201, {lanelet::Point3d(202, 9, -5, 0), lanelet::Point3d(203, 9, 5, 0)}),
    lanelet::LineString3d(204, {lanelet::Point3d(205, 11, -5, 0), lanelet::Point3d(206, 11, 5, 0)}));
  crossing.attributes()["subtype"] = "crosswalk";
  crossing.attributes()["location"] = "urban";
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road); map->add(crossing);
  autoware::route_handler::RouteHandler handler(map);
  const auto graphs = handler.getOverallGraphPtr();
  ASSERT_NE(graphs, nullptr);
  ASSERT_FALSE(graphs->conflictingInGraph(road, 1).empty());
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 2.0;
  EXPECT_NEAR(autoware::behavior_path_planner::utils::getDistanceToCrosswalk(
    pose, {road}, *graphs), 7.0, 1e-6);
  pose.position.x = 12.0;
  EXPECT_TRUE(std::isinf(autoware::behavior_path_planner::utils::getDistanceToCrosswalk(
    pose, {road}, *graphs)));
  auto clear_map = std::make_shared<lanelet::LaneletMap>();
  clear_map->add(road);
  autoware::route_handler::RouteHandler clear_handler(clear_map);
  pose.position.x = 2.0;
  EXPECT_TRUE(std::isinf(autoware::behavior_path_planner::utils::getDistanceToCrosswalk(
    pose, {road}, *clear_handler.getOverallGraphPtr())));
}
}  // namespace
