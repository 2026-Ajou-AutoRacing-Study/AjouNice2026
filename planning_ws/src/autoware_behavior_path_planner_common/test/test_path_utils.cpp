// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
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

PathWithLaneId sparseCenterline(const double length)
{
  PathWithLaneId path;
  // A long straight and a short lateral seam, followed by a shallow bend.
  // These are native map knots, not an already-resampled planner output.
  const std::vector<std::pair<double, double>> xy{
    {-45.0, 0.0}, {0.0, 0.0}, {length, 0.0}, {length, 0.09},
    {length + 0.5, 0.04}, {length + 1.0, -0.01}, {length + 2.0, -0.11},
    {length + 10.0, -0.91}};
  for (size_t i = 0; i < xy.size(); ++i) {
    PathPointWithLaneId p;
    p.point.pose.position.x = xy[i].first;
    p.point.pose.position.y = xy[i].second;
    p.point.pose.orientation.w = 1.0;
    p.point.longitudinal_velocity_mps = 8.0;
    p.lane_ids = {i < 3 ? 100 : 200};
    path.points.push_back(p);
  }
  path.header.frame_id = "map";
  return path;
}

autoware::behavior_path_planner::BehaviorPathPlannerParameters centerlineParameters()
{
  autoware::behavior_path_planner::BehaviorPathPlannerParameters p;
  p.input_path_interval = 1.0;
  p.ego_nearest_dist_threshold = 3.0;
  p.ego_nearest_yaw_threshold = 1.0;
  p.enable_akima_spline_first = false;
  return p;
}

TEST(BehaviorPathPlanningPathUtilTest, SparseCenterlineCropDoesNotChangeSplineSupport)
{
  using autoware::behavior_path_planner::utils::resampleCenterLinePathWithSupport;
  // Removing the native point behind the moving window reproduced a >5 m
  // bow on a 75 m straight. Check other spacings and both sides of the seam.
  for (const double length : {20.0, 75.0, 250.0}) {
    const auto source = sparseCenterline(length);
    auto old_cropped = source;
    old_cropped.points.erase(old_cropped.points.begin());
    const auto legacy = autoware::motion_utils::resamplePath(old_cropped, 1.0);
    double legacy_bow = 0.0;
    for (const auto & p : legacy.points) {
      if (p.point.pose.position.x < length - 1.0)
        legacy_bow = std::max(legacy_bow, std::abs(p.point.pose.position.y));
    }
    EXPECT_GT(legacy_bow, length * 0.05);

    for (const double crop_start : {-0.20, -0.01, 0.01, 0.20}) {
      geometry_msgs::Pose ego;
      ego.orientation.w = 1.0;
      ego.position.x = 5.0 + crop_start;
      const auto result = resampleCenterLinePathWithSupport(
        source, ego, 5.0, length + 5.0, centerlineParameters());
      ASSERT_GT(result.points.size(), 2U);
      EXPECT_EQ(result.header.frame_id, "map");
      for (const auto & p : result.points) {
        if (p.point.pose.position.x >= 0.0 && p.point.pose.position.x < length - 1.0) {
          EXPECT_NEAR(p.point.pose.position.y, 0.0, 1e-6);
          EXPECT_EQ(p.lane_ids.front(), 100);
        }
      }
      // The real bend after the seam must remain; do not flatten the map.
      EXPECT_LT(result.points.back().point.pose.position.y, -0.2);
    }
  }
}

TEST(BehaviorPathPlanningPathUtilTest, SparseStraightDoesNotAnticipateFollowingBend)
{
  // v98 removed the lateral seam: a 75 m straight now connects directly to a
  // dense shallow bend. Native support alone leaves a >1 m Akima overshoot.
  for (const double length : {20.0, 75.0, 250.0}) {
    for (const double rotation : {0.0, -1.57, 2.4}) {
      PathWithLaneId source;
      const std::vector<std::pair<double, double>> xy{
        {-45., -.014}, {0., 0.}, {length, 0.},
        {length + .5, -.05}, {length + 1., -.10}, {length + 1.5, -.15},
        {length + 10., -1.}};
      for (size_t i = 0; i < xy.size(); ++i) {
        PathPointWithLaneId point;
        point.point.pose.position.x = 200. + std::cos(rotation) * xy[i].first -
          std::sin(rotation) * xy[i].second;
        point.point.pose.position.y = 400. + std::sin(rotation) * xy[i].first +
          std::cos(rotation) * xy[i].second;
        point.point.pose.orientation.z = std::sin(rotation / 2.);
        point.point.pose.orientation.w = std::cos(rotation / 2.);
        point.point.longitudinal_velocity_mps = 8.;
        point.lane_ids = {i < 2 ? 100 : 200};
        if (i == 2) point.lane_ids = {100, 200};
        source.points.push_back(point);
      }
      const auto legacy = autoware::motion_utils::resamplePath(source, .1, false);
      double legacy_bow = 0.;
      for (const auto & p : legacy.points) {
        const double dx = p.point.pose.position.x - 200.;
        const double dy = p.point.pose.position.y - 400.;
        const double forward = std::cos(rotation) * dx + std::sin(rotation) * dy;
        if (forward > 0. && forward < length)
          legacy_bow = std::max(legacy_bow, -std::sin(rotation) * dx + std::cos(rotation) * dy);
      }
      EXPECT_GT(legacy_bow, .01 * length);
      for (const double crop_start : {0., .2, 5., 5.2}) {
        geometry_msgs::Pose ego;
        ego.position.x = 200. + std::cos(rotation) * crop_start;
        ego.position.y = 400. + std::sin(rotation) * crop_start;
        ego.orientation.z = std::sin(rotation / 2.);
        ego.orientation.w = std::cos(rotation / 2.);
        auto params = centerlineParameters();
        params.input_path_interval = .1;
        const auto result = autoware::behavior_path_planner::utils::resampleCenterLinePathWithSupport(
          source, ego, 1., length + 12., params);
        ASSERT_GT(result.points.size(), 2U);
        for (const auto & p : result.points) {
          const double dx = p.point.pose.position.x - 200.;
          const double dy = p.point.pose.position.y - 400.;
          const double forward = std::cos(rotation) * dx + std::sin(rotation) * dy;
          const double lateral = -std::sin(rotation) * dx + std::cos(rotation) * dy;
          if (forward > 0. && forward < length) EXPECT_LT(std::abs(lateral), .01);
        }
        EXPECT_NEAR(
          result.points.back().point.pose.position.x, source.points.back().point.pose.position.x, 1e-6);
        EXPECT_NEAR(
          result.points.back().point.pose.position.y, source.points.back().point.pose.position.y, 1e-6);
      }
    }
  }
}

TEST(BehaviorPathPlanningPathUtilTest, SupportSubdivisionPreservesFineDetourAndStop)
{
  auto source = makePath(5, 1.0);
  const std::vector<std::pair<double, double>> xy{
    {0., 0.}, {1.3, 0.}, {1.3, .2}, {1.55, .4}, {2., .4}};
  for (size_t i = 0; i < source.points.size(); ++i) {
    source.points[i].point.pose.position.x = xy[i].first;
    source.points[i].point.pose.position.y = xy[i].second;
    source.points[i].lane_ids = i < 2 ? std::vector<int64_t>{100} : std::vector<int64_t>{200};
    if (i == 2) source.points[i].lane_ids = {100, 200};
    if (i >= 2) source.points[i].point.longitudinal_velocity_mps = 0.;
  }
  geometry_msgs::Pose ego;
  ego.orientation.w = 1.;
  for (const bool linear : {false, true}) {
    auto params = centerlineParameters();
    params.enable_akima_spline_first = linear;
    params.input_path_interval = .01;
    const auto out = autoware::behavior_path_planner::utils::resampleCenterLinePathWithSupport(
      source, ego, 1., 10., params);
    ASSERT_GT(out.points.size(), 2U);
    const auto stop = std::find_if(out.points.begin(), out.points.end(), [](const auto & p) {
      return std::hypot(p.point.pose.position.x - 1.3, p.point.pose.position.y - .2) < 1e-6;
    });
    ASSERT_NE(stop, out.points.end());
    EXPECT_EQ(stop->lane_ids, (std::vector<int64_t>{100, 200}));
    for (auto p = stop; p != out.points.end(); ++p)
      EXPECT_DOUBLE_EQ(p->point.longitudinal_velocity_mps, 0.);
    EXPECT_NEAR(out.points.back().point.pose.position.x, 2., 1e-6);
    EXPECT_NEAR(out.points.back().point.pose.position.y, .4, 1e-6);
  }
}

TEST(BehaviorPathPlanningPathUtilTest, SupportedWindowPreservesStopLaneIdsAndTerminal)
{
  using autoware::behavior_path_planner::utils::resampleCenterLinePathWithSupport;
  auto source = makePath(6, 3.3);
  for (size_t i = 0; i < source.points.size(); ++i) {
    source.points[i].lane_ids = i < 3 ? std::vector<int64_t>{100} : std::vector<int64_t>{200};
    if (i >= 3) source.points[i].point.longitudinal_velocity_mps = 0.0;
  }
  source.points[3].lane_ids = {100, 200};
  geometry_msgs::Pose ego;
  ego.orientation.w = 1.0;
  ego.position.x = 5.0;
  const auto result = resampleCenterLinePathWithSupport(
    source, ego, 2.0, 100.0, centerlineParameters());
  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.front().point.pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(result.points.back().point.pose.position.x, 16.5, 1e-6);
  const auto stop = std::find_if(result.points.begin(), result.points.end(), [](const auto & p) {
    return std::abs(p.point.pose.position.x - 9.9) < 1e-6;
  });
  ASSERT_NE(stop, result.points.end());
  EXPECT_DOUBLE_EQ(stop->point.longitudinal_velocity_mps, 0.0);
  EXPECT_EQ(stop->lane_ids.size(), 2U);
}

TEST(BehaviorPathPlanningPathUtilTest, SupportedWindowHandlesRouteStartAndInvalidInput)
{
  using autoware::behavior_path_planner::utils::resampleCenterLinePathWithSupport;
  geometry_msgs::Pose ego;
  ego.orientation.w = 1.0;
  auto params = centerlineParameters();
  const auto result = resampleCenterLinePathWithSupport(makePath(5, 2.0), ego, 10., 3., params);
  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.front().point.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(result.points.back().point.pose.position.x, 3.0, 1e-6);
  params.input_path_interval = 0.0;
  EXPECT_THROW(resampleCenterLinePathWithSupport(makePath(5, 2.0), ego, 1., 3., params),
    std::invalid_argument);
  params = centerlineParameters();
  auto invalid = makePath(5, 2.0);
  invalid.points[2].point.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(resampleCenterLinePathWithSupport(invalid, ego, 1., 3., params),
    std::invalid_argument);
}

TEST(BehaviorPathPlanningPathUtilTest, NativeSupportSurvivesCurrentLaneChange)
{
  // The distance-selected lanelet sequence itself can move past a sparse
  // predecessor. Exercise topology extension, not only the resampling helper.
  auto map = std::make_shared<lanelet::LaneletMap>();
  const std::vector<double> x{-90., -45., 0., 75., 100.};
  std::vector<lanelet::Point3d> left, right;
  for (size_t i = 0; i < x.size(); ++i) {
    left.emplace_back(1000 + 2 * i, x[i], 2., 0.);
    right.emplace_back(1001 + 2 * i, x[i], -2., 0.);
  }
  lanelet::ConstLanelets lanes;
  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = -80.;
  route.start_pose.orientation.w = 1.;
  route.goal_pose.position.x = 99.;
  route.goal_pose.orientation.w = 1.;
  for (size_t i = 0; i + 1 < x.size(); ++i) {
    lanelet::Lanelet lane(100 + i,
      lanelet::LineString3d(2000 + 2 * i, {left[i], left[i + 1]}),
      lanelet::LineString3d(2001 + 2 * i, {right[i], right[i + 1]}));
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["one_way"] = "yes";
    lane.attributes()["speed_limit"] = "40";
    lanelet::Points3d knots;
    if (i < 3) {
      knots = {lanelet::Point3d(4000 + 10 * i, x[i], 0., 0.),
        lanelet::Point3d(4001 + 10 * i, x[i + 1], 0., 0.)};
    } else {
      const std::vector<std::pair<double, double>> xy{
        {75., .09}, {75.5, .04}, {76., -.01}, {77., -.11}, {85., -.91}, {100., 0.}};
      for (size_t j = 0; j < xy.size(); ++j)
        knots.emplace_back(4030 + j, xy[j].first, xy[j].second, 0.);
    }
    lanelet::LineString3d center(3000 + i, knots);
    lane.setCenterline(center);
    lane.attributes()["waypoints"] = std::to_string(center.id());
    map->add(center);
    map->add(lane);
    lanes.push_back(lane);
    autoware_planning_msgs::LaneletSegment section;
    section.preferred_primitive.id = lane.id();
    section.preferred_primitive.primitive_type = "lane";
    section.primitives = {section.preferred_primitive};
    route.segments.push_back(section);
  }
  autoware::route_handler::RouteHandler handler(map);
  handler.setRoute(route);
  ASSERT_TRUE(handler.isHandlerReady());
  for (const auto & sample : std::vector<std::pair<size_t, double>>{{2, 74.5}, {3, 75.5}}) {
    geometry_msgs::Pose ego;
    ego.orientation.w = 1.;
    ego.position.x = sample.second;
    const lanelet::ConstLanelets selected = sample.first == 2
      ? lanelet::ConstLanelets{lanes[1], lanes[2], lanes[3]}
      : lanelet::ConstLanelets{lanes[2], lanes[3]};
    const auto result = autoware::behavior_path_planner::utils::getCenterLinePath(
      handler, selected, ego, 20., 30., centerlineParameters());
    ASSERT_FALSE(result.points.empty());
    EXPECT_LT(result.points.front().point.pose.position.x, 56.);
    EXPECT_LE(result.points.back().point.pose.position.x, 99.1);  // route goal
    for (const auto & p : result.points) {
      if (p.point.pose.position.x > 56. && p.point.pose.position.x < 74.)
        EXPECT_NEAR(p.point.pose.position.y, 0., 1e-6);
    }
  }
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
