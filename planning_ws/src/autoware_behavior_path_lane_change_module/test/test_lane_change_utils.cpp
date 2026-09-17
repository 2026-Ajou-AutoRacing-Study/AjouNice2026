// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

constexpr double epsilon = 1e-6;

TEST(BehaviorPathPlanningLaneChangeUtilsTest, RejectTruncatedOrReversedManeuver)
{
  using autoware::behavior_path_planner::LaneChangePath;
  using autoware::behavior_path_planner::utils::lane_change::contains_lane_change_maneuver;
  for (const double direction : {-1.0, 1.0}) {
    LaneChangePath candidate;
    for (int i = 0; i <= 20; ++i) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose.position.x = i * 5.0;
      point.point.pose.position.y = direction * i * 0.2;
      candidate.path.points.push_back(point);
    }
    candidate.info.lane_changing_start = candidate.path.points[4].point.pose;
    candidate.info.lane_changing_end = candidate.path.points[16].point.pose;
    EXPECT_TRUE(contains_lane_change_maneuver(candidate));
    candidate.shifted_path.path = candidate.path;
    const auto check_path =
      autoware::behavior_path_planner::utils::lane_change::get_maneuver_check_path(candidate, 9.0);
    EXPECT_EQ(check_path.points.size(), 19U);
    EXPECT_EQ(candidate.path.points.size(), 21U);  // publication is never cropped here
    auto truncated = candidate;
    truncated.path.points.resize(12);
    EXPECT_FALSE(contains_lane_change_maneuver(truncated));
    std::swap(candidate.info.lane_changing_start, candidate.info.lane_changing_end);
    EXPECT_FALSE(contains_lane_change_maneuver(candidate));
    candidate.path.points.clear();
    EXPECT_FALSE(contains_lane_change_maneuver(candidate));
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, LaneSequenceChangeInvalidatesCachedGeometry)
{
  using namespace autoware::behavior_path_planner;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lanes_ptr = std::make_shared<lane_change::Lanes>();
  const lanelet::ConstLanelet a(lanelet::Lanelet(1)), b(lanelet::Lanelet(2));
  const lanelet::ConstLanelet c(lanelet::Lanelet(3)), d(lanelet::Lanelet(4));
  data->lanes_ptr->current = {a, b};
  data->lanes_ptr->target = {c, d};
  const auto same = [&](const lanelet::ConstLanelets & current, const lanelet::ConstLanelets & target) {
    return utils::lane_change::is_same_lane_with_prev_iteration(data, current, target);
  };
  EXPECT_TRUE(same({a, b}, {c, d}));
  EXPECT_FALSE(same({a, c}, {c, d}));
  EXPECT_FALSE(same({a, b}, {c, a}));
  EXPECT_FALSE(same({a, d, b}, {c, d}));
  EXPECT_FALSE(same({}, {c, d}));
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, LocalBoundaryCheckStillRejectsUnsafeLeftAndRightShift)
{
  using namespace autoware::behavior_path_planner;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lanes_ptr = std::make_shared<lane_change::Lanes>();
  lanelet::LineString3d left(11, {lanelet::Point3d(1, 0, 2, 0), lanelet::Point3d(2, 100, 2, 0)});
  lanelet::LineString3d right(12, {lanelet::Point3d(3, 0, -2, 0), lanelet::Point3d(4, 100, -2, 0)});
  data->lanes_ptr->target = {lanelet::Lanelet(20, left, right)};
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  vehicle.wheel_base_m = 2.0;
  vehicle.front_overhang_m = 1.0;
  for (const double side : {-1.0, 1.0}) {
    data->direction = side > 0 ? autoware::route_handler::Direction::LEFT
                               : autoware::route_handler::Direction::RIGHT;
    LaneChangePath candidate;
    for (int i = 0; i <= 20; ++i) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose.position.x = i * 5.0;
      point.point.pose.orientation.w = 1.0;
      candidate.shifted_path.path.points.push_back(point);
    }
    candidate.info.lane_changing_end = candidate.shifted_path.path.points[10].point.pose;
    // An unchanged far continuation is not the LC maneuver's overshoot check.
    candidate.shifted_path.path.points[18].point.pose.position.y = side * 2.0;
    auto check = utils::lane_change::get_maneuver_check_path(candidate, 5.0);
    EXPECT_FALSE(utils::lane_change::path_footprint_exceeds_target_lane_bound(data, check, vehicle));
    EXPECT_TRUE(utils::lane_change::path_footprint_exceeds_target_lane_bound(
      data, candidate.shifted_path.path, vehicle));
    // The identical violation during the maneuver is still rejected.
    candidate.shifted_path.path.points[8].point.pose.position.y = side * 2.0;
    check = utils::lane_change::get_maneuver_check_path(candidate, 5.0);
    EXPECT_TRUE(utils::lane_change::path_footprint_exceeds_target_lane_bound(data, check, vehicle));
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, MergeDoesNotAppendSharedTailTwice)
{
  using autoware::behavior_path_planner::utils::lane_change::generateDrivableLanes;
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id id = 1000;
  const auto bound = [&](double x0, double y0, double x1, double y1) {
    lanelet::LineString3d line(++id, {
      lanelet::Point3d(++id, x0, y0, 0), lanelet::Point3d(++id, x1, y1, 0)});
    line.attributes()["type"] = "line_thin";
    line.attributes()["subtype"] = "dashed";
    return line;
  };
  const auto lane = [&](lanelet::Id lane_id, const lanelet::LineString3d & left,
                        const lanelet::LineString3d & right) {
    lanelet::Lanelet result(lane_id, left, right);
    result.attributes()["subtype"] = "road";
    result.attributes()["location"] = "urban";
    result.attributes()["one_way"] = "yes";
    map->add(result);
    return result;
  };
  const auto divider = bound(0, 0, 10, 0);
  const auto current = lane(1, divider, bound(0, -3, 10, -3));
  const auto target = lane(2, bound(0, 3, 10, 3), divider);
  const auto current_merge = lane(3, bound(10, 0, 20, 1.5), bound(10, -3, 20, -1.5));
  const auto target_merge = lane(4, bound(10, 3, 20, 1.5), bound(10, 0, 20, -1.5));
  const auto common = lane(5, bound(20, 1.5, 30, 1.5), bound(20, -1.5, 30, -1.5));
  const auto tail = lane(6, bound(30, 1.5, 40, 1.5), bound(30, -1.5, 40, -1.5));
  autoware::route_handler::RouteHandler handler;
  handler.setMap(map);
  const auto groups = generateDrivableLanes(
    handler, {current, current_merge, common}, {target, target_merge, common, tail});
  ASSERT_EQ(groups.size(), 4U);
  EXPECT_EQ(groups[0].left_lane.id(), 2);
  EXPECT_EQ(groups[0].right_lane.id(), 1);
  EXPECT_EQ(groups[1].left_lane.id(), 4);
  EXPECT_EQ(groups[1].right_lane.id(), 3);
  EXPECT_EQ(groups[2].left_lane.id(), 5);
  EXPECT_EQ(groups[3].left_lane.id(), 6);
  EXPECT_FALSE(autoware::behavior_path_planner::utils::getOverlappedLaneletId(groups));
  const auto loop = std::vector<autoware::behavior_path_planner::DrivableLanes>{
    groups[0], groups[1], groups[0]};
  const auto overlap = autoware::behavior_path_planner::utils::getOverlappedLaneletId(loop);
  ASSERT_TRUE(overlap);
  EXPECT_EQ(*overlap, 2U);
  const auto reverse = generateDrivableLanes(
    handler, {target, target_merge, common}, {current, current_merge, common, tail});
  ASSERT_EQ(reverse.size(), 4U);
  EXPECT_EQ(reverse[1].left_lane.id(), 4);
  EXPECT_EQ(reverse[1].right_lane.id(), 3);
  const auto shared = generateDrivableLanes(handler, {common, tail}, {common, tail});
  ASSERT_EQ(shared.size(), 2U);
  const auto empty = generateDrivableLanes(handler, {}, {common, tail});
  ASSERT_EQ(empty.size(), 2U);
  // The common successor need not be the next lanelet: both branches can
  // overlap over multiple segments without any routable lateral edge.
  const auto current_merge_two = lane(7, bound(20, 1.5, 30, 1.5), bound(20, -1.5, 30, -1.5));
  const auto target_merge_two = lane(8, bound(20, 1.5, 30, 1.5), bound(20, -1.5, 30, -1.5));
  handler.setMap(map);
  const auto multi = generateDrivableLanes(
    handler, {current, current_merge, current_merge_two, common},
    {target, target_merge, target_merge_two, common, tail});
  ASSERT_EQ(multi.size(), 5U);
  EXPECT_EQ(multi[1].left_lane.id(), 4);
  EXPECT_EQ(multi[1].right_lane.id(), 3);
  EXPECT_EQ(multi[2].left_lane.id(), 8);
  EXPECT_EQ(multi[2].right_lane.id(), 7);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, LongitudinalForkFollowsRoutePreference)
{
  for (const double side : {-1.0, 1.0}) {
    auto map = std::make_shared<lanelet::LaneletMap>();
    lanelet::Id id = 1000;
    const auto point = [&](double x, double y) { return lanelet::Point3d(++id, x, y, 0); };
    const auto a = point(0, 2), b = point(0, -2);
    const auto c = point(10, 2), d = point(10, -2);
    const auto e = point(20, 2), f = point(20, -2);
    const auto g = point(20, 2 + side * 4), h = point(20, -2 + side * 4);
    const auto i = point(30, 2 + side * 4), j = point(30, -2 + side * 4);
    const auto make_lane = [&](lanelet::Id lane_id, const lanelet::Point3d & l0,
      const lanelet::Point3d & l1, const lanelet::Point3d & r0, const lanelet::Point3d & r1) {
      lanelet::LineString3d left(++id, {l0, l1}), right(++id, {r0, r1});
      left.attributes()["type"] = right.attributes()["type"] = "line_thin";
      left.attributes()["subtype"] = right.attributes()["subtype"] = "dashed";
      lanelet::Lanelet lane(lane_id, left, right);
      lane.attributes()["subtype"] = "road";
      lane.attributes()["location"] = "urban";
      lane.attributes()["one_way"] = "yes";
      map->add(lane);
      return lane;
    };
    const auto current = make_lane(1, a, c, b, d);
    make_lane(2, c, e, d, f);
    make_lane(3, c, g, d, h);
    make_lane(4, g, i, h, j);
    autoware::route_handler::RouteHandler handler;
    handler.setMap(map);
    autoware_planning_msgs::LaneletRoute route;
    route.segments.resize(3);
    const auto primitive = [](int64_t lane_id) {
      autoware_planning_msgs::LaneletPrimitive p;
      p.id = lane_id; p.primitive_type = "lane";
      return p;
    };
    route.segments[0].primitives = {primitive(1)};
    route.segments[0].preferred_primitive = primitive(1);
    route.segments[1].primitives = {primitive(2), primitive(3)};
    route.segments[1].preferred_primitive = primitive(3);
    route.segments[2].primitives = {primitive(4)};
    route.segments[2].preferred_primitive = primitive(4);
    route.goal_pose.position.x = 29;
    route.goal_pose.position.y = side * 4;
    route.goal_pose.orientation.w = route.start_pose.orientation.w = 1;
    handler.setRoute(route);
    lanelet::ConstLanelets successors;
    ASSERT_TRUE(handler.getNextLaneletsWithinRoute(current, &successors));
    ASSERT_EQ(successors.size(), 2U);
    lanelet::ConstLanelet selected;
    ASSERT_TRUE(handler.getNextLaneletWithinRoute(current, &selected));
    EXPECT_EQ(selected.id(), 3);
    geometry_msgs::Pose overlap_pose;
    overlap_pose.position.x = 11.0;
    overlap_pose.orientation.w = 1.0;
    ASSERT_TRUE(handler.getClosestRouteLaneletFromLanelet(
      overlap_pose, current, &selected, 3.0, 1.0));
    EXPECT_EQ(selected.id(), 3);
    // Outside the preferred successor polygon, preserve the actual branch.
    overlap_pose.position.x = 19.0;
    ASSERT_TRUE(handler.getClosestRouteLaneletFromLanelet(
      overlap_pose, current, &selected, 3.0, 1.0));
    EXPECT_EQ(selected.id(), 2);
    // A future preferred lane is not a direct successor and must not be selected.
    route.segments[1].preferred_primitive = primitive(2);
    handler.setRoute(route);
    ASSERT_TRUE(handler.getNextLaneletWithinRoute(current, &selected));
    EXPECT_EQ(selected.id(), 2);
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, projectCurrentPoseToTarget)
{
  geometry_msgs::msg::Pose ego_pose;
  const auto ego_yaw = autoware_utils::deg2rad(0.0);
  ego_pose.orientation = autoware_utils::create_quaternion_from_yaw(ego_yaw);
  ego_pose.position = autoware_utils::create_point(0, 0, 0);

  geometry_msgs::msg::Pose obj_pose;
  const auto obj_yaw = autoware_utils::deg2rad(0.0);
  obj_pose.orientation = autoware_utils::create_quaternion_from_yaw(obj_yaw);
  obj_pose.position = autoware_utils::create_point(-4, 3, 0);

  const auto result = autoware_utils::inverse_transform_pose(obj_pose, ego_pose);

  EXPECT_NEAR(result.position.x, -4, epsilon);
  EXPECT_NEAR(result.position.y, 3, epsilon);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, TESTLateralAccelerationMap)
{
  autoware::behavior_path_planner::LateralAccelerationMap lat_acc_map;
  lat_acc_map.add(0.0, 0.2, 0.315);
  lat_acc_map.add(3.0, 0.2, 0.315);
  lat_acc_map.add(5.0, 0.2, 0.315);
  lat_acc_map.add(6.0, 0.315, 0.40);
  lat_acc_map.add(10.0, 0.315, 0.50);

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(-1.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(0.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(1.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(3.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(5.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(5.5);
    EXPECT_NEAR(min_acc, 0.2575, epsilon);
    EXPECT_NEAR(max_acc, 0.3575, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(6.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.4, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(8.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.45, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(10.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.50, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(11.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.50, epsilon);
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, testExcludeLanelets)
{
  const auto create_lane = [](lanelet::Id id) {
    lanelet::Lanelet ll(id, lanelet::LineString3d{}, lanelet::LineString3d{});
    return lanelet::ConstLanelet{ll};
  };
  lanelet::ConstLanelets alternative{
    create_lane(1), create_lane(2), create_lane(3), create_lane(4)};
  lanelet::ConstLanelets preferred{create_lane(1), create_lane(2), create_lane(4)};

  // | P1 | P2 | A3 | P4 |
  // we want  to remove any preferred lanes after A3
  autoware::behavior_path_planner::utils::lane_change::trim_preferred_after_alternative(
    alternative, preferred);

  ASSERT_EQ(alternative.size(), 3u);
  EXPECT_EQ(alternative[0].id(), 1);
  EXPECT_EQ(alternative[1].id(), 2);
  EXPECT_EQ(alternative[2].id(), 3);
}
