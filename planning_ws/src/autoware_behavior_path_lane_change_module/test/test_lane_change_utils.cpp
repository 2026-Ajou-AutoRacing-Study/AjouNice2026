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
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/prepare_candidate_selection.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_lane_change_module/utils/speed_profile.hpp"
#include "autoware/behavior_path_lane_change_module/utils/speed_refresh.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <array>
#include <set>

constexpr double epsilon = 1e-6;

TEST(BehaviorPathPlanningLaneChangeUtilsTest, SafePrepareFallbackPreservesIndicesAndLastContract)
{
  using namespace autoware::behavior_path_planner::lane_change;
  SafePrepareCandidateSelection best;
  std::vector<int> paths{10};
  EXPECT_TRUE(best.consider(0,true,5.0));
  paths.push_back(20); // Later unsafe candidate must never become selected.
  EXPECT_FALSE(best.consider(1,false,0.0));
  EXPECT_TRUE(best.append_selected(paths)); // Timeout/exhaustion finalize.
  ASSERT_EQ(paths.size(),3U);
  EXPECT_EQ(paths[0],10);EXPECT_EQ(paths[1],20);EXPECT_EQ(paths.back(),10);
}
TEST(BehaviorPathPlanningLaneChangeUtilsTest, SafePrepareLowerLossAndZeroDuration)
{
  using namespace autoware::behavior_path_planner::lane_change;
  EXPECT_DOUBLE_EQ(prepare_speed_loss(16.1,0.0,11.0),0.0);
  EXPECT_NEAR(prepare_speed_loss(16.1,2.56,11.03),5.07,1e-9);
  EXPECT_DOUBLE_EQ(prepare_speed_loss(16.1,2.0,18.0),0.0);
  SafePrepareCandidateSelection best;std::vector<int> paths{10,20,30};
  EXPECT_TRUE(best.consider(0,true,5.07));
  EXPECT_TRUE(best.consider(1,true,1.0));
  EXPECT_TRUE(best.consider(2,true,0.0));
  EXPECT_TRUE(best.no_prepare_loss());
  EXPECT_TRUE(best.append_selected(paths));
  EXPECT_EQ(paths.size(),3U);EXPECT_EQ(paths.back(),30);
}
TEST(BehaviorPathPlanningLaneChangeUtilsTest, SafePrepareNoSafeAndInvalidRankFailSelectionClosed)
{
  using namespace autoware::behavior_path_planner::lane_change;
  SafePrepareCandidateSelection best;std::vector<int> paths{10,20};
  EXPECT_FALSE(best.consider(0,false,0.0));
  EXPECT_FALSE(best.append_selected(paths));
  EXPECT_TRUE(best.consider(0,true,std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(best.no_prepare_loss());
  EXPECT_TRUE(best.consider(1,true,2.0));
  EXPECT_TRUE(best.append_selected(paths));EXPECT_EQ(paths.back(),20);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, SafeZeroLossDoesNotHideFasterSafeCandidate)
{
  using namespace autoware::behavior_path_planner::lane_change;
  SafePrepareCandidateSelection best;std::vector<int> paths{10,20,30};
  EXPECT_TRUE(best.consider(0,true,0.0,48.0,3.0)); // R23 coast during preparation.
  EXPECT_TRUE(best.consider(1,true,0.0,57.0,0.0)); // Faster, equally loss-free.
  EXPECT_FALSE(best.consider(2,false,0.0,65.0,0.0));
  EXPECT_TRUE(best.append_selected(paths)); // Timeout preserves best, not latest unsafe.
  EXPECT_EQ(paths[0],10);EXPECT_EQ(paths[1],20);EXPECT_EQ(paths[2],30);
  EXPECT_EQ(paths.back(),20);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, SafeProgressTieUsesShorterPreparation)
{
  using namespace autoware::behavior_path_planner::lane_change;
  SafePrepareCandidateSelection best;std::vector<int> paths{10,20,30};
  EXPECT_TRUE(best.consider(0,true,0.0,48.0,3.0));
  EXPECT_TRUE(best.consider(1,true,0.0,48.0005,1.0));
  EXPECT_FALSE(best.consider(2,true,0.0,48.0008,2.0));
  EXPECT_TRUE(best.append_selected(paths));EXPECT_EQ(paths.back(),20);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, SafeRankingFailureRetainsFallbackAndNoExtraLoss)
{
  using namespace autoware::behavior_path_planner::lane_change;
  SafePrepareCandidateSelection best;std::vector<int> paths{10,20,30};
  EXPECT_TRUE(best.consider(0,true,0.0,-1.0,3.0)); // Scoring failed; safety did not.
  EXPECT_FALSE(best.consider(1,true,2.0,60.0,0.0)); // Do not buy progress with extra loss.
  EXPECT_TRUE(best.consider(2,true,0.0,49.0,2.0));
  EXPECT_TRUE(best.append_selected(paths));EXPECT_EQ(paths.back(),30);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, ApprovedSpeedRefreshPreservesGeometryStopsAndBounds)
{
  using namespace autoware::behavior_path_planner;
  PathWithLaneId original;
  for (int i = 0; i <= 120; ++i) {
    PathPointWithLaneId p;
    p.point.pose.position.x = i;
    p.point.pose.orientation.w = 1;
    p.point.longitudinal_velocity_mps = i >= 100 ? 0.0 : 10.0;
    p.lane_ids = {10, 20};
    original.points.push_back(p);
  }
  const auto refreshed = utils::lane_change::refresh_path_speed(
    original, 10, 90, std::vector<double>(81, 25.0), 10.0, 3.0, 4.0);
  EXPECT_GT(refreshed.points[50].point.longitudinal_velocity_mps, 15.0);
  for (size_t i = 0; i < original.points.size(); ++i) {
    EXPECT_EQ(refreshed.points[i].point.pose, original.points[i].point.pose);
    EXPECT_EQ(refreshed.points[i].lane_ids, original.points[i].lane_ids);
    if (i <= 10 || i >= 90) EXPECT_EQ(
      refreshed.points[i].point.longitudinal_velocity_mps,
      original.points[i].point.longitudinal_velocity_mps);
    if (i > 10 && i <= 90) {
      const double a = (std::pow(refreshed.points[i].point.longitudinal_velocity_mps, 2) -
        std::pow(refreshed.points[i-1].point.longitudinal_velocity_mps, 2)) / 2;
      EXPECT_LE(a, 3.0 + 1e-4);
      EXPECT_GE(a, -4.0 - 1e-4);
    }
  }
  auto caps = std::vector<double>(101, 25.0);
  caps[70] = 8.0;
  const auto stopped = utils::lane_change::refresh_path_speed(
    original, 10, 110, caps, 10.0, 3.0, 4.0);
  EXPECT_LE(stopped.points[80].point.longitudinal_velocity_mps, 8.0);
  for (size_t i = 100; i <= 120; ++i) EXPECT_EQ(
    stopped.points[i].point.longitudinal_velocity_mps, 0.0);
  EXPECT_THROW(utils::lane_change::refresh_path_speed(
    original, 10, 90, std::vector<double>(81, 1.0), 10.0, 3.0, 4.0), std::logic_error);
  // Failed proposals have never mutated the original approved path.
  EXPECT_EQ(original.points[50].point.longitudinal_velocity_mps, 10.0);
  const autoware::motion_utils::AccelerationLimits map({{0.0, 1.0}, {40.0, 1.0}});
  const auto map_limited = utils::lane_change::refresh_path_speed(
    original, 10, 90, std::vector<double>(81, 25.0), 10.0, 3.7, 4.0, map);
  for (size_t i = 11; i <= 90; ++i) {
    const double a = (std::pow(map_limited.points[i].point.longitudinal_velocity_mps, 2) -
      std::pow(map_limited.points[i-1].point.longitudinal_velocity_mps, 2)) / 2;
    EXPECT_LE(a, 1.0 + 1e-4);
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, ApprovedSpeedRefreshRemovesEndValleyWithoutRaisingTail)
{
  using namespace autoware::behavior_path_planner;
  using namespace utils::lane_change;
  PathWithLaneId original;
  std::vector<double> caps;
  constexpr size_t lc_end = 30;
  for (size_t i = 0; i <= 120; ++i) {
    PathPointWithLaneId p;
    p.point.pose.position.x = i;
    p.point.pose.orientation.w = 1.0;
    p.point.longitudinal_velocity_mps = i <= lc_end ? 10.0 : (i >= 100 ? 0.0 : 25.0);
    p.lane_ids = i <= lc_end ? std::vector<int64_t>{1, 2} : std::vector<int64_t>{2};
    original.points.push_back(p);
    caps.push_back(i <= lc_end ? 25.0 : p.point.longitudinal_velocity_mps);
  }
  const auto refreshed = refresh_path_speed(original, 0, 120, caps, 10.0, 3.0, 4.0);
  EXPECT_GT(refreshed.points[lc_end].point.longitudinal_velocity_mps, 16.0);
  EXPECT_GT(refreshed.points[lc_end + 1].point.longitudinal_velocity_mps,
            refreshed.points[lc_end].point.longitudinal_velocity_mps);
  for (size_t i = lc_end + 1; i < original.points.size(); ++i) {
    EXPECT_LE(refreshed.points[i].point.longitudinal_velocity_mps,
              original.points[i].point.longitudinal_velocity_mps);
    EXPECT_EQ(refreshed.points[i].point.pose, original.points[i].point.pose);
    EXPECT_EQ(refreshed.points[i].lane_ids, original.points[i].lane_ids);
  }
  const auto lc = refresh_lc_metrics(refreshed, 0, lc_end);
  const auto tail = refresh_lc_metrics(refreshed, 0, 100);
  EXPECT_NEAR(lc.terminal_velocity, refreshed.points[lc_end].point.longitudinal_velocity_mps, 1e-6);
  EXPECT_GT(lc.terminal_velocity, 10.0);
  EXPECT_LT(lc.duration, tail.duration);
  EXPECT_EQ(tail.terminal_velocity, 0.0);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, ApprovedSpeedRefreshFinalOutputAtomicCommit)
{
  using namespace autoware::behavior_path_planner;
  using utils::lane_change::commit_final_speed_refresh;
  LaneChangePath approved;
  for (int i = 0; i < 4; ++i) {
    PathPointWithLaneId p;
    p.point.pose.position.x = i;
    p.point.pose.orientation.w = 1.0;
    p.point.longitudinal_velocity_mps = 10.0;
    p.lane_ids = {1};
    approved.path.points.push_back(p);
  }
  auto output = approved.path;
  output.points.back().point.longitudinal_velocity_mps = 0.0;  // Late terminal stop.
  auto proposal = approved;
  proposal.path = output;
  proposal.path.points[1].point.longitudinal_velocity_mps = 11.0;
  int checks = 0;
  const auto reject = [&](const LaneChangePath & final_path) {
    ++checks;
    EXPECT_EQ(final_path.path.points.back().point.longitudinal_velocity_mps, 0.0);
    return false;  // E.g. newly inserted stop fails trailing-object RSS.
  };
  EXPECT_FALSE(commit_final_speed_refresh(approved, output, proposal, false, reject));
  EXPECT_EQ(checks, 1);
  EXPECT_EQ(approved.path.points[1].point.longitudinal_velocity_mps, 10.0);
  EXPECT_EQ(output.points[1].point.longitudinal_velocity_mps, 10.0);
  EXPECT_EQ(output.points.back().point.longitudinal_velocity_mps, 0.0);
  EXPECT_FALSE(commit_final_speed_refresh(approved, output, proposal, true, reject));
  EXPECT_EQ(checks, 1);  // Shortened-output fallback cannot commit even if safe.
  const auto accept = [](const LaneChangePath &) { return true; };
  auto erased_stop = proposal;
  erased_stop.path.points.back().point.longitudinal_velocity_mps = 10.0;
  EXPECT_FALSE(commit_final_speed_refresh(approved, output, erased_stop, false, accept));
  auto moved_geometry = proposal;
  moved_geometry.path.points[1].point.pose.position.y = 1.0;
  EXPECT_FALSE(commit_final_speed_refresh(approved, output, moved_geometry, false, accept));
  EXPECT_THROW(commit_final_speed_refresh(approved, output, proposal, false,
    [](const LaneChangePath &) -> bool { throw std::logic_error("prediction failed"); }),
    std::logic_error);
  EXPECT_EQ(output.points[1].point.longitudinal_velocity_mps, 10.0);
  EXPECT_TRUE(commit_final_speed_refresh(approved, output, proposal, false, accept));
  EXPECT_EQ(approved.path, output);
  EXPECT_EQ(output.points[1].point.longitudinal_velocity_mps, 11.0);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, ApprovedSpeedRefreshRejectsIncompleteHypotheses)
{
  using namespace autoware::behavior_path_planner;
  using utils::lane_change::complete_refresh_predictions;
  struct Sample { double time; };
  std::vector<std::vector<Sample>> paths{{{0.0}, {1.0}}, {{0.0}, {1.0}}};
  EXPECT_TRUE(complete_refresh_predictions(paths));
  paths[1].resize(1);
  EXPECT_FALSE(complete_refresh_predictions(paths));
  paths[1] = {{0.0}, {0.0}};
  EXPECT_FALSE(complete_refresh_predictions(paths));
  paths[1] = {{0.0}, {1.0}, {0.5}};
  EXPECT_FALSE(complete_refresh_predictions(paths));
  paths.resize(1);
  EXPECT_FALSE(complete_refresh_predictions(paths));
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, SpatialSpeedProfileRespectsTransitionsAndDynamics)
{
  using namespace autoware::behavior_path_planner::utils::lane_change;
  const double entry = 59.0 / 3.6, highway = 150.0 / 3.6;
  std::vector<double> s, caps;
  for (int i = 0; i <= 150; ++i) {
    s.push_back(i);
    caps.push_back(i <= 30 || i >= 120 ? entry : highway);
  }
  const auto v = make_speed_profile(s, caps, entry, 3.7, 6.0);
  EXPECT_GT(v[70], entry + 5);
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_LE(v[i], caps[i] + epsilon);
    if (i) {
      const double a = (v[i] * v[i] - v[i - 1] * v[i - 1]) / 2;
      EXPECT_LE(a, 3.7 + epsilon);
      EXPECT_GE(a, -6.0 - epsilon);
    }
  }
  EXPECT_NEAR(sample_speed_profile(s, v, 1).first, entry, epsilon);
  EXPECT_NEAR(sample_speed_profile(s, v, 1).second, entry, epsilon);
  const auto end = sample_speed_profile(s, v, speed_profile_duration(s, v));
  EXPECT_NEAR(end.first, s.back(), epsilon);
  EXPECT_NEAR(end.second, v.back(), epsilon);
  EXPECT_THROW(make_speed_profile({0, 1}, {40, 1}, 30, 3.7, 6), std::logic_error);
  const auto constant = make_speed_profile(s, std::vector<double>(s.size(), entry), entry, 3.7, 6);
  for (const auto speed : constant) EXPECT_NEAR(speed, entry, epsilon);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, SpatialCollisionPredictionUsesOutputSpeedAndFullDuration)
{
  using namespace autoware::behavior_path_planner;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
  data->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
  data->lc_param_ptr->safety.collision_check.prediction_time_resolution = 0.5;
  data->bpp_param_ptr->ego_nearest_dist_threshold = 3;
  data->bpp_param_ptr->ego_nearest_yaw_threshold = 1;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.orientation.w = 1;
  odometry->twist.twist.linear.x = 10;
  data->self_odometry_ptr = odometry;
  LaneChangePath path;
  path.type = lane_change::PathType::SpatialSpeedProfile;
  std::vector<double> s, caps;
  for (int i = 0; i <= 100; ++i) {s.push_back(i); caps.push_back(i <= 20 ? 10 : 40);}
  const auto speeds = utils::lane_change::make_speed_profile(s, caps, 10, 3.7, 6);
  for (int i = 0; i <= 100; ++i) {
    PathPointWithLaneId p;
    p.point.pose.position.x = i;
    p.point.pose.orientation.w = 1;
    p.point.longitudinal_velocity_mps = speeds[i];
    path.path.points.push_back(p);
  }
  path.info.lane_changing_end = path.path.points.back().point.pose;
  // Deliberately stale nominal duration must not truncate the spatial prediction.
  path.info.duration = {0, 1};
  const auto predictions = utils::lane_change::convert_to_predicted_paths(data, path, 1, false);
  ASSERT_EQ(predictions.size(), 1U);
  EXPECT_NEAR(predictions[0].back().pose.position.x, 100, 1e-5);
  EXPECT_GT(predictions[0].back().time, 2);
  for (const auto & state : predictions[0]) {
    const auto expected = utils::lane_change::sample_speed_profile(s, speeds, state.time);
    EXPECT_NEAR(state.pose.position.x, expected.first, 1e-4);
    EXPECT_NEAR(state.velocity, expected.second, 1e-4);
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, CompetitionDiverseCandidateRankWithinShortBudget)
{
  using namespace autoware::behavior_path_planner::utils::lane_change::calculation;
  std::vector<PhaseMetrics> preparations;
  // Reproduce duration-major fine acceleration grid that previously consumed
  // all ten checks at duration=0 and lateral acceleration=1.
  for (const double duration : {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0}) {
    for (const double acceleration : {3.25, 2.8, 2.0, 1.0, 0.0, -0.013, -1.0, -2.0, -4.0}) {
      preparations.emplace_back(duration, 20.0 * duration, 20.0, acceleration,
        duration == 0.0 ? 0.0 : acceleration, 0.0);
    }
  }
  const auto order = diverse_phase_order(preparations, true);
  ASSERT_EQ(order.size(), preparations.size());
  EXPECT_EQ(std::set<size_t>(order.begin(), order.end()).size(), preparations.size());
  const std::array<double, 3> expected_acc{3.25, 0.0, -2.0};
  const std::array<double, 3> expected_lat{4.5, 2.75, 1.0};
  for (size_t rank = 0; rank < 6; ++rank) {
    const auto & prep = preparations[order[rank]];
    EXPECT_DOUBLE_EQ(prep.duration, rank < 3 ? 0.0 : 1.5);
    EXPECT_DOUBLE_EQ(prep.sampled_lon_accel, expected_acc[rank % 3]);
    std::vector<PhaseMetrics> execution;
    for (const double lat : {1.0, 1.5, 2.0, 2.75, 3.5, 4.0, 4.5}) {
      execution.emplace_back(3.0, 60.0, 20.0, prep.sampled_lon_accel,
        prep.sampled_lon_accel, lat);
    }
    const auto execution_order = diverse_phase_order(execution, false, rank);
    EXPECT_DOUBLE_EQ(execution[execution_order.front()].lat_accel, expected_lat[rank % 3]);
    EXPECT_EQ(std::set<size_t>(execution_order.begin(), execution_order.end()).size(), execution.size());
  }
  EXPECT_TRUE(diverse_phase_order({}, true).empty());
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, OverlappingLaneMembershipUsesIntendedPolygonAndHeading)
{
  using namespace autoware::behavior_path_planner::utils::lane_change;
  lanelet::LineString3d left(11, {lanelet::Point3d(1, 0, 2, 0), lanelet::Point3d(2, 100, 2, 0)});
  lanelet::LineString3d right(12, {lanelet::Point3d(3, 0, -2, 0), lanelet::Point3d(4, 100, -2, 0)});
  const lanelet::ConstLanelet intended(lanelet::Lanelet(201439, left, right));
  const lanelet::ConstLanelet overlapping(lanelet::Lanelet(201441, left, right));
  geometry_msgs::Pose pose;
  pose.position.x = 50; pose.position.y = 0.016; pose.orientation.w = 1;
  const lanelet::ConstLanelets lanes{intended};
  // Both polygons have distance zero, but an arbitrary global closest ID can
  // differ from the current maneuver's lane ID. Membership must remain true.
  EXPECT_FALSE(is_lanelet_in_lanelet_collections(lanes, overlapping));
  EXPECT_TRUE(contains_direction_compatible_pose(lanes, pose, 0.785));
  EXPECT_TRUE(contains_direction_compatible_pose({overlapping, intended}, pose, 0.785));
  EXPECT_TRUE(contains_direction_compatible_pose({intended, overlapping}, pose, 0.785));
  pose.position.y = 2.0;
  EXPECT_TRUE(contains_direction_compatible_pose(lanes, pose, 0.785));
  pose.position.y = 2.0001;
  EXPECT_FALSE(contains_direction_compatible_pose(lanes, pose, 0.785));
  pose.position.y = 0;
  pose.position.x = 100.001;
  EXPECT_FALSE(contains_direction_compatible_pose(lanes, pose, 0.785));
  pose.position.x = 50;
  pose.orientation.z = 1; pose.orientation.w = 0;  // Opposing carriageway.
  EXPECT_FALSE(contains_direction_compatible_pose(lanes, pose, 0.785));
  pose.orientation.z = std::sin(0.8 / 2); pose.orientation.w = std::cos(0.8 / 2);
  EXPECT_FALSE(contains_direction_compatible_pose(lanes, pose, 0.785));
  pose.orientation.z = std::sin(0.7 / 2); pose.orientation.w = std::cos(0.7 / 2);
  EXPECT_TRUE(contains_direction_compatible_pose(lanes, pose, 0.785));
  EXPECT_FALSE(contains_direction_compatible_pose({}, pose, 0.785));
  EXPECT_FALSE(contains_direction_compatible_pose(lanes, pose, -1));
  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(contains_direction_compatible_pose(lanes, pose, 0.785));
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, CompetitionDiverseRankPreservesExistingBounds)
{
  using namespace autoware::behavior_path_planner::utils::lane_change::calculation;
  std::vector<PhaseMetrics> metrics;
  for (const double lon : {0.0, -0.1, -0.5}) {
    for (const double lat : {1.0, 1.5}) metrics.emplace_back(2.0, 30.0, 15.0, lon, lon, lat);
  }
  const auto order = diverse_phase_order(metrics, false);
  ASSERT_EQ(order.size(), metrics.size());
  EXPECT_EQ(std::set<size_t>(order.begin(), order.end()).size(), metrics.size());
  // A -2 anchor ranks only existing bounded values; it cannot create -2.
  EXPECT_DOUBLE_EQ(metrics[order[1]].actual_lon_accel, -0.5);
  metrics[0].lat_accel = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_order = diverse_phase_order(metrics, false);
  for (size_t i = 0; i < metrics.size(); ++i) EXPECT_EQ(invalid_order[i], i);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, CompetitionDiverseRankOnGeneratedMetrics)
{
  using namespace autoware::behavior_path_planner;
  using namespace utils::lane_change::calculation;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
  data->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->twist.twist.linear.x = 22.227;
  data->self_odometry_ptr = odometry;
  auto & p = data->lc_param_ptr->trajectory;
  p.min_prepare_duration = 0; p.max_prepare_duration = 3;
  p.lon_acc_sampling_num = 20; p.lat_acc_sampling_num = 11;
  p.min_longitudinal_acc = -4; p.max_longitudinal_acc = 3.7;
  p.min_lane_changing_velocity = 2.78; p.lateral_jerk = 5;
  p.lat_acc_map.add(0, 1, 4.5);
  data->bpp_param_ptr->min_acc = -4; data->bpp_param_ptr->max_acc = 3.7;
  data->bpp_param_ptr->max_vel = 33.333;
  data->transient_data.lane_change_prepare_duration = 0;
  const auto preparations = calc_prepare_phase_metrics(data, 22.227, 33.333, 0, 245);
  const auto order = diverse_phase_order(preparations, true);
  ASSERT_GE(order.size(), 6U);
  for (size_t rank = 0; rank < 6; ++rank) {
    const auto & prep = preparations[order[rank]];
    EXPECT_NEAR(prep.duration, rank < 3 ? 0.0 : 1.5, epsilon);
    if (rank % 3 == 0) EXPECT_GT(prep.sampled_lon_accel, 0.0);
    if (rank % 3 == 1) EXPECT_NEAR(prep.sampled_lon_accel, 0.0, epsilon);
    if (rank % 3 == 2) EXPECT_LT(prep.sampled_lon_accel, -1.5);
    const auto execution = calc_shift_phase_metrics(
      data, 3.55, prep.velocity, 33.333, prep.sampled_lon_accel, 192);
    const auto ranked = diverse_phase_order(execution, false, rank);
    ASSERT_FALSE(ranked.empty());
    const auto & first = execution[ranked.front()];
    if (rank % 3 == 0) EXPECT_NEAR(first.lat_accel, 4.5, epsilon);
    if (rank % 3 == 1) { EXPECT_GT(first.lat_accel, 2.0); EXPECT_LT(first.lat_accel, 3.5); }
    if (rank % 3 == 2) EXPECT_NEAR(first.lat_accel, 1.0, epsilon);
    EXPECT_LE(first.actual_lon_accel, 3.7);
    EXPECT_GE(first.actual_lon_accel, -4.0);
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, CompetitionJointBudgetCoversLongitudinalRegimes)
{
  using namespace autoware::behavior_path_planner::utils::lane_change::calculation;
  // Lazy scheduling checks the first group before even initializing the others.
  JointSampleQueue lazy(100, true);
  EXPECT_EQ(lazy.pop(), std::make_pair(size_t{0}, size_t{0}));
  lazy.refine(0, 1);
  EXPECT_EQ(lazy.pop(), std::make_pair(size_t{1}, size_t{0}));
  JointSampleQueue streaming(100, false);
  EXPECT_EQ(streaming.pop(), std::make_pair(size_t{0}, size_t{0}));
  streaming.refine(0, 1);
  EXPECT_EQ(streaming.pop(), std::make_pair(size_t{0}, size_t{1}));
  const auto order = joint_sample_order({12, 12, 12});
  ASSERT_EQ(order.size(), 36U);
  // Even a three-candidate budget reaches accelerate/coast/mild-brake rather
  // than consuming twelve collision checks on accelerating lateral variants.
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(order[i].first, i);
    EXPECT_EQ(order[i].second, 0U);
  }
  const auto ragged = joint_sample_order({0, 2, 1});
  ASSERT_EQ(ragged.size(), 3U);
  EXPECT_EQ(ragged[0], std::make_pair(size_t{1}, size_t{0}));
  EXPECT_EQ(ragged[1], std::make_pair(size_t{2}, size_t{0}));
  EXPECT_EQ(ragged[2], std::make_pair(size_t{1}, size_t{1}));
  EXPECT_TRUE(joint_sample_order({}).empty());
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, CompetitionLongitudinalGridAndExecutionLimits)
{
  using namespace autoware::behavior_path_planner;
  using namespace utils::lane_change::calculation;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
  data->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->twist.twist.linear.x = 30.0;
  data->self_odometry_ptr = odometry;
  auto & p = data->lc_param_ptr->trajectory;
  p.min_prepare_duration = 0;
  p.lon_acc_sampling_num = 20;
  p.min_longitudinal_acc = -8;
  p.max_longitudinal_acc = 3.7;
  p.min_lane_changing_velocity = 2.78;
  data->bpp_param_ptr->min_acc = -6;
  data->bpp_param_ptr->max_acc = 3.7;
  data->bpp_param_ptr->max_vel = 50;
  // No route/lanes are needed: sampling also runs far from a lane end.
  auto samples = calc_lon_acceleration_samples(data, 50, 0);
  ASSERT_EQ(samples.size(), 22U);
  EXPECT_DOUBLE_EQ(samples[0], 3.7);
  EXPECT_DOUBLE_EQ(samples.back(), -6.0);
  EXPECT_DOUBLE_EQ(samples[1], 0.0);
  EXPECT_LT(samples[2], 0.0);
  EXPECT_GT(samples[2], -0.485);
  std::sort(samples.begin(), samples.end());
  EXPECT_NE(std::find(samples.begin(), samples.end(), 0.0), samples.end());
  for (size_t i = 1; i < samples.size(); ++i) EXPECT_LE(samples[i] - samples[i-1], 0.485 + epsilon);
  EXPECT_DOUBLE_EQ(calc_lane_changing_acceleration(data, 30, 50, 3, 3.7), 3.7);
  EXPECT_DOUBLE_EQ(calc_lane_changing_acceleration(data, 30, 50, 3, -6), -6);
  EXPECT_NEAR(calc_lane_changing_acceleration(data, 5, 50, 3, -6), (2.78 - 5) / 3, epsilon);
  EXPECT_NEAR(calc_lane_changing_acceleration(data, 49, 50, 3, 3.7), 1.0 / 3, epsilon);
  EXPECT_TRUE(std::isnan(calc_lane_changing_acceleration(data, 30, 1, 3, -6)));
  // Regression: with acceleration disabled, the first fallback after coasting
  // must be mild braking, not the extreme deceleration that caused a 60->10
  // km/h LC in official022. Preserve the full grid and strongest fallback.
  p.max_longitudinal_acc = 0;
  const auto braking_samples = calc_lon_acceleration_samples(data, 50, 0);
  ASSERT_EQ(braking_samples.size(), 21U);
  EXPECT_DOUBLE_EQ(braking_samples.front(), 0.0);
  EXPECT_NEAR(braking_samples[1], -0.3, epsilon);
  EXPECT_DOUBLE_EQ(braking_samples.back(), -6.0);
  for (size_t i = 1; i < braking_samples.size(); ++i) {
    EXPECT_GE(braking_samples[i-1], braking_samples[i]);
  }
  p.max_longitudinal_acc = 3.7;
  p.lon_acc_sampling_num = 0;
  EXPECT_TRUE(calc_lon_acceleration_samples(data, 50, 0).empty());
  p.lon_acc_sampling_num = 20;
  p.lat_acc_map.add(0, 1, 6.5);
  p.lat_acc_sampling_num = 11;
  p.lateral_jerk = 5;
  for (const double requested : {-6.0, 3.7}) {
    const auto metrics = calc_shift_phase_metrics(data, 3.5, 30, 50, requested, 1000);
    ASSERT_EQ(metrics.size(), 12U);
    for (const auto & metric : metrics) {
      EXPECT_DOUBLE_EQ(metric.actual_lon_accel, requested);
      EXPECT_NEAR(metric.velocity, 30 + requested * metric.duration, epsilon);
      EXPECT_NEAR(metric.length, 30 * metric.duration + 0.5 * requested * metric.duration * metric.duration, epsilon);
    }
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, CompetitionPredictionsMatchAccelerationAndStop)
{
  using namespace autoware::behavior_path_planner;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
  data->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
  data->lc_param_ptr->trajectory.min_prepare_duration = 0;
  data->lc_param_ptr->safety.collision_check.prediction_time_resolution = 0.5;
  data->bpp_param_ptr->max_vel = 50;
  data->bpp_param_ptr->ego_nearest_dist_threshold = 3;
  data->bpp_param_ptr->min_acc = -6;
  data->bpp_param_ptr->max_acc = 3.7;
  data->bpp_param_ptr->ego_nearest_yaw_threshold = 1;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.orientation.w = 1;
  odometry->twist.twist.linear.x = 10;
  data->self_odometry_ptr = odometry;
  LaneChangePath path;
  for (int x = 0; x <= 150; ++x) {
    PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.orientation.w = 1;
    path.path.points.push_back(point);
  }
  path.info.duration = {0, 4};
  path.info.velocity = {10, 10};
  path.info.lane_changing_end = path.path.points.back().point.pose;
  path.info.terminal_lane_changing_velocity = 24.8;
  for (const double acceleration : {3.7, -6.0}) {
    const auto predicted = utils::lane_change::convert_to_predicted_path(data, path, acceleration);
    ASSERT_EQ(predicted.size(), 8U);
    for (const auto & state : predicted) {
      const double t = acceleration < 0 ? std::min(state.time, 10.0 / 6.0) : state.time;
      EXPECT_NEAR(state.velocity, std::max(0.0, 10 + acceleration * t), epsilon);
      EXPECT_NEAR(state.pose.position.x, 10 * t + 0.5 * acceleration * t * t, epsilon);
    }
    auto measured = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    measured->accel.accel.linear.x = acceleration;
    data->current_acceleration = measured;
    const auto approved = utils::lane_change::convert_to_predicted_path(data, path);
    ASSERT_EQ(approved.size(), predicted.size());
    for (size_t i = 0; i < approved.size(); ++i) {
      EXPECT_NEAR(approved[i].velocity, predicted[i].velocity, epsilon);
      EXPECT_NEAR(approved[i].pose.position.x, predicted[i].pose.position.x, epsilon);
    }
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, SignalFreePreparationIsImmediateAndEarliestFirst)
{
  using namespace autoware::behavior_path_planner;
  using namespace utils::lane_change::calculation;
  auto data = std::make_shared<lane_change::CommonData>();
  data->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
  data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
  auto & p = data->lc_param_ptr->trajectory;
  p.min_prepare_duration = 0.0;
  p.max_prepare_duration = 4.0;
  p.min_lane_changing_velocity = 2.78;
  p.max_longitudinal_acc = 2.0;
  data->bpp_param_ptr->max_acc = 2.0;
  for (double signal_time : {0.0, 1.0, 10.0}) {
    EXPECT_DOUBLE_EQ(calc_actual_prepare_duration(data, 30.0, signal_time), 0.0);
  }
  data->transient_data.lane_change_prepare_duration = 0.0;
  for (double terminal_distance : {2.0, 200.0}) {
    data->transient_data.dist_to_terminal_start = terminal_distance;
    EXPECT_EQ(calc_prepare_durations(data),
      (std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0}));
  }
  // Removing signal wait must not remove the physical minimum-speed requirement.
  const double ready = calc_actual_prepare_duration(data, 0.0, 10.0);
  EXPECT_NEAR(ready, 1.39, epsilon);
  data->transient_data.lane_change_prepare_duration = ready;
  const auto low_speed_samples = calc_prepare_durations(data);
  ASSERT_FALSE(low_speed_samples.empty());
  EXPECT_DOUBLE_EQ(low_speed_samples.front(), ready);
  EXPECT_DOUBLE_EQ(low_speed_samples.back(), 4.0);
  data->bpp_param_ptr->max_acc = 0.0;
  data->transient_data.lane_change_prepare_duration = calc_actual_prepare_duration(data, 0, 0);
  EXPECT_TRUE(calc_prepare_durations(data).empty());
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, NonzeroMinimumPreservesSignalPolicy)
{
  using namespace autoware::behavior_path_planner;
  using namespace utils::lane_change::calculation;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
  data->lc_param_ptr->trajectory.min_prepare_duration = 2.0;
  data->lc_param_ptr->trajectory.max_prepare_duration = 4.0;
  EXPECT_DOUBLE_EQ(calc_actual_prepare_duration(data, 30, 0), 4.0);
  EXPECT_DOUBLE_EQ(calc_actual_prepare_duration(data, 30, 1), 3.0);
  EXPECT_DOUBLE_EQ(calc_actual_prepare_duration(data, 30, 10), 2.0);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, DenseLateralSamplesKeepBothLimits)
{
  using namespace autoware::behavior_path_planner;
  using namespace utils::lane_change::calculation;
  auto data = std::make_shared<lane_change::CommonData>();
  data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
  data->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
  data->bpp_param_ptr->max_vel = 40.0;
  auto & p = data->lc_param_ptr->trajectory;
  p.lat_acc_map.add(0.0, 1.0, 6.5);
  p.lat_acc_sampling_num = 11;
  p.lateral_jerk = 5.0;
  const auto metrics = calc_shift_phase_metrics(data, 3.5, 20.0, 40.0, 0.0, 1000.0);
  ASSERT_EQ(metrics.size(), 12U);
  for (size_t i = 0; i < metrics.size(); ++i) {
    EXPECT_NEAR(metrics[i].lat_accel, 1.0 + 0.5 * i, epsilon);
    EXPECT_TRUE(std::isfinite(metrics[i].length));
    EXPECT_GT(metrics[i].duration, 0.0);
  }
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, FuturePreferredTransitionHasDistanceBeforeEgoLeavesPreferred)
{
  using namespace autoware::behavior_path_planner;
  using autoware::route_handler::Direction;
  using utils::lane_change::calculation::calc_lc_length_and_dist_buffer;
  // Three longitudinal sections, each containing three lanes. No course IDs.
  for (const auto direction : {Direction::LEFT, Direction::RIGHT}) {
    for (const bool blocked : {false, true}) {
      auto map = std::make_shared<lanelet::LaneletMap>();
      lanelet::Id id = 100;
      std::vector<std::vector<lanelet::Point3d>> points(4);
      for (int s = 0; s < 4; ++s) {
        for (int b = 0; b < 4; ++b) {
          points[s].emplace_back(++id, 20.0 * s, 6.0 - 4.0 * b, 0.0);
        }
      }
      std::vector<lanelet::ConstLanelets> sections;
      for (int s = 0; s < 3; ++s) {
        std::vector<lanelet::LineString3d> bounds;
        for (int b = 0; b < 4; ++b) {
          lanelet::LineString3d bound(++id, {points[s][b], points[s + 1][b]});
          bound.attributes()["type"] = "line_thin";
          bound.attributes()["subtype"] = "dashed";
          bounds.push_back(bound);
        }
        lanelet::ConstLanelets section;
        for (int l = 0; l < 3; ++l) {
          auto left = bounds[l];
          auto right = bounds[l + 1];
          if (blocked && s == 1 && l == 1) {
            // A real break in the lateral corridor, not merely a solid marking:
            // lateral distance queries intentionally include adjacent solid lanes.
            const auto detached = [&](const lanelet::LineString3d & original, double dy) {
              lanelet::LineString3d line(++id, {
                lanelet::Point3d(++id, original.front().x(), original.front().y() + dy, 0),
                lanelet::Point3d(++id, original.back().x(), original.back().y() + dy, 0)});
              line.attributes() = original.attributes();
              return line;
            };
            left = detached(left, -0.1);
            right = detached(right, 0.1);
          }
          lanelet::Lanelet lane(++id, left, right);
          lane.attributes()["subtype"] = "road";
          lane.attributes()["location"] = "urban";
          lane.attributes()["one_way"] = "yes";
          map->add(lane);
          section.push_back(lane);
        }
        sections.push_back(section);
      }
      auto data = std::make_shared<lane_change::CommonData>();
      data->route_handler_ptr = std::make_shared<RouteHandler>();
      data->route_handler_ptr->setMap(map);
      data->self_odometry_ptr.reset(new nav_msgs::Odometry());
      data->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
      data->lc_param_ptr = std::make_shared<lane_change::Parameters>();
      data->lc_param_ptr->trajectory.lat_acc_map.add(0.0, 0.5, 2.0);
      data->lc_param_ptr->trajectory.lat_acc_map.add(40.0, 0.5, 2.0);
      data->lanes_ptr = std::make_shared<lane_change::Lanes>();
      data->lanes_polygon_ptr = std::make_shared<lane_change::LanesPolygon>();
      data->direction = direction;
      data->lc_type = LaneChangeModuleType::NORMAL;
      data->lanes_ptr->ego_lane = sections[0][1];
      const lanelet::ConstLanelets current{sections[0][1], sections[1][1], sections[2][1]};
      autoware_planning_msgs::LaneletRoute route;
      const int target = direction == Direction::LEFT ? 0 : 2;
      for (int s = 0; s < 3; ++s) {
        autoware_planning_msgs::LaneletSegment segment;
        for (const auto & lane : sections[s]) {
          autoware_planning_msgs::LaneletPrimitive p;
          p.id = lane.id(); p.primitive_type = "lane";
          segment.primitives.push_back(p);
        }
        segment.preferred_primitive = segment.primitives[s == 0 ? 1 : target];
        route.segments.push_back(segment);
      }
      route.start_pose.orientation.w = route.goal_pose.orientation.w = 1.0;
      route.goal_pose.position.x = 59.0;
      route.goal_pose.position.y = direction == Direction::LEFT ? 4.0 : -4.0;
      data->route_handler_ptr->setRoute(route);
      const auto distance = calc_lc_length_and_dist_buffer(data, current);
      SCOPED_TRACE(blocked ? "disconnected corridor before future LC" : "legal future LC");
      EXPECT_EQ(distance.has_value(), !blocked);
      if (distance) {
        EXPECT_GT(distance->first.min, 0.0);
        EXPECT_TRUE(std::isfinite(distance->first.max));
      }
      // Do not bridge a missing longitudinal section to a distant target.
      EXPECT_FALSE(calc_lc_length_and_dist_buffer(data, {current[0], current[2]}));
      // Staying in the preferred chain requires no LC.
      for (auto & segment : route.segments) segment.preferred_primitive = segment.primitives[1];
      data->route_handler_ptr->setRoute(route);
      EXPECT_FALSE(calc_lc_length_and_dist_buffer(data, current));
    }
  }
}

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
