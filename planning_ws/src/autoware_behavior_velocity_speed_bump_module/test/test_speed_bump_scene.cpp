#include "scene.hpp"
#include "normal/scene.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <gtest/gtest.h>

#include <algorithm>

namespace
{
using autoware::behavior_velocity_planner::experimental::SpeedBumpModule;
using autoware::behavior_velocity_planner::experimental::Trajectory;

autoware_internal_planning_msgs::PathPointWithLaneId point(const double x)
{
  autoware_internal_planning_msgs::PathPointWithLaneId value;
  value.point.pose.position.x = x;
  value.point.pose.orientation.w = 1.0;
  value.point.longitudinal_velocity_mps = 8.0F;
  value.lane_ids = {42};
  return value;
}

TEST(SpeedBumpScene, AppliesAnnotatedSlowDownSpeedAcrossIntersectingPath)
{
  ros::NodeHandle node;
  lanelet::AttributeMap bump_attributes;
  bump_attributes["height"] = "0.10";
  bump_attributes["slow_down_speed"] = "18.0";  // km/h, as specified by the upstream contract
  const lanelet::Polygon3d bump_polygon(
    100, {lanelet::Point3d(101, 4.0, -1.0, 0.0), lanelet::Point3d(102, 6.0, -1.0, 0.0),
          lanelet::Point3d(103, 6.0, 1.0, 0.0), lanelet::Point3d(104, 4.0, 1.0, 0.0)},
    bump_attributes);
  auto speed_bump = lanelet::autoware::SpeedBump::make(200, {}, bump_polygon);
  SpeedBumpModule::PlannerParam param{1.0, 1.0, false, 0.05F, 0.30F, 1.39F, 2.78F};
  auto factor = std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
    &node, "test_speed_bump");
  SpeedBumpModule module(200, *speed_bump, param, node,
    std::make_shared<autoware_utils_debug::TimeKeeper>(), factor);
  auto data = std::make_shared<autoware::behavior_velocity_planner::PlannerData>();
  data->vehicle_info_.max_longitudinal_offset_m = 1.0;
  data->vehicle_info_.rear_overhang_m = 1.0;
  auto odometry = std::make_shared<geometry_msgs::PoseStamped>();
  odometry->pose.orientation.w = 1.0;
  data->current_odometry = odometry;

  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int x = 0; x <= 10; ++x) path.points.push_back(point(x));

  auto trajectory = *Trajectory::Builder{}.build(path.points);
  ASSERT_TRUE(module.modifyPathVelocity(trajectory, {}, {}, *data));
  const auto output = trajectory.restore();
  const auto minimum_velocity = std::min_element(
    output.begin(), output.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.point.longitudinal_velocity_mps < rhs.point.longitudinal_velocity_mps;
    })->point.longitudinal_velocity_mps;
  const auto slowed = std::count_if(output.begin(), output.end(), [](const auto & p) {
    return std::abs(p.point.longitudinal_velocity_mps - 5.0F) < 1e-4F;
  });
  EXPECT_GE(slowed, 3) << "minimum velocity was " << minimum_velocity;
  EXPECT_FLOAT_EQ(output.front().point.longitudinal_velocity_mps, 8.0F);
  EXPECT_FLOAT_EQ(output.back().point.longitudinal_velocity_mps, 8.0F);
  EXPECT_FALSE(module.createDebugMarkerArray().markers.empty());
}

TEST(SpeedBumpScene, NormalAbiPreservesUpstreamSlowDownSection)
{
  ros::NodeHandle node;
  lanelet::AttributeMap attributes;
  attributes["height"] = "0.10";
  attributes["slow_down_speed"] = "18.0";
  const lanelet::Polygon3d polygon(
    300, {lanelet::Point3d(301, 4.0, -1.0, 0.0), lanelet::Point3d(302, 6.0, -1.0, 0.0),
          lanelet::Point3d(303, 6.0, 1.0, 0.0), lanelet::Point3d(304, 4.0, 1.0, 0.0)},
    attributes);
  auto speed_bump = lanelet::autoware::SpeedBump::make(400, {}, polygon);
  autoware::behavior_velocity_planner::SpeedBumpModule::PlannerParam param{
    1.0, 1.0, false, 0.05F, 0.30F, 1.39F, 2.78F};
  auto factor = std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
    &node, "test_speed_bump_normal");
  autoware::behavior_velocity_planner::SpeedBumpModule module(
    400, 42, *speed_bump, param, node,
    std::make_shared<autoware_utils_debug::TimeKeeper>(), factor);
  auto data = std::make_shared<autoware::behavior_velocity_planner::PlannerData>();
  data->vehicle_info_.max_longitudinal_offset_m = 1.0;
  data->vehicle_info_.rear_overhang_m = 1.0;
  auto odometry = std::make_shared<geometry_msgs::PoseStamped>();
  odometry->pose.orientation.w = 1.0;
  data->current_odometry = odometry;
  module.setPlannerData(data);

  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int x = 0; x <= 10; ++x) path.points.push_back(point(x));
  ASSERT_TRUE(module.modifyPathVelocity(&path));
  const auto slowed = std::count_if(path.points.begin(), path.points.end(), [](const auto & p) {
    return std::abs(p.point.longitudinal_velocity_mps - 5.0F) < 1e-4F;
  });
  EXPECT_GE(slowed, 3);
  EXPECT_FLOAT_EQ(path.points.front().point.longitudinal_velocity_mps, 8.0F);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 8.0F);
  EXPECT_FALSE(module.createDebugMarkerArray().markers.empty());
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "speed_bump_scene_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
