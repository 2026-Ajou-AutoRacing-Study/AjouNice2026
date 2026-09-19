#include "scene.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <gtest/gtest.h>

namespace
{
using autoware::behavior_velocity_planner::StopLineModule;

autoware_internal_planning_msgs::PathPointWithLaneId point(const double x)
{
  autoware_internal_planning_msgs::PathPointWithLaneId value;
  value.point.pose.position.x = x;
  value.point.pose.orientation.w = 1.0;
  value.lane_ids = {42};
  return value;
}

TEST(StopLineScene, ComputesApproachStoppedAndStartSemantics)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "stop_line_scene_test", ros::init_options::AnonymousName);
  }
  ros::NodeHandle node;
  StopLineModule::PlannerParam param{0.5, 2.0, 0.0, 0.5};
  const lanelet::ConstLineString3d stop_line(
    7, {lanelet::Point3d(8, 7.0, -1.0, 0.0), lanelet::Point3d(9, 7.0, 1.0, 0.0)});
  auto factor = std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(&node, "test_stopline");
  StopLineModule module(1, stop_line, 42, param, node, std::make_shared<autoware_utils_debug::TimeKeeper>(), factor);
  auto data = std::make_shared<autoware::behavior_velocity_planner::PlannerData>();
  data->vehicle_info_.max_longitudinal_offset_m = 1.0;
  module.setPlannerData(data);

  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int x = 0; x <= 10; ++x) path.points.push_back(point(x));
  path.left_bound = {geometry_msgs::Point(), geometry_msgs::Point()};
  path.left_bound.front().x = 0.0; path.left_bound.front().y = 1.0;
  path.left_bound.back().x = 10.0; path.left_bound.back().y = 1.0;
  path.right_bound = path.left_bound;
  path.right_bound.front().y = -1.0; path.right_bound.back().y = -1.0;
  const auto trajectory = *StopLineModule::Trajectory::Builder{}.build(path.points);
  geometry_msgs::Pose ego; ego.position.x = 5.0;

  const auto [ego_s, approach_stop] = module.getEgoAndStopPoint(trajectory, path, ego, StopLineModule::State::APPROACH);
  EXPECT_DOUBLE_EQ(ego_s, 5.0);
  ASSERT_TRUE(approach_stop.has_value());
  EXPECT_DOUBLE_EQ(*approach_stop, 5.5);
  EXPECT_DOUBLE_EQ(module.getEgoAndStopPoint(trajectory, path, ego, StopLineModule::State::STOPPED).second.value(), 5.0);
  EXPECT_FALSE(module.getEgoAndStopPoint(trajectory, path, ego, StopLineModule::State::START).second.has_value());

  StopLineModule::State state = StopLineModule::State::APPROACH;
  std::optional<ros::Time> stopped_time;
  module.updateStateAndStoppedTime(&state, &stopped_time, ros::Time(10.0), 0.1, true);
  EXPECT_EQ(state, StopLineModule::State::STOPPED);
  module.updateStateAndStoppedTime(&state, &stopped_time, ros::Time(13.0), 0.1, true);
  EXPECT_EQ(state, StopLineModule::State::START);
  EXPECT_FALSE(stopped_time.has_value());

  auto odometry = std::make_shared<geometry_msgs::PoseStamped>();
  odometry->pose = ego;
  data->current_odometry = odometry;
  auto velocity = std::make_shared<geometry_msgs::TwistStamped>();
  velocity->twist.linear.x = 0.0;
  data->current_velocity = velocity;
  for (auto & path_point : path.points) path_point.point.longitudinal_velocity_mps = 8.0F;
  const auto original_count = path.points.size();
  ASSERT_TRUE(module.modifyPathVelocity(&path));
  EXPECT_GE(path.points.size(), original_count);
  EXPECT_FALSE(factor->get_factors().empty());
  EXPECT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0F);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "stop_line_scene_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
