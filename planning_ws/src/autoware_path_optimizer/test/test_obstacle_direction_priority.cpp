// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include <autoware/path_optimizer/node.hpp>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <cmath>

namespace autoware::path_optimizer
{
namespace
{
TrajectoryPoint point(double x, double y = 0.0)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  p.longitudinal_velocity_mps = 5.0;
  return p;
}

PlannerData scene(double upper = 6.0, double lower = -6.0)
{
  PlannerData data;
  data.header.frame_id = "map";
  data.header.stamp = ros::Time::now();
  data.ego_pose = point(5.0).pose;
  data.ego_vel = 5.0;
  data.optimization_forward_length = 50.0;
  for (int i = 0; i <= 160; ++i) {
    data.traj_points.push_back(point(0.5 * i));
    data.left_bound.push_back(point(0.5 * i, upper).pose.position);
    data.right_bound.push_back(point(0.5 * i, lower).pose.position);
  }
  autoware_planning_msgs::StaticObstacle obstacle;
  for (const auto & xy : std::vector<std::pair<double, double>>{
      {24.0, -0.5}, {26.0, -0.5}, {26.0, 0.5}, {24.0, 0.5}}) {
    geometry_msgs::Point32 p;
    p.x = xy.first;
    p.y = xy.second;
    obstacle.polygon.points.push_back(p);
  }
  obstacle.hard_margin = 0.05;
  obstacle.soft_margin = 0.55;
  data.obstacles.push_back(obstacle);
  return data;
}

std::vector<TrajectoryPoint> shortPrevious(double side)
{
  const auto smooth = [](double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
  };
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i <= 90; ++i) {
    const double x = 0.5 * i;
    const double shift = x <= 25.0 ? smooth((x - 8.0) / 12.0) :
      1.0 - smooth((x - 30.0) / 14.0);
    points.push_back(point(x, side * 2.5 * shift));
  }
  autoware::motion_utils::insertOrientation(points, true);
  return points;
}

class DirectionHarness : public PathOptimizer
{
public:
  DirectionHarness() : PathOptimizer(ros::NodeHandle(), ros::NodeHandle("~")) {}
  nlohmann::json run(
    const PlannerData & data, const std::vector<TrajectoryPoint> & previous, size_t cursor)
  {
    if (previous.empty()) last_validated_output_.reset();
    else last_validated_output_ = previous;
    obstacle_search_cursor_ = cursor;
    obstacle_candidate_trace_enabled_ = true;
    // Keep numerical scheduling variation out of these side-selection tests.
    obstacle_search_time_budget_ms_ = 1000.0;
    debug_data_ptr_->ego_pose = data.ego_pose;
    mpt_optimizer_ptr_->restoreHistory({});
    const auto output = optimizeObstacleCandidates(data, obstacle_search_time_budget_ms_);
    auto trace = nlohmann::json::parse(obstacle_candidate_trace_);
    trace["output_present"] = output.has_value();
    if (output && !output->empty()) trace["output_end_x"] = output->back().pose.position.x;
    return trace;
  }
};

TEST(ObstacleDirectionPriority, ShortLeftHistoryStillStartsWithNewLeftSolve)
{
  DirectionHarness node;
  const auto trace = node.run(scene(), shortPrevious(1.0), 0);
  EXPECT_FALSE(trace["previous_checks"]["covers_horizon"].get<bool>());
  EXPECT_FALSE(trace["retained_incumbent"].get<bool>());
  EXPECT_EQ(trace["first_mask"], 1);
  ASSERT_FALSE(trace["attempts"].empty());
  EXPECT_EQ(trace["attempts"][0]["mask"], 1);
  EXPECT_TRUE(trace["output_present"].get<bool>());
  EXPECT_EQ(trace["winner_mask"], 1);
  EXPECT_FALSE(trace["winner_previous"].get<bool>());
  EXPECT_GT(trace["output_end_x"].get<double>(), 45.0);
}

TEST(ObstacleDirectionPriority, ShortRightHistoryStillStartsWithNewRightSolve)
{
  DirectionHarness node;
  const auto trace = node.run(scene(), shortPrevious(-1.0), 1);
  EXPECT_FALSE(trace["retained_incumbent"].get<bool>());
  EXPECT_EQ(trace["first_mask"], 0);
  EXPECT_TRUE(trace["output_present"].get<bool>());
  EXPECT_EQ(trace["winner_mask"], 0);
  EXPECT_FALSE(trace["winner_previous"].get<bool>());
}

TEST(ObstacleDirectionPriority, DirectionHintDoesNotRequirePreferredSpatialSeed)
{
  DirectionHarness node;
  auto data = scene();
  data.ego_pose.position.y = 0.4;
  const auto trace = node.run(data, shortPrevious(1.0), 0);
  EXPECT_FALSE(trace["prefer_previous_seed"].get<bool>());
  EXPECT_TRUE(trace.value("prefer_previous_direction", false));
  EXPECT_EQ(trace["first_mask"], 1);
  EXPECT_TRUE(trace["output_present"].get<bool>());
  EXPECT_EQ(trace["winner_mask"], 1);
}

TEST(ObstacleDirectionPriority, InfeasiblePreviousSideCanFallBackToOtherSide)
{
  DirectionHarness node;
  const auto trace = node.run(scene(1.5), shortPrevious(1.0), 0);
  EXPECT_EQ(trace["first_mask"], 1);
  ASSERT_FALSE(trace["attempts"].empty());
  EXPECT_EQ(trace["attempts"][0]["mask"], 1);
  EXPECT_TRUE(trace["output_present"].get<bool>());
  EXPECT_EQ(trace["winner_mask"], 0);
  EXPECT_EQ(trace["examined"], 2);
}

TEST(ObstacleDirectionPriority, NoHistoryStartsOnWiderLeftSide)
{
  DirectionHarness node;
  const auto trace = node.run(scene(6.0, -3.0), {}, 0);
  EXPECT_FALSE(trace.value("prefer_previous_direction", false));
  EXPECT_EQ(trace["first_mask"], 1);
  ASSERT_FALSE(trace["attempts"].empty());
  EXPECT_EQ(trace["attempts"][0]["mask"], 1);
  EXPECT_TRUE(trace["side_room"][0]["pass_left"].get<bool>());
  EXPECT_NEAR(trace["side_room"][0]["left_gap"].get<double>(), 5.5, 1e-6);
  EXPECT_NEAR(trace["side_room"][0]["right_gap"].get<double>(), 2.5, 1e-6);
}

TEST(ObstacleDirectionPriority, NoHistoryStartsOnWiderRightSide)
{
  DirectionHarness node;
  const auto trace = node.run(scene(3.0, -6.0), {}, 1);
  EXPECT_EQ(trace["first_mask"], 0);
  EXPECT_FALSE(trace["side_room"][0]["pass_left"].get<bool>());
}

TEST(ObstacleDirectionPriority, NoHistoryEqualRoomUsesObstacleCenterNotCursor)
{
  DirectionHarness node;
  auto data = scene();
  for (auto & vertex : data.obstacles.front().polygon.points) vertex.y -= 0.03;
  const auto trace = node.run(data, {}, 0);
  // Room differs by 0.06 m (within tolerance); the center is right of the path.
  EXPECT_EQ(trace["first_mask"], 1);
}
}  // namespace
}  // namespace autoware::path_optimizer

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_obstacle_direction_priority");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
