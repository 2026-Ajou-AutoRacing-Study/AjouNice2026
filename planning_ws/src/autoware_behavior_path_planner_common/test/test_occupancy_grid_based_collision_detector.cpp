// Copyright 2024 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace
{
using autoware::behavior_path_planner::OccupancyGridBasedCollisionDetector;
using autoware::behavior_path_planner::OccupancyGridMapParam;

geometry_msgs::Pose makePose(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

class OccupancyGridBasedCollisionDetectorTest : public ::testing::Test
{
protected:
  OccupancyGridBasedCollisionDetector detector_;
  nav_msgs::OccupancyGrid costmap_;

  void SetUp() override
  {
    costmap_.info.width = 40;
    costmap_.info.height = 40;
    costmap_.info.resolution = 0.25;
    costmap_.data.assign(1600, 0);
    costmap_.data[28 * costmap_.info.width + 28] = 100;
    costmap_.info.origin = makePose(1.0, 1.0);
    OccupancyGridMapParam param;
    param.theta_size = 8;
    param.obstacle_threshold = 1;
    param.vehicle_shape.base2back = 0.0;
    param.vehicle_shape.length = 4.0;
    param.vehicle_shape.width = 2.0;
    detector_.setParam(param);
  }
};

TEST_F(OccupancyGridBasedCollisionDetectorTest, DiscretizeAndTransformCoordinates)
{
  using autoware::behavior_path_planner::discretize_angle;
  using autoware::behavior_path_planner::global2local;
  using autoware::behavior_path_planner::index2pose;
  using autoware::behavior_path_planner::pose2index;
  EXPECT_EQ(discretize_angle(0.0, 8), 0);
  EXPECT_EQ(discretize_angle(M_PI * 0.25, 8), 1);
  EXPECT_EQ(discretize_angle(2.5 * M_PI, 8), 2);
  const auto index = pose2index(costmap_, makePose(2.0, 3.0), 8);
  EXPECT_EQ(index.x, 8);
  EXPECT_EQ(index.y, 12);
  EXPECT_EQ(index.theta, 0);
  const auto pose = index2pose(costmap_, {4, 6, 2}, 8);
  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 1.5);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), M_PI * 0.5);
  const auto local = global2local(costmap_, makePose(2.0, 2.0));
  EXPECT_DOUBLE_EQ(local.position.x, 1.0);
  EXPECT_DOUBLE_EQ(local.position.y, 1.0);
}

TEST_F(OccupancyGridBasedCollisionDetectorTest, FailsClosedAndFindsObstacle)
{
  using autoware::behavior_path_planner::IndexXYT;
  IndexXYT index{24, 24, 0};
  EXPECT_FALSE(detector_.detectCollision(index, true));
  detector_.setMap(costmap_);
  EXPECT_TRUE(detector_.detectCollision(index, true));
  index = {4, 4, 0};
  EXPECT_FALSE(detector_.detectCollision(index, true));
  index = {-100, -100, 0};
  EXPECT_TRUE(detector_.detectCollision(index, true));
  EXPECT_FALSE(detector_.detectCollision(index, false));
}

TEST_F(OccupancyGridBasedCollisionDetectorTest, FindsObstacleOnPath)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  detector_.setMap(costmap_);
  EXPECT_FALSE(detector_.hasObstacleOnPath(path, true));
  for (size_t i = 0; i < 10; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose = makePose(static_cast<double>(i), 0.0);
    path.points.push_back(point);
  }
  EXPECT_FALSE(detector_.hasObstacleOnPath(path, false));
  for (auto & point : path.points) point.point.pose.position.y = 8.0;
  EXPECT_TRUE(detector_.hasObstacleOnPath(path, false));
}
}  // namespace
