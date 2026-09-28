#include "autoware/path_optimizer/obstacle_clustering.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

namespace autoware::path_optimizer
{
namespace
{
TrajectoryPoint referencePoint(const double x, const double y, const double yaw = 0.0)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation.z = std::sin(yaw / 2.0);
  point.pose.orientation.w = std::cos(yaw / 2.0);
  return point;
}

autoware_planning_msgs::StaticObstacle box(
  const double x, const double y, const double length = 1.5, const double width = 0.8)
{
  autoware_planning_msgs::StaticObstacle obstacle;
  for (const auto & vertex : std::vector<std::pair<double, double>>{
      {x, y}, {x + length, y}, {x + length, y + width}, {x, y + width}}) {
    geometry_msgs::Point32 point;
    point.x = vertex.first;
    point.y = vertex.second;
    obstacle.polygon.points.push_back(point);
  }
  obstacle.hard_margin = 0.1;
  obstacle.soft_margin = 0.2;
  return obstacle;
}

std::vector<TrajectoryPoint> straightReference()
{
  std::vector<TrajectoryPoint> reference;
  for (int x = 0; x <= 50; ++x) reference.push_back(referencePoint(x, 0.0));
  return reference;
}
}  // namespace

TEST(ObstacleClustering, LeavesScenesBelowTriggerUnchanged)
{
  const std::vector<autoware_planning_msgs::StaticObstacle> obstacles{
    box(10.0, 1.0), box(10.1, 1.0), box(10.2, 1.0)};
  EXPECT_EQ(clusterStaticObstacles(obstacles, straightReference(), {}).size(), 3U);
}

TEST(ObstacleClustering, CollapsesNearbyDuplicateFootprints)
{
  std::vector<autoware_planning_msgs::StaticObstacle> obstacles;
  for (int i = 0; i < 7; ++i) obstacles.push_back(box(10.0 + 0.12 * i, 1.0 + 0.03 * i));
  obstacles.back().hard_margin = 0.4;
  obstacles.back().soft_margin = 0.6;
  const auto clustered = clusterStaticObstacles(obstacles, straightReference(), {});
  ASSERT_EQ(clustered.size(), 1U);
  EXPECT_DOUBLE_EQ(clustered.front().hard_margin, 0.4);
  EXPECT_DOUBLE_EQ(clustered.front().soft_margin, 0.6);
  EXPECT_GE(clustered.front().polygon.points.size(), 4U);
}

TEST(ObstacleClustering, KeepsOppositeSidesSeparate)
{
  const std::vector<autoware_planning_msgs::StaticObstacle> obstacles{
    box(10.0, 0.3), box(10.1, 0.35), box(10.0, -1.1), box(10.1, -1.05)};
  EXPECT_EQ(clusterStaticObstacles(obstacles, straightReference(), {}).size(), 2U);
}

TEST(ObstacleClustering, DoesNotBridgeAChainPastMaximumSpan)
{
  const std::vector<autoware_planning_msgs::StaticObstacle> obstacles{
    box(5.0, 1.0), box(6.8, 1.0), box(8.6, 1.0), box(10.4, 1.0)};
  ObstacleClusteringParameters parameters;
  parameters.polygon_gap_threshold = 0.5;
  parameters.max_longitudinal_span = 4.0;
  EXPECT_GT(clusterStaticObstacles(obstacles, straightReference(), parameters).size(), 1U);
}

TEST(ObstacleClustering, KeepsSeparatedRealObstaclesDistinct)
{
  const std::vector<autoware_planning_msgs::StaticObstacle> obstacles{
    box(5.0, 1.0), box(12.0, 1.0), box(19.0, 1.0), box(26.0, 1.0),
    box(33.0, 1.0), box(40.0, 1.0), box(47.0, 1.0)};
  EXPECT_EQ(clusterStaticObstacles(obstacles, straightReference(), {}).size(), 7U);
}
}  // namespace autoware::path_optimizer

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
