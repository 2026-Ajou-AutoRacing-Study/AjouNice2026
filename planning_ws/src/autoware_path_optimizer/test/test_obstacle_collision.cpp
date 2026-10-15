#include "autoware/path_optimizer/obstacle_collision.hpp"
#include <gtest/gtest.h>
#include "autoware/path_optimizer/utils/geometry_utils.hpp"
#include "autoware/path_optimizer/obstacle_interval.hpp"
#include "autoware/path_optimizer/utils/trajectory_utils.hpp"

namespace autoware::path_optimizer
{
static autoware::vehicle_info_utils::VehicleInfo vehicle()
{
  autoware::vehicle_info_utils::VehicleInfo v;
  v.vehicle_width_m = 1.89; v.wheel_base_m = 2.745;
  v.front_overhang_m = 1.1; v.rear_overhang_m = 0.79;
  return v;
}
static autoware_planning_msgs::StaticObstacle box(double x, double y, double length, double width)
{
  autoware_planning_msgs::StaticObstacle o;
  for (auto p : std::vector<std::pair<double,double>>{{x,y},{x+length,y},{x+length,y+width},{x,y+width}}) {
    geometry_msgs::Point32 q; q.x=p.first; q.y=p.second; o.polygon.points.push_back(q);
  }
  o.hard_margin = 0.1;
  return o;
}
static TrajectoryPoint point(double x, double y)
{
  TrajectoryPoint p; p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.0; return p;
}
TEST(ObstacleFootprint, PreparedRoadMatchesAnalyticCircleClearance)
{
  std::vector<geometry_msgs::Point> left, right;
  for (int x = 0; x <= 100; ++x) {
    geometry_msgs::Point p; p.x = x; p.y = 3.0; left.push_back(p);
    p.y = -3.0; right.push_back(p);
  }
  geometry_utils::CircleFootprintRoadChecker road(left, right);
  const std::vector<double> offsets{-0.5, 0.5, 1.5, 2.5};
  const std::vector<double> radii(offsets.size(), 0.8);
  for (int angle = -14; angle <= 14; ++angle) {
    for (int lateral = -30; lateral <= 30; ++lateral) {
      auto pose = point(40.0, 0.103 * lateral).pose;
      const double yaw = angle * 0.107;
      pose.orientation.z = std::sin(yaw / 2.0);
      pose.orientation.w = std::cos(yaw / 2.0);
      bool outside = false;
      for (double offset : offsets) {
        outside |= std::abs(pose.position.y + std::sin(yaw) * offset) + 0.8 >= 3.0;
      }
      EXPECT_EQ(road.isOutside(pose, radii, offsets), outside);
    }
  }
}
TEST(ObstacleFootprint, PreparedRoadOwnsCurrentSceneAndRefreshesNextScene)
{
  auto left = std::vector<geometry_msgs::Point>{point(0,3).pose.position, point(100,3).pose.position};
  auto right = std::vector<geometry_msgs::Point>{point(0,-3).pose.position, point(100,-3).pose.position};
  geometry_utils::CircleFootprintRoadChecker previous(left, right);
  const auto pose = point(10,0).pose;
  EXPECT_FALSE(previous.isOutside(pose, {1.0}, {0.0}));
  for (auto & p : left) p.y = 0.9;
  EXPECT_FALSE(previous.isOutside(pose, {1.0}, {0.0}));
  geometry_utils::CircleFootprintRoadChecker current(left, right);
  EXPECT_TRUE(current.isOutside(pose, {1.0}, {0.0}));
  EXPECT_TRUE(current.isOutside(pose, {}, {}));
}

TEST(ObstacleFootprint, PublishedForwardHeadingDoesNotInheritFirstPoseDirection)
{
  std::vector<TrajectoryPoint> input{point(0,0), point(1,0), point(2,0)};
  // Reproduce the direction ambiguity in a deformed behind-ego prefix.
  input.front().pose.orientation.w = 0.0;
  input.front().pose.orientation.z = 1.0;
  for (auto & p : input) p.longitudinal_velocity_mps = 3.0;
  const auto forward = trajectory_utils::resampleTrajectoryPoints(input, 0.2, true);
  for (const auto & p : forward) EXPECT_NEAR(tf2::getYaw(p.pose.orientation), 0.0, 1e-8);
  const auto reverse = trajectory_utils::resampleTrajectoryPoints(input, 0.2, false);
  for (const auto & p : reverse) EXPECT_NEAR(std::abs(tf2::getYaw(p.pose.orientation)), M_PI, 1e-8);
}
TEST(ObstacleFootprint, PublishedInitialPoseMustMatchMeasuredState)
{
  auto measured = point(0,0).pose;
  auto planned = measured;
  const auto set_yaw = [&](double yaw) {
    planned.orientation.z = std::sin(yaw / 2.0);
    planned.orientation.w = std::cos(yaw / 2.0);
  };
  set_yaw(0.5686);  // Recorded forward-facing but untrackable publication.
  EXPECT_FALSE(isAlignedWithEgoPose(planned, measured));
  set_yaw(0.1);
  EXPECT_TRUE(isAlignedWithEgoPose(planned, measured));
  planned.position.x = 1.01;
  EXPECT_FALSE(isAlignedWithEgoPose(planned, measured));
}
TEST(ObstacleFootprint, LocalKinkCannotBeHiddenByMetreWideCurvature)
{
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < 11; ++i) points.push_back(point(0.0, 0.2 * i));
  EXPECT_FALSE(firstCurvatureViolation(points, 1, 10, 0.25));
  points[5].pose.position.x = 0.02;
  points[6].pose.position.x = -0.04;
  EXPECT_TRUE(firstCurvatureViolation(points, 1, 10, 0.25));
}
TEST(ObstacleFootprint, VehicleCornerNotJustCenterline)
{
  EXPECT_TRUE(firstObstacleCollision({point(0,0)},0,{box(3.7,0.8,1,1)},vehicle()));
  EXPECT_FALSE(firstObstacleCollision({point(0,0)},0,{box(3.7,1.2,1,1)},vehicle()));
}
TEST(ObstacleFootprint, ConstraintCirclesCoverBumpersAndGaps)
{
  const auto v = vehicle();
  const std::vector<double> offsets{-0.15, 0.65625, 1.4625, 2.26875, 3.075};
  const auto radii = obstacleCoveringRadii(v, offsets);
  for (double x = -v.rear_overhang_m; x <= v.wheel_base_m + v.front_overhang_m; x += 0.001) {
    bool covered = false;
    for (size_t i = 0; i < offsets.size(); ++i) {
      covered = covered || std::hypot(x - offsets[i], v.vehicle_width_m / 2.0) <= radii[i] + 1e-8;
    }
    ASSERT_TRUE(covered) << "Uncovered vehicle edge at x=" << x;
  }
}
TEST(TrajectoryGeometry, RejectsTurnsBeyondPhysicalSteeringLimit)
{
  for (const double radius : {3.0, 8.0}) {
    std::vector<TrajectoryPoint> points;
    for (size_t i = 0; i < 80; ++i) {
      const double angle = i * 0.025;
      points.push_back(point(radius * std::sin(angle), radius * (1.0 - std::cos(angle))));
    }
    EXPECT_EQ(firstCurvatureViolation(points, 0, points.size(), 0.25).has_value(), radius == 3.0);
  }
}
TEST(ObstacleFootprint, ThinObstacleBetweenTrajectorySamples)
{
  EXPECT_TRUE(firstObstacleCollision({point(0,0),point(10,0)},0,{box(5,-0.1,0.01,0.2)},vehicle()));
}
TEST(ObstacleFootprint, LatestLargerObservationInvalidatesPreviousPath)
{
  auto path=std::vector<TrajectoryPoint>{point(0,0),point(1,0)};
  EXPECT_FALSE(firstObstacleCollision(path,0,{box(2,2,1,1)},vehicle()));
  EXPECT_TRUE(firstObstacleCollision(path,0,{box(2,0.5,1,2.5)},vehicle()));
}
TEST(ObstacleFootprint, InvalidPolygonIsNotFreeSpace)
{
  EXPECT_TRUE(firstObstacleCollision({point(0,0)},0,{autoware_planning_msgs::StaticObstacle{}},vehicle()));
}
}

TEST(ObstacleInterval, RoundedLongitudinalCorner)
{
  geometry_msgs::Polygon polygon;
  for (const auto & p : std::vector<std::pair<double,double>>{{.9,-1},{2,-1},{2,1},{.9,1}}) {
    geometry_msgs::Point32 q; q.x=p.first; q.y=p.second; polygon.points.push_back(q);
  }
  geometry_msgs::Pose origin; origin.orientation.w=1;
  const auto interval=autoware::path_optimizer::obstacleLateralInterval(polygon,origin,1.0);
  ASSERT_TRUE(interval);
  EXPECT_NEAR(interval->second,1+std::sqrt(1-.9*.9),1e-6);
  EXPECT_NEAR(interval->first,-1-std::sqrt(1-.9*.9),1e-6);
  EXPECT_FALSE(autoware::path_optimizer::obstacleLateralInterval(polygon,origin,.5));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace autoware::path_optimizer {
TEST(ObstacleSidePruning, AlongsideRejectsOnlyDisconnectedSide) {
  const auto obstacle = box(-3, -1, 6, 2);
  auto ego = point(0, 3).pose;
  EXPECT_TRUE(oppositeSideAtMeasuredState(obstacle.polygon, ego, 0, false, 0.5));
  EXPECT_FALSE(oppositeSideAtMeasuredState(obstacle.polygon, ego, 0, true, 0.5));
  ego.position.y = -3;
  EXPECT_TRUE(oppositeSideAtMeasuredState(obstacle.polygon, ego, 0, true, 0.5));
  EXPECT_FALSE(oppositeSideAtMeasuredState(obstacle.polygon, ego, 0, false, 0.5));
}
TEST(ObstacleSidePruning, ApproachDepartureAndTurningRemainSearchable) {
  const auto obstacle = box(-3, -1, 6, 2);
  for (double x : {-5.0, -2.5, 2.5, 5.0}) {
    auto ego = point(x,3).pose;
    EXPECT_FALSE(oppositeSideAtMeasuredState(obstacle.polygon, ego, 0, false, 0.5));
  }
  EXPECT_FALSE(oppositeSideAtMeasuredState(obstacle.polygon, point(0,3).pose, 0.3, false, 0.5));
  EXPECT_FALSE(oppositeSideAtMeasuredState(obstacle.polygon, point(0,0).pose, 0, false, 0.5));
}
}

namespace autoware::path_optimizer {
TEST(ObstacleSidePruning, RotatedMapPreservesSideAndApproach) {
  const double yaw = 1.2, c = std::cos(yaw), sn = std::sin(yaw);
  auto obstacle = box(-3, -1, 6, 2);
  for (auto & vertex : obstacle.polygon.points) {
    const double x = vertex.x, y = vertex.y;
    vertex.x = 100 + c*x - sn*y;
    vertex.y = 200 + sn*x + c*y;
  }
  auto ego = point(100-3*sn, 200+3*c).pose;
  ego.orientation.z = std::sin(yaw/2); ego.orientation.w = std::cos(yaw/2);
  EXPECT_TRUE(oppositeSideAtMeasuredState(obstacle.polygon, ego, yaw, false, 0.5));
  EXPECT_FALSE(oppositeSideAtMeasuredState(obstacle.polygon, ego, yaw, true, 0.5));
  ego.position.x -= 5*c; ego.position.y -= 5*sn;
  EXPECT_FALSE(oppositeSideAtMeasuredState(obstacle.polygon, ego, yaw, false, 0.5));
}
}
