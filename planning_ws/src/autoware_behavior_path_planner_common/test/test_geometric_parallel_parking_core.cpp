#include <autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking_core.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <stdexcept>

namespace parking = autoware::behavior_path_planner::utils::parking_departure;

TEST(GeometricParallelParkingCore, GeneratesExactEndpointAndDirectionPose)
{
  geometry_msgs::Pose center;
  center.orientation.w = 1.0;
  const auto path = parking::generateArcPath(center, 2.0, -M_PI_2, 0.0, 0.7, true, true);
  ASSERT_GE(path.points.size(), 2U);
  EXPECT_NEAR(path.points.front().point.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(path.points.front().point.pose.position.y, -2.0, 1e-9);
  EXPECT_NEAR(path.points.back().point.pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(path.points.back().point.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.points.front().point.pose.orientation), 0.0, 1e-9);
}

TEST(GeometricParallelParkingCore, PreservesDirectionAndTerminalStopPolicy)
{
  geometry_msgs::Pose center;
  center.orientation.w = 1.0;
  auto left = parking::generateArcPath(center, 2.0, -M_PI_2, 0.0, 1.0, true, true);
  auto right_reverse = parking::generateArcPath(center, 2.0, M_PI_2, 0.0, 1.0, false, false);
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> paths{left, right_reverse};
  parking::setVelocityToArcPaths(paths, -2.0, true);
  ASSERT_FALSE(paths.front().points.empty());
  EXPECT_FLOAT_EQ(paths.front().points.front().point.longitudinal_velocity_mps, -2.0F);
  EXPECT_FLOAT_EQ(paths.front().points.back().point.longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(paths.back().points.back().point.longitudinal_velocity_mps, 0.0F);
  EXPECT_THROW(parking::generateArcPath(center, 0.0, 0.0, 1.0, 1.0, true, true), std::invalid_argument);
}

TEST(GeometricParallelParkingCore, PreservesPathIndexAndFullPathOverlapSemantics)
{
  geometry_msgs::Pose center;
  center.orientation.w = 1.0;
  auto first = parking::generateArcPath(center, 2.0, -M_PI_2, -0.5, 0.5, true, true);
  auto second = parking::generateArcPath(center, 2.0, -0.5, 0.0, 0.5, true, true);
  parking::GeometricParallelParkingPathState state;
  state.setArcPaths({first, second});
  state.setPaths({first, second});
  EXPECT_FALSE(state.isParking());
  const auto full = state.getFullPath();
  EXPECT_LT(full.points.size(), first.points.size() + second.points.size());
  EXPECT_EQ(state.getArcPath().points.size(), first.points.size() + second.points.size());
  state.incrementPathIndex();
  EXPECT_TRUE(state.isParking());
  EXPECT_EQ(state.getCurrentPath().points.size(), second.points.size());
  state.incrementPathIndex();
  EXPECT_EQ(state.getCurrentPath().points.size(), second.points.size());
  EXPECT_TRUE(state.getPathByIdx(2).points.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
