#include <autoware/path_optimizer/node.hpp>
#include <gtest/gtest.h>

namespace autoware::path_optimizer
{
class VelocityReuseHarness : public PathOptimizer
{
public:
  VelocityReuseHarness() : PathOptimizer(ros::NodeHandle(), ros::NodeHandle("~")) {}
  void seed(const PlannerData & data, const double old_speed)
  {
    auto previous = data.traj_points;
    for (auto & point : previous) {
      point.pose.position.y = 0.05;  // Keep the already optimized lateral geometry.
      point.longitudinal_velocity_mps = old_speed;
    }
    auto history = mpt_optimizer_ptr_->history();
    history.trajectory = std::make_shared<std::vector<TrajectoryPoint>>(previous);
    mpt_optimizer_ptr_->restoreHistory(history);
    replan_checker_ptr_->updateData(data, true, now());
    enable_reset_prev_optimization_ = false;
    enable_skip_optimization_ = false;
    is_optimization_failed_ = false;
  }
  using PathOptimizer::optimizeTrajectory;
};

PlannerData straight(const double speed)
{
  PlannerData data;
  data.header.frame_id = "map";
  data.header.stamp = ros::Time::now();
  data.ego_pose.position.x = 20.0;
  data.ego_pose.orientation.w = 1.0;
  data.ego_vel = 15.0;
  for (int i = 0; i <= 240; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 0.5;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = speed;
    data.traj_points.push_back(point);
  }
  return data;
}

TEST(PathOptimizerReusedGeometryVelocity, NewSlowdownReplacesOldVelocityWithoutMovingTheGeometry)
{
  ros::Time::setNow(ros::Time(100));
  VelocityReuseHarness optimizer;
  const auto input = straight(4.0);
  optimizer.seed(input, 25.0);
  const auto output = optimizer.optimizeTrajectory(input);
  ASSERT_FALSE(output.empty());
  for (const auto & point : output) {
    EXPECT_NEAR(point.pose.position.y, 0.05, 1e-9);
    EXPECT_NEAR(point.longitudinal_velocity_mps, 4.0, 1e-6);
  }
}
TEST(PathOptimizerReusedGeometryVelocity, StopReplacesOldCruisingVelocityImmediately)
{
  ros::Time::setNow(ros::Time(100));
  VelocityReuseHarness optimizer;
  const auto input = straight(0.0);
  optimizer.seed(input, 25.0);
  const auto output = optimizer.optimizeTrajectory(input);
  ASSERT_FALSE(output.empty());
  for (const auto & point : output) EXPECT_DOUBLE_EQ(point.longitudinal_velocity_mps, 0.0);
}
TEST(PathOptimizerReusedGeometryVelocity, ClearedSlowdownDoesNotLatchOldLowVelocity)
{
  ros::Time::setNow(ros::Time(100));
  VelocityReuseHarness optimizer;
  const auto input = straight(20.0);
  optimizer.seed(input, 4.0);
  const auto output = optimizer.optimizeTrajectory(input);
  ASSERT_FALSE(output.empty());
  for (const auto & point : output) EXPECT_NEAR(point.longitudinal_velocity_mps, 20.0, 1e-6);
}
}
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_reused_geometry_velocity");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
