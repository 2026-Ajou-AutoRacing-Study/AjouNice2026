#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <autoware/velocity_smoother/ros1_compat.hpp>
#include <autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <gtest/gtest.h>
#include <ros/master.h>

#include <cmath>
#include <vector>

namespace
{
using autoware::behavior_velocity_planner::PlannerData;
using autoware::behavior_velocity_planner::smoothPath;

autoware_internal_planning_msgs::PathWithLaneId makePath()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i = 0; i <= 100; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i);
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = i == 100 ? 0.0F : 8.0F;
    path.points.push_back(point);
  }
  return path;
}

void setJerkFilteredSmootherParameters(ros::NodeHandle & node)
{
  // These are the installed ROS1 JerkFiltered/default configuration values.
  // The provider intentionally has no implicit defaults, so the oracle owns
  // its complete parameter contract rather than relying on ambient rosparam.
  node.setParam("normal/max_acc", 1.0);
  node.setParam("normal/min_acc", -0.5);
  node.setParam("normal/max_jerk", 1.0);
  node.setParam("normal/min_jerk", -0.5);
  node.setParam("stop_decel", 0.0);
  node.setParam("min_decel_for_lateral_acc_lim_filter", -2.5);
  node.setParam("resample_ds", 0.1);
  node.setParam("curvature_threshold", 0.02);
  node.setParam("lateral_acceleration_limits", std::vector<double>{0.8, 0.8, 0.8, 0.8});
  node.setParam("velocity_thresholds", std::vector<double>{0.1, 0.3, 20.0, 30.0});
  node.setParam("steering_angle_rate_limits", std::vector<double>{57.0, 57.0, 57.0, 40.0});
  node.setParam("curvature_calculation_distance", 5.0);
  node.setParam("decel_distance_before_curve", 3.5);
  node.setParam("decel_distance_after_curve", 2.0);
  node.setParam("min_curve_velocity", 2.74);
  node.setParam("max_trajectory_length", 200.0);
  node.setParam("min_trajectory_length", 150.0);
  node.setParam("resample_time", 2.0);
  node.setParam("dense_resample_dt", 0.2);
  node.setParam("dense_min_interval_distance", 0.1);
  node.setParam("sparse_resample_dt", 0.5);
  node.setParam("sparse_min_interval_distance", 4.0);
  node.setParam("jerk_weight", 10.0);
  node.setParam("over_v_weight", 100000.0);
  node.setParam("over_a_weight", 5000.0);
  node.setParam("over_j_weight", 2000.0);
  node.setParam("jerk_filter_ds", 0.1);
}
}  // namespace

TEST(TrajectoryUtils, SmoothsInstalledJerkFilteredChainAndAppliesExternalLimit)
{
  ASSERT_TRUE(ros::master::check());
  ros::NodeHandle private_nh("/trajectory_utils_oracle");
  setJerkFilteredSmootherParameters(private_nh);
  rclcpp::Node node(private_nh);
  auto time_keeper = std::make_shared<autoware_utils_debug::TimeKeeper>();
  PlannerData data;
  auto odometry = std::make_shared<geometry_msgs::PoseStamped>();
  odometry->pose.orientation.w = 1.0;
  data.current_odometry = odometry;
  auto velocity = std::make_shared<geometry_msgs::TwistStamped>();
  velocity->twist.linear.x = 8.0;
  data.current_velocity = velocity;
  auto acceleration = std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  acceleration->accel.accel.linear.x = 0.0;
  data.current_acceleration = acceleration;
  data.velocity_smoother_ = std::make_shared<autoware::velocity_smoother::JerkFilteredSmoother>(node, time_keeper);
  autoware_internal_planning_msgs::VelocityLimit limit;
  limit.max_velocity = 2.0;
  data.external_velocity_limit = limit;

  autoware_internal_planning_msgs::PathWithLaneId out;
  ASSERT_TRUE(smoothPath(makePath(), out, data));
  ASSERT_GT(out.points.size(), 10U);
  // The installed QP smoother leaves a small numerical residual at the stop;
  // the oracle accepts the documented solver-scale tolerance, not a fake clamp.
  EXPECT_LE(std::abs(out.points.back().point.longitudinal_velocity_mps), 0.01F);
  EXPECT_LE(out.points.at(out.points.size() - 2).point.longitudinal_velocity_mps, 2.0001F);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "trajectory_utils_oracle_gtest", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
