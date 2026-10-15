#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <autoware/velocity_smoother/ros1_compat.hpp>
#include <autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp>
#include <autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <gtest/gtest.h>
#include <ros/master.h>

#include <cmath>
#include <limits>
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
  // A moving vehicle cannot reach the new cap instantaneously. The old
  // post-smoothing clamp incorrectly replaced this initial speed with 2 m/s.
  EXPECT_NEAR(out.points.front().point.longitudinal_velocity_mps, 8.0F, 0.05F);
}

namespace
{
using TrajectoryPoints = autoware::velocity_smoother::TrajectoryPoints;
class InspectingSmoother : public autoware::velocity_smoother::JerkFilteredSmoother
{
public:
  using JerkFilteredSmoother::JerkFilteredSmoother;
  TrajectoryPoints smoothing_input;
  bool apply(
    const double v0, const double a0, const TrajectoryPoints & input,
    TrajectoryPoints & output, std::vector<TrajectoryPoints> & debug, const bool publish) override
  {
    smoothing_input = input;
    return JerkFilteredSmoother::apply(v0, a0, input, output, debug, publish);
  }
};

class ExternalLimitPrediction : public testing::Test
{
protected:
  void SetUp() override
  {
    setJerkFilteredSmootherParameters(node_handle);
    adapter = std::make_unique<rclcpp::Node>(node_handle);
    smoother = std::make_shared<InspectingSmoother>(
      *adapter, std::make_shared<autoware_utils_debug::TimeKeeper>());
    auto pose = std::make_shared<geometry_msgs::PoseStamped>();
    pose->pose.orientation.w = 1.0;
    data.current_odometry = pose;
    auto velocity = std::make_shared<geometry_msgs::TwistStamped>();
    velocity->twist.linear.x = 8.0;
    data.current_velocity = velocity;
    data.current_acceleration = std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    data.velocity_smoother_ = smoother;
  }

  double firstStopDistance() const
  {
    for (const auto & p : smoother->smoothing_input) {
      if (std::abs(p.longitudinal_velocity_mps) < 1e-3) return p.pose.position.x;
    }
    return std::numeric_limits<double>::infinity();
  }
  ros::NodeHandle node_handle{"/external_limit_prediction_oracle"};
  std::unique_ptr<rclcpp::Node> adapter;
  std::shared_ptr<InspectingSmoother> smoother;
  PlannerData data;
};
}  // namespace

TEST_F(ExternalLimitPrediction, ZeroLimitPreservesMovingPrefixAndStopsAhead)
{
  autoware_internal_planning_msgs::VelocityLimit limit;
  limit.max_velocity = 0.0;
  data.external_velocity_limit = limit;
  autoware_internal_planning_msgs::PathWithLaneId out;
  ASSERT_TRUE(smoothPath(makePath(), out, data));
  ASSERT_FALSE(smoother->smoothing_input.empty());
  EXPECT_NEAR(smoother->smoothing_input.front().longitudinal_velocity_mps, 8.0, 1e-3);
  EXPECT_GT(firstStopDistance(), 50.0);  // Weak default braking, not a stop at the ego pose.
  EXPECT_LT(firstStopDistance(), 100.0);
  EXPECT_NEAR(out.points.front().point.longitudinal_velocity_mps, 8.0, 0.05);
  EXPECT_NEAR(out.points.back().point.longitudinal_velocity_mps, 0.0, 0.01);
}

TEST_F(ExternalLimitPrediction, RequestedConstraintsChangeStopInsertionDistance)
{
  autoware_internal_planning_msgs::VelocityLimit limit;
  limit.max_velocity = 0.0;
  data.external_velocity_limit = limit;
  autoware_internal_planning_msgs::PathWithLaneId out;
  ASSERT_TRUE(smoothPath(makePath(), out, data));
  const double default_distance = firstStopDistance();
  limit.use_constraints = true;
  limit.constraints.min_acceleration = -2.0;
  limit.constraints.max_jerk = 2.0;
  limit.constraints.min_jerk = -2.0;
  data.external_velocity_limit = limit;
  ASSERT_TRUE(smoothPath(makePath(), out, data));
  EXPECT_LT(firstStopDistance(), default_distance);
  EXPECT_GT(firstStopDistance(), 10.0);
  EXPECT_NEAR(smoother->smoothing_input.front().longitudinal_velocity_mps, 8.0, 1e-3);
}

TEST_F(ExternalLimitPrediction, NoExternalLimitLeavesInputStopUnchanged)
{
  autoware_internal_planning_msgs::PathWithLaneId out;
  ASSERT_TRUE(smoothPath(makePath(), out, data));
  EXPECT_NEAR(firstStopDistance(), 100.0, 0.2);
}

TEST_F(ExternalLimitPrediction, InvalidConstraintsDoNotPublishPartiallyLimitedOutput)
{
  autoware_internal_planning_msgs::VelocityLimit limit;
  limit.max_velocity = 0.0;
  limit.use_constraints = true;
  limit.constraints.min_acceleration = -2.0;
  limit.constraints.max_jerk = 0.0;
  limit.constraints.min_jerk = -2.0;
  data.external_velocity_limit = limit;
  autoware_internal_planning_msgs::PathWithLaneId out;
  EXPECT_FALSE(smoothPath(makePath(), out, data));
  EXPECT_TRUE(out.points.empty());
  EXPECT_TRUE(smoother->smoothing_input.empty());
}

TEST_F(ExternalLimitPrediction, ShortHorizonPreservesPrefixWithoutDroppingLimit)
{
  auto path = makePath();
  path.points.resize(11);
  autoware_internal_planning_msgs::VelocityLimit limit;
  limit.max_velocity = 0.0;
  data.external_velocity_limit = limit;
  autoware_internal_planning_msgs::PathWithLaneId out;
  ASSERT_TRUE(smoothPath(path, out, data));
  EXPECT_NEAR(smoother->smoothing_input.front().longitudinal_velocity_mps, 8.0, 1e-3);
  EXPECT_NEAR(firstStopDistance(), 10.0, 0.2);
}

TEST_F(ExternalLimitPrediction, AnalyticalPredictionPreservesMovingInitialSpeed)
{
  // Production algorithm and the matched acceleration/jerk configuration.
  for (const auto & entry : std::vector<std::pair<std::string, double>>{
         {"normal/max_acc", 3.7}, {"normal/min_acc", -6.0},
         {"normal/max_jerk", 100.0}, {"normal/min_jerk", -10.0},
         {"resample/ds_resample", 0.1}, {"resample/delta_yaw_threshold", 0.785},
         {"latacc/constant_velocity_dist_threshold", 2.0},
         {"forward/max_acc", 3.7}, {"forward/min_acc", -6.0},
         {"forward/max_jerk", 100.0}, {"forward/min_jerk", -10.0}, {"forward/kp", 0.3},
         {"backward/start_jerk", -0.1}, {"backward/min_jerk_mild_stop", -0.3},
         {"backward/min_jerk", -10.0}, {"backward/min_acc_mild_stop", -1.0},
         {"backward/min_acc", -6.0}, {"backward/span_jerk", -0.01}}) {
    node_handle.setParam(entry.first, entry.second);
  }
  node_handle.setParam("resample/num_resample", 1);
  node_handle.setParam("latacc/enable_constant_velocity_while_turning", false);
  node_handle.setParam("lateral_acceleration_limits", std::vector<double>{6.0, 6.0, 6.0, 6.0});
  node_handle.setParam("min_decel_for_lateral_acc_lim_filter", -6.0);
  data.velocity_smoother_ =
    std::make_shared<autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother>(
      *adapter, std::make_shared<autoware_utils_debug::TimeKeeper>());
  autoware_internal_planning_msgs::VelocityLimit limit;
  limit.max_velocity = 0.0;
  data.external_velocity_limit = limit;
  autoware_internal_planning_msgs::PathWithLaneId out;
  ASSERT_TRUE(smoothPath(makePath(), out, data));
  ASSERT_FALSE(out.points.empty());
  EXPECT_NEAR(out.points.front().point.longitudinal_velocity_mps, 8.0, 0.05);
  EXPECT_NEAR(out.points.back().point.longitudinal_velocity_mps, 0.0, 0.01);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "trajectory_utils_oracle_gtest", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
