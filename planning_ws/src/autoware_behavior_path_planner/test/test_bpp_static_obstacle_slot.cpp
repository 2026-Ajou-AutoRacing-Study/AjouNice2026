#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cstdlib>
#include <string>

namespace
{
autoware::behavior_path_planner::BppInputState readyState()
{
  autoware::behavior_path_planner::BppInputState state;
  state.route_received = true;
  state.vector_map_received = true;
  state.scenario_received = true;
  state.acceleration_received = true;
  state.operation_mode_received = true;
  state.perception_received_at_seconds = 10.0;
  state.odometry_received_at_seconds = 10.0;
  state.occupancy_grid_received_at_seconds = 10.0;
  return state;
}

TEST(BehaviorPathPlannerCore, EnablesStaticObstacleOnlyWithRenderedParameters)
{
  constexpr const char * kNamespace = "/bpp_static_obstacle_slot";
  const std::string load =
    std::string("rosparam load ") + STATIC_OBSTACLE_AVOIDANCE_CONFIG + " " + kNamespace;
  ASSERT_EQ(std::system(load.c_str()), 0);
  ros::NodeHandle private_node(kNamespace);
  private_node.setParam("enable_static_obstacle_avoidance_module", true);
  private_node.setParam("side_shift/min_distance_to_start_shifting", 1.0);
  private_node.setParam("side_shift/min_shifting_distance", 2.0);
  private_node.setParam("side_shift/min_shifting_speed", 1.0);
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
  EXPECT_TRUE(core.isInitialized());
  EXPECT_TRUE(core.staticObstacleAvoidanceEnabled());
  EXPECT_EQ(core.activeModuleCount(), 0U);
}

TEST(BehaviorPathPlannerCore, StaticObstacleFailsClosedForMissingSceneInputs)
{
  constexpr const char * kNamespace = "/bpp_static_obstacle_input_boundary";
  const std::string load =
    std::string("rosparam load ") + STATIC_OBSTACLE_AVOIDANCE_CONFIG + " " + kNamespace;
  ASSERT_EQ(std::system(load.c_str()), 0);
  ros::NodeHandle private_node(kNamespace);
  private_node.setParam("enable_static_obstacle_avoidance_module", true);
  private_node.setParam("side_shift/min_distance_to_start_shifting", 1.0);
  private_node.setParam("side_shift/min_shifting_distance", 2.0);
  private_node.setParam("side_shift/min_shifting_speed", 1.0);
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
  ASSERT_TRUE(core.isInitialized());
  ASSERT_TRUE(core.staticObstacleAvoidanceEnabled());

  auto data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  data->self_odometry = boost::make_shared<nav_msgs::Odometry>();
  data->self_acceleration = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  autoware::behavior_path_planner::BehaviorModuleOutput input;
  // The safety boundary must reject a received-but-null perception payload.
  EXPECT_FALSE(core.run(data, input, readyState(), 10.0, 1.0, false));
  // An explicit empty message is valid, but it still requires a route context.
  data->dynamic_object = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  EXPECT_FALSE(core.run(data, input, readyState(), 10.0, 1.0, false));
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bpp_static_obstacle_slot_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
