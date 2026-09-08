#include <autoware_freespace_planner/freespace_planner_node.hpp>

#include <gtest/gtest.h>

#include <atomic>

namespace
{
TEST(FreespacePlannerRos1Node, SuppressesOutsideParkingThenPublishesFreeSpacePartialTrajectory)
{
  ros::NodeHandle node;
  ros::NodeHandle planner_node("/freespace_transport_oracle");
  for (const auto & entry : {std::pair{"wheel_radius", 0.3}, {"wheel_width", 0.2},
                            {"wheel_base", 2.7}, {"wheel_tread", 1.5},
                            {"front_overhang", 0.9}, {"rear_overhang", 1.0},
                            {"left_overhang", 0.2}, {"right_overhang", 0.2},
                            {"vehicle_height", 1.6}, {"max_steer_angle", 0.6}}) {
    planner_node.setParam(entry.first, entry.second);
  }
  planner_node.setParam("update_rate", 20.0);
  planner_node.setParam("waypoints_velocity", 5.0);
  planner_node.setParam("replan_when_course_out", false);
  planner_node.setParam("astar/search_method", std::string("forward"));
  planner_node.setParam("astar/only_behind_solutions", false);
  planner_node.setParam("astar/use_back", true);
  planner_node.setParam("astar/adapt_expansion_distance", true);
  planner_node.setParam("astar/expansion_distance", 0.4);
  planner_node.setParam("astar/near_goal_distance", 4.0);
  planner_node.setParam("astar/distance_heuristic_weight", 2.0);
  planner_node.setParam("astar/smoothness_weight", 0.5);
  planner_node.setParam("astar/obstacle_distance_weight", 1.7);
  planner_node.setParam("astar/goal_lat_distance_weight", 1.0);
  autoware::freespace_planner::FreespacePlannerNode planner(planner_node, planner_node);
  std::atomic<int> output_count{0};
  std::atomic<bool> completed{false};
  autoware_planning_msgs::Trajectory output;
  const auto output_subscriber = node.subscribe<autoware_planning_msgs::Trajectory>(
    "/freespace_transport_oracle/output/trajectory", 1,
    [&output_count, &output](const autoware_planning_msgs::TrajectoryConstPtr & message) {
      output = *message;
      ++output_count;
    });
  const auto completed_subscriber = node.subscribe<std_msgs::Bool>(
    "/freespace_transport_oracle/is_completed", 1,
    [&completed](const std_msgs::BoolConstPtr & message) { completed.store(message->data); });
  const auto route_publisher = node.advertise<autoware_planning_msgs::LaneletRoute>(
    "/freespace_transport_oracle/input/route", 1, true);
  const auto map_publisher = node.advertise<nav_msgs::OccupancyGrid>(
    "/freespace_transport_oracle/input/occupancy_grid", 1, true);
  const auto scenario_publisher = node.advertise<autoware_internal_planning_msgs::Scenario>(
    "/freespace_transport_oracle/input/scenario", 1, true);
  const auto odometry_publisher = node.advertise<nav_msgs::Odometry>(
    "/freespace_transport_oracle/input/odometry", 10, true);

  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.goal_pose.position.x = 4.0;
  route.goal_pose.orientation.w = 1.0;
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.2F;
  map.info.width = 100;
  map.info.height = 100;
  map.info.origin.position.x = -10.0;
  map.info.origin.position.y = -10.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.orientation.w = 1.0;
  autoware_internal_planning_msgs::Scenario inactive;
  inactive.current_scenario = autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  autoware_internal_planning_msgs::Scenario parking;
  parking.current_scenario = autoware_internal_planning_msgs::Scenario::PARKING;
  parking.activating_scenarios.push_back(autoware_internal_planning_msgs::Scenario::PARKING);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  for (int i = 0; i < 8; ++i) {
    route_publisher.publish(route);
    map_publisher.publish(map);
    odometry_publisher.publish(odometry);
    scenario_publisher.publish(inactive);
    ros::WallDuration(0.05).sleep();
  }
  EXPECT_EQ(output_count.load(), 0);
  const auto deadline = ros::WallTime::now() + ros::WallDuration(5.0);
  while (ros::ok() && ros::WallTime::now() < deadline && output_count.load() == 0) {
    route_publisher.publish(route);
    map_publisher.publish(map);
    odometry_publisher.publish(odometry);
    scenario_publisher.publish(parking);
    ros::WallDuration(0.05).sleep();
  }
  ASSERT_GT(output_count.load(), 0);
  ASSERT_FALSE(output.points.empty());
  EXPECT_FLOAT_EQ(output.points.back().longitudinal_velocity_mps, 0.0F);
  const auto initial_front_x = output.points.front().pose.position.x;
  const auto outputs_before_course_out = output_count.load();
  odometry.pose.pose.position.x = -5.0;
  for (int i = 0; i < 12; ++i) {
    odometry_publisher.publish(odometry);
    scenario_publisher.publish(parking);
    ros::WallDuration(0.05).sleep();
  }
  ASSERT_GT(output_count.load(), outputs_before_course_out);
  ASSERT_FALSE(output.points.empty());
  EXPECT_NEAR(output.points.front().pose.position.x, initial_front_x, 0.5);
  odometry.pose.pose = output.points.back().pose;
  const auto completion_deadline = ros::WallTime::now() + ros::WallDuration(3.0);
  while (ros::ok() && ros::WallTime::now() < completion_deadline && !completed.load()) {
    odometry_publisher.publish(odometry);
    scenario_publisher.publish(parking);
    ros::WallDuration(0.05).sleep();
  }
  EXPECT_TRUE(completed.load());
  spinner.stop();
  (void)output_subscriber;
  (void)completed_subscriber;
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "freespace_planner_ros1_node_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
