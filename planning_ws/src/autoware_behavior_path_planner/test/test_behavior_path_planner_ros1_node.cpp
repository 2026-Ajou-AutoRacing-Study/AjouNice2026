#include <autoware/behavior_path_planner/behavior_path_planner_ros1_node.hpp>

#include "ros1_side_shift_fixture.hpp"
#include <autoware/lanelet2_utils/conversion.hpp>
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
#include <autoware_test_utils/mock_data_parser.hpp>
#endif

#include <lanelet2_core/LaneletMap.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>

namespace
{
TEST(BehaviorPathPlannerRos1Node, SuppressesUntilReadyThenPublishesLaneDrivingPath)
{
  ros::NodeHandle node;
  ros::NodeHandle planner_node("/bpp_transport_oracle");
  aw149::test::loadSideShiftFixture(planner_node);
  // VehicleInfoUtils is part of the node's mandatory ROS1 startup contract.
  // Supply the same bounded geometry used by the pipeline fixtures before
  // constructing the node, rather than allowing a zero-footprint fallback.
  planner_node.setParam("wheel_radius", 0.5);
  planner_node.setParam("wheel_width", 0.3);
  planner_node.setParam("wheel_base", 2.7);
  planner_node.setParam("wheel_tread", 1.6);
  planner_node.setParam("front_overhang", 1.0);
  planner_node.setParam("rear_overhang", 1.1);
  planner_node.setParam("left_overhang", 0.2);
  planner_node.setParam("right_overhang", 0.2);
  planner_node.setParam("vehicle_height", 1.8);
  planner_node.setParam("max_steer_angle", 0.6);
  autoware::behavior_path_planner::BehaviorPathPlannerRos1Node planner(node, planner_node);
  std::atomic<int> output_count{0};
  size_t output_points = 0;
  std::string output_frame;
  const auto output_subscriber = node.subscribe<autoware_internal_planning_msgs::PathWithLaneId>(
    "/bpp_transport_oracle/output/path", 1,
    [&output_count, &output_points, &output_frame](const autoware_internal_planning_msgs::PathWithLaneIdConstPtr & path) {
      output_points = path->points.size();
      output_frame = path->header.frame_id;
      ++output_count;
    });
  const auto route_publisher = node.advertise<autoware_planning_msgs::LaneletRoute>(
    "/bpp_transport_oracle/input/route", 1, true);
  const auto map_publisher = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/bpp_transport_oracle/input/vector_map", 1, true);
  const auto scenario_publisher = node.advertise<autoware_internal_planning_msgs::Scenario>(
    "/bpp_transport_oracle/input/scenario", 1, true);
  const auto acceleration_publisher = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/bpp_transport_oracle/input/acceleration", 1, true);
  const auto operation_mode_publisher = node.advertise<autoware_adapi_v1_msgs::OperationModeState>(
    "/bpp_transport_oracle/input/operation_mode_state", 1, true);
  const auto perception_publisher = node.advertise<autoware_perception_msgs::PredictedObjects>(
    "/bpp_transport_oracle/input/objects", 1, true);
  const auto odometry_publisher = node.advertise<nav_msgs::Odometry>(
    "/bpp_transport_oracle/input/odometry", 1, true);
  const auto occupancy_publisher = node.advertise<nav_msgs::OccupancyGrid>(
    "/bpp_transport_oracle/input/occupancy_grid", 1, true);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::WallDuration(0.2).sleep();
  EXPECT_EQ(output_count.load(), 0);

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  const auto centerline = lane.centerline();
  ASSERT_GE(centerline.size(), 2U);
  autoware_map_msgs::LaneletMapBin map_message =
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  // Keep the fixture message equivalent to the source map-loader output.  The
  // generic serializer owns only binary data; map-loader adds this OSM's
  // frame and parsed MetaInfo versions before publishing.
  map_message.header.frame_id = "map";
  map_message.version_map_format = "1";
  map_message.version_map = "5";
  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = centerline.front().x();
  route.start_pose.position.y = centerline.front().y();
  route.goal_pose.position.x = centerline.back().x();
  route.goal_pose.position.y = centerline.back().y();
  route.start_pose.orientation.w = 1.0;
  route.goal_pose.orientation.w = 1.0;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  route.segments.push_back(segment);
  autoware_internal_planning_msgs::Scenario scenario;
  scenario.current_scenario = autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware_adapi_v1_msgs::OperationModeState operation_mode;
  operation_mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  autoware_perception_msgs::PredictedObjects perception;
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose = route.start_pose;
  odometry.twist.twist.linear.x = 2.0;
  nav_msgs::OccupancyGrid occupancy;
  occupancy.header.frame_id = "map";

  const auto deadline = ros::WallTime::now() + ros::WallDuration(3.0);
  while (ros::ok() && ros::WallTime::now() < deadline && output_count.load() == 0) {
    map_publisher.publish(map_message);
    route_publisher.publish(route);
    scenario_publisher.publish(scenario);
    acceleration_publisher.publish(acceleration);
    operation_mode_publisher.publish(operation_mode);
    perception_publisher.publish(perception);
    odometry_publisher.publish(odometry);
    occupancy_publisher.publish(occupancy);
    ros::WallDuration(0.05).sleep();
  }
  EXPECT_GT(output_count.load(), 0);
  EXPECT_GT(output_points, 2U);
  EXPECT_EQ(output_frame, "map");
  spinner.stop();
  (void)output_subscriber;
}

#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
TEST(BehaviorPathPlannerRos1Node, CommittedSourceFixtureDoesNotInventAvoidanceRequest)
{
  ros::NodeHandle node;
  constexpr const char * kNamespace = "/bpp_static_obstacle_transport_oracle";
  ASSERT_EQ(
    std::system((std::string("rosparam load ") + STATIC_OBSTACLE_AVOIDANCE_CONFIG + " " + kNamespace).c_str()),
    0);
  ros::NodeHandle planner_node(kNamespace);
  aw149::test::loadSideShiftFixture(planner_node);
  planner_node.setParam("enable_static_obstacle_avoidance_module", true);
  planner_node.setParam("side_shift/min_distance_to_start_shifting", 1.0);
  planner_node.setParam("side_shift/min_shifting_distance", 2.0);
  planner_node.setParam("side_shift/min_shifting_speed", 1.0);
  planner_node.setParam("wheel_radius", 0.5);
  planner_node.setParam("wheel_width", 0.3);
  planner_node.setParam("wheel_base", 2.7);
  planner_node.setParam("wheel_tread", 1.6);
  planner_node.setParam("front_overhang", 1.0);
  planner_node.setParam("rear_overhang", 1.1);
  planner_node.setParam("left_overhang", 0.2);
  planner_node.setParam("right_overhang", 0.2);
  planner_node.setParam("vehicle_height", 1.8);
  planner_node.setParam("max_steer_angle", 0.6);
  autoware::behavior_path_planner::BehaviorPathPlannerRos1Node planner(node, planner_node);
  std::atomic<int> output_count{0};
  const auto output_subscriber = node.subscribe<autoware_internal_planning_msgs::PathWithLaneId>(
    "/bpp_static_obstacle_transport_oracle/output/path", 1,
    [&output_count](const autoware_internal_planning_msgs::PathWithLaneIdConstPtr &) { ++output_count; });
  const auto route_publisher = node.advertise<autoware_planning_msgs::LaneletRoute>(
    "/bpp_static_obstacle_transport_oracle/input/route", 1, true);
  const auto map_publisher = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/bpp_static_obstacle_transport_oracle/input/vector_map", 1, true);
  const auto scenario_publisher = node.advertise<autoware_internal_planning_msgs::Scenario>(
    "/bpp_static_obstacle_transport_oracle/input/scenario", 1, true);
  const auto acceleration_publisher = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/bpp_static_obstacle_transport_oracle/input/acceleration", 1, true);
  const auto operation_mode_publisher = node.advertise<autoware_adapi_v1_msgs::OperationModeState>(
    "/bpp_static_obstacle_transport_oracle/input/operation_mode_state", 1, true);
  const auto perception_publisher = node.advertise<autoware_perception_msgs::PredictedObjects>(
    "/bpp_static_obstacle_transport_oracle/input/objects", 1, true);
  const auto odometry_publisher = node.advertise<nav_msgs::Odometry>(
    "/bpp_static_obstacle_transport_oracle/input/odometry", 1, true);
  const auto occupancy_publisher = node.advertise<nav_msgs::OccupancyGrid>(
    "/bpp_static_obstacle_transport_oracle/input/occupancy_grid", 1, true);

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_STATIC_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  map_message.header.frame_id = "map";
  map_message.version_map_format = "1";
  map_message.version_map = "5";
  const lanelet::Ids route_ids{3008384, 3008383, 3008382};
  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = 1034.4942626953125;
  route.start_pose.position.y = 1065.66455078125;
  route.start_pose.position.z = 100.0;
  route.start_pose.orientation.z = -0.9999274638548634;
  route.start_pose.orientation.w = 0.012044377475855648;
  route.goal_pose.position.x = 932.8187255859375;
  route.goal_pose.position.y = 1062.9110107421875;
  route.goal_pose.position.z = 100.0;
  route.goal_pose.orientation.z = 0.9993430668091411;
  route.goal_pose.orientation.w = -0.03624134131762842;
  for (const auto id : route_ids) {
    autoware_planning_msgs::LaneletSegment segment;
    segment.preferred_primitive.id = id;
    segment.primitives.push_back(segment.preferred_primitive);
    route.segments.push_back(segment);
  }
  const auto perception = autoware::test_utils::parse<autoware_perception_msgs::PredictedObjects>(
    YAML::LoadFile(AW149_STATIC_OBJECT_FIXTURE_PATH));
  ASSERT_FALSE(perception.objects.empty());
  autoware_internal_planning_msgs::Scenario scenario;
  scenario.current_scenario = autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware_adapi_v1_msgs::OperationModeState operation_mode;
  operation_mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose = route.start_pose;
  nav_msgs::OccupancyGrid occupancy;
  occupancy.header.frame_id = "map";

  ros::AsyncSpinner spinner(2);
  spinner.start();
  const auto deadline = ros::WallTime::now() + ros::WallDuration(3.0);
  while (ros::ok() && ros::WallTime::now() < deadline && output_count.load() == 0) {
    map_publisher.publish(map_message);
    route_publisher.publish(route);
    scenario_publisher.publish(scenario);
    acceleration_publisher.publish(acceleration);
    operation_mode_publisher.publish(operation_mode);
    perception_publisher.publish(perception);
    odometry_publisher.publish(odometry);
    occupancy_publisher.publish(occupancy);
    ros::WallDuration(0.05).sleep();
  }
  EXPECT_GT(output_count.load(), 0);
  EXPECT_TRUE(planner.staticObstacleAvoidanceEnabled());
  EXPECT_EQ(planner.activeModuleCount(), 0U);
  spinner.stop();
  (void)output_subscriber;
}
#endif
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "behavior_path_planner_ros1_node_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
