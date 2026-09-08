#include <autoware/behavior_path_planner/behavior_path_planner_ros1_node.hpp>
#include <autoware/behavior_velocity_planner/node.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_velocity_planner/motion_velocity_planner_ros1_node.hpp>
#include <autoware/planning_topic_converter/path_to_trajectory.hpp>
#include <gtest/gtest.h>
#include <autoware/mission_planner_universe/mission_planner_initialization_ros1.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <atomic>
#include <cstdlib>

namespace
{
void setVehicleInfo(ros::NodeHandle & node, const std::string & prefix)
{
  for (const auto & entry : {std::pair{"wheel_radius", 0.5}, {"wheel_width", 0.3},
                             {"wheel_base", 2.7}, {"wheel_tread", 1.6},
                             {"front_overhang", 1.0}, {"rear_overhang", 1.1},
                             {"left_overhang", 0.2}, {"right_overhang", 0.2},
                             {"vehicle_height", 1.8}, {"max_steer_angle", 0.6}}) {
    node.setParam(prefix + entry.first, entry.second);
  }
}

TEST(MissionBppRoutePipeline, MissionServiceRouteDrivesBppPath)
{
  ros::NodeHandle node;
  const bool require_validator_output = std::getenv("AW149_REQUIRE_VALIDATOR_OUTPUT") != nullptr;
  const std::string mission_ns = "/mission_bpp_pipeline/mission/";
  const std::string bpp_ns = "/mission_bpp_pipeline/bpp/";
  setVehicleInfo(node, mission_ns);
  setVehicleInfo(node, bpp_ns);
  node.setParam(bpp_ns + "max_accel", 1.2);
  node.setParam(bpp_ns + "max_jerk", 0.3);
  node.setParam(bpp_ns + "system_delay", 0.4);
  node.setParam(bpp_ns + "delay_response_time", 0.5);
  setVehicleInfo(node, "/mission_bpp_route_pipeline_test/");
  node.setParam("/mission_bpp_route_pipeline_test/max_accel", 1.2);
  node.setParam("/mission_bpp_route_pipeline_test/max_jerk", 0.3);
  node.setParam("/mission_bpp_route_pipeline_test/system_delay", 0.4);
  node.setParam("/mission_bpp_route_pipeline_test/delay_response_time", 0.5);

  ros::NodeHandle mission_private(mission_ns);
  ros::NodeHandle bpp_private(bpp_ns);
  ros::NodeHandle converter_private("/mission_bpp_pipeline/converter");
  ros::NodeHandle mvp_private("/mission_bpp_pipeline/mvp");
  converter_private.setParam("input_topic", "/mission_bpp_route_pipeline_test/output/path");
  converter_private.setParam("output_topic", "/mission_bpp_pipeline/mvp/input/trajectory");
  autoware::mission_planner_universe::MissionPlannerInitializationRos1 mission(node, mission_private);
  autoware::behavior_path_planner::BehaviorPathPlannerRos1Node bpp(node, bpp_private);
  autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode bvp(node);
  autoware::planning_topic_converter::PathToTrajectory converter(node, converter_private);
  autoware::motion_velocity_planner::MotionVelocityPlannerRos1Node mvp(mvp_private);

  std::atomic<size_t> routed_segments{0};
  std::atomic<size_t> bpp_points{0};
  const auto bpp_route = node.advertise<autoware_planning_msgs::LaneletRoute>(bpp_ns + "input/route", 1, true);
  const auto validator_route = node.advertise<autoware_planning_msgs::LaneletRoute>(
    "/planning/mission_planning/route", 1, true);
  const auto route_bridge = node.subscribe<autoware_planning_msgs::LaneletRoute>(
    mission_ns + "route", 1, [&bpp_route, &validator_route, &routed_segments](const autoware_planning_msgs::LaneletRouteConstPtr & route) {
      routed_segments = route->segments.size(); bpp_route.publish(*route); validator_route.publish(*route);
    });
  const auto bpp_output = node.subscribe<autoware_internal_planning_msgs::PathWithLaneId>(
    bpp_ns + "output/path", 1, [&bpp_points](const autoware_internal_planning_msgs::PathWithLaneIdConstPtr & path) {
      bpp_points = path->points.size();
    });
  const auto bvp_input = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/mission_bpp_route_pipeline_test/input/path_with_lane_id", 1);
  const auto bpp_to_bvp = node.subscribe<autoware_internal_planning_msgs::PathWithLaneId>(
    bpp_ns + "output/path", 1, [&bvp_input](const autoware_internal_planning_msgs::PathWithLaneIdConstPtr & path) {
      bvp_input.publish(*path);
    });
  std::atomic<size_t> trajectory_points{0};
  const auto mvp_output = node.subscribe<autoware_planning_msgs::Trajectory>(
    "/mission_bpp_pipeline/mvp/output/trajectory", 1,
    [&trajectory_points](const autoware_planning_msgs::TrajectoryConstPtr & trajectory) {
      trajectory_points = trajectory->points.size();
    });
  std::atomic<size_t> validated_trajectory_points{0};
  const auto validator_output = node.subscribe<autoware_planning_msgs::Trajectory>(
    "/planning/trajectory", 1, [&validated_trajectory_points](const autoware_planning_msgs::TrajectoryConstPtr & trajectory) {
      validated_trajectory_points = trajectory->points.size();
    });

  const auto mission_map = node.advertise<autoware_map_msgs::LaneletMapBin>(mission_ns + "input/vector_map", 1, true);
  const auto bpp_map = node.advertise<autoware_map_msgs::LaneletMapBin>(bpp_ns + "input/vector_map", 1, true);
  const auto mission_odom = node.advertise<nav_msgs::Odometry>(mission_ns + "input/odometry", 1, true);
  const auto scenario = node.advertise<autoware_internal_planning_msgs::Scenario>(bpp_ns + "input/scenario", 1, true);
  const auto acceleration = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(bpp_ns + "input/acceleration", 1, true);
  const auto operation_mode = node.advertise<autoware_adapi_v1_msgs::OperationModeState>(bpp_ns + "input/operation_mode_state", 1, true);
  const auto objects = node.advertise<autoware_perception_msgs::PredictedObjects>(bpp_ns + "input/objects", 1, true);
  const auto bpp_odom = node.advertise<nav_msgs::Odometry>(bpp_ns + "input/odometry", 1, true);
  const auto mvp_odom = node.advertise<nav_msgs::Odometry>("/mission_bpp_pipeline/mvp/input/odometry", 1, true);
  const auto mvp_acceleration = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/mission_bpp_pipeline/mvp/input/acceleration", 1, true);
  const auto validator_map = node.advertise<autoware_map_msgs::LaneletMapBin>("/map/vector_map", 1, true);
  const auto validator_odom = node.advertise<nav_msgs::Odometry>("/localization/kinematic_state", 1, true);
  const auto validator_acceleration = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/localization/acceleration", 1, true);
  const auto validator_pointcloud = node.advertise<sensor_msgs::PointCloud2>(
    "/perception/obstacle_segmentation/pointcloud", 1, true);
  const auto grid = node.advertise<nav_msgs::OccupancyGrid>(bpp_ns + "input/occupancy_grid", 1, true);

  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  ASSERT_GE(lane.centerline().size(), 2U);
  auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  // overlap_map.osm MetaInfo, as emitted by the source Lanelet2 map loader.
  map_message.header.frame_id = "map";
  map_message.version_map_format = "1";
  map_message.version_map = "5";
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = lane.centerline().front().x();
  odometry.pose.pose.position.y = lane.centerline().front().y();
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 2.0;
  autoware_internal_planning_msgs::Scenario lane_driving;
  lane_driving.current_scenario = autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  autoware_adapi_v1_msgs::OperationModeState local_mode;
  local_mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  geometry_msgs::AccelWithCovarianceStamped accel;
  autoware_perception_msgs::PredictedObjects predicted_objects;
  nav_msgs::OccupancyGrid occupancy_grid;
  occupancy_grid.header.frame_id = "map";
  sensor_msgs::PointCloud2 pointcloud;
  pointcloud.header.frame_id = "map";

  ros::AsyncSpinner spinner(3);
  spinner.start();
  for (int i = 0; ros::ok() && i < 20; ++i) {
    mission_map.publish(map_message); bpp_map.publish(map_message); mission_odom.publish(odometry); bpp_odom.publish(odometry); mvp_odom.publish(odometry); validator_map.publish(map_message); validator_odom.publish(odometry);
    scenario.publish(lane_driving); acceleration.publish(accel); operation_mode.publish(local_mode);
    objects.publish(predicted_objects); grid.publish(occupancy_grid); mvp_acceleration.publish(accel); validator_acceleration.publish(accel); validator_pointcloud.publish(pointcloud);
    ros::WallDuration(0.05).sleep();
  }

  auto set_route = node.serviceClient<autoware_planning_msgs::SetLaneletRoute>(mission_ns + "set_lanelet_route");
  ASSERT_TRUE(set_route.waitForExistence(ros::Duration(2.0)));
  autoware_planning_msgs::SetLaneletRoute request;
  request.request.header.frame_id = "map";
  request.request.goal_pose = odometry.pose.pose;
  request.request.goal_pose.position.x = lane.centerline().back().x();
  request.request.goal_pose.position.y = lane.centerline().back().y();
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  request.request.segments.push_back(segment);
  ASSERT_TRUE(set_route.call(request));
  ASSERT_TRUE(request.response.status.success);

  const auto deadline = ros::WallTime::now() + ros::WallDuration(4.0);
  while (ros::ok() && ros::WallTime::now() < deadline && (trajectory_points.load() == 0U || (require_validator_output && validated_trajectory_points.load() == 0U))) {
    mission_map.publish(map_message); bpp_map.publish(map_message); mission_odom.publish(odometry); bpp_odom.publish(odometry); mvp_odom.publish(odometry); validator_map.publish(map_message); validator_odom.publish(odometry);
    scenario.publish(lane_driving); acceleration.publish(accel); operation_mode.publish(local_mode);
    objects.publish(predicted_objects); grid.publish(occupancy_grid); mvp_acceleration.publish(accel); validator_acceleration.publish(accel); validator_pointcloud.publish(pointcloud);
    ros::WallDuration(0.05).sleep();
  }
  EXPECT_GT(routed_segments.load(), 0U);
  EXPECT_GT(bpp_points.load(), 2U);
  EXPECT_GT(trajectory_points.load(), 2U);
  if (require_validator_output) EXPECT_GT(validated_trajectory_points.load(), 2U);
  spinner.stop();
  (void)route_bridge; (void)bpp_output; (void)bpp_to_bvp; (void)mvp_output; (void)validator_output;
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mission_bpp_route_pipeline_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
