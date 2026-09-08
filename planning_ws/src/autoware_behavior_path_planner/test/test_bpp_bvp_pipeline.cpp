#include <autoware/behavior_path_planner/behavior_path_planner_ros1_node.hpp>

#include "ros1_side_shift_fixture.hpp"
#include <autoware/behavior_velocity_planner/node.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <gtest/gtest.h>
#include <atomic>

namespace {
class Receiver {
public:
  void callback(const autoware_planning_msgs::Path::ConstPtr & message) { message_ = message; }
  autoware_planning_msgs::Path::ConstPtr message_;
};
void setVehicleInfo(ros::NodeHandle & node, const std::string & prefix) {
  node.setParam(prefix + "wheel_radius", 0.5); node.setParam(prefix + "wheel_width", 0.3);
  node.setParam(prefix + "wheel_base", 2.7); node.setParam(prefix + "wheel_tread", 1.6);
  node.setParam(prefix + "front_overhang", 1.0); node.setParam(prefix + "rear_overhang", 1.1);
  node.setParam(prefix + "left_overhang", 0.2); node.setParam(prefix + "right_overhang", 0.2);
  node.setParam(prefix + "vehicle_height", 1.8); node.setParam(prefix + "max_steer_angle", 0.6);
  node.setParam(prefix + "max_accel", 1.2); node.setParam(prefix + "max_jerk", 0.3);
  node.setParam(prefix + "system_delay", 0.4); node.setParam(prefix + "delay_response_time", 0.5);
}
TEST(BppBvpPipeline, BridgesReadyBppPathIntoBvpOutput) {
  ros::NodeHandle node;
  setVehicleInfo(node, "/bpp_bvp_pipeline_test/");
  setVehicleInfo(node, "/bpp_bvp_pipeline/bpp/");
  ros::NodeHandle bpp_private("/bpp_bvp_pipeline/bpp");
  aw149::test::loadSideShiftFixture(bpp_private);
  ros::NodeHandle bvp_private("/bpp_bvp_pipeline_test");
  aw149::test::loadBvpSmootherFixture(bvp_private);
  autoware::behavior_path_planner::BehaviorPathPlannerRos1Node bpp(node, bpp_private);
  autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode bvp(node);
  const auto bvp_input = node.advertise<autoware_internal_planning_msgs::PathWithLaneId>(
    "/bpp_bvp_pipeline_test/input/path_with_lane_id", 1);
  std::atomic<size_t> bpp_points{0};
  const auto bridge = node.subscribe<autoware_internal_planning_msgs::PathWithLaneId>(
    "/bpp_bvp_pipeline/bpp/output/path", 1,
    [&bvp_input, &bpp_points](const autoware_internal_planning_msgs::PathWithLaneIdConstPtr & path) {
      bpp_points = path->points.size(); bvp_input.publish(*path);
    });
  Receiver receiver;
  const auto output = node.subscribe("/bpp_bvp_pipeline_test/output/path", 1, &Receiver::callback, &receiver);
  const auto route_pub = node.advertise<autoware_planning_msgs::LaneletRoute>("/bpp_bvp_pipeline/bpp/input/route", 1, true);
  const auto map_pub = node.advertise<autoware_map_msgs::LaneletMapBin>("/bpp_bvp_pipeline/bpp/input/vector_map", 1, true);
  const auto scenario_pub = node.advertise<autoware_internal_planning_msgs::Scenario>("/bpp_bvp_pipeline/bpp/input/scenario", 1, true);
  const auto accel_pub = node.advertise<geometry_msgs::AccelWithCovarianceStamped>("/bpp_bvp_pipeline/bpp/input/acceleration", 1, true);
  const auto mode_pub = node.advertise<autoware_adapi_v1_msgs::OperationModeState>("/bpp_bvp_pipeline/bpp/input/operation_mode_state", 1, true);
  const auto objects_pub = node.advertise<autoware_perception_msgs::PredictedObjects>("/bpp_bvp_pipeline/bpp/input/objects", 1, true);
  const auto odom_pub = node.advertise<nav_msgs::Odometry>("/bpp_bvp_pipeline/bpp/input/odometry", 1, true);
  const auto grid_pub = node.advertise<nav_msgs::OccupancyGrid>("/bpp_bvp_pipeline/bpp/input/occupancy_grid", 1, true);
  ros::AsyncSpinner spinner(3); spinner.start();
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map); ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin(); const auto centerline = lane.centerline(); ASSERT_GE(centerline.size(), 2U);
  auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  // overlap_map.osm MetaInfo, as emitted by the source Lanelet2 map loader.
  map_message.header.frame_id = "map";
  map_message.version_map_format = "1";
  map_message.version_map = "5";
  autoware_planning_msgs::LaneletRoute route; route.header.frame_id = "map";
  route.start_pose.position.x = centerline.front().x(); route.start_pose.position.y = centerline.front().y(); route.start_pose.orientation.w = 1.0;
  route.goal_pose.position.x = centerline.back().x(); route.goal_pose.position.y = centerline.back().y(); route.goal_pose.orientation.w = 1.0;
  autoware_planning_msgs::LaneletPrimitive primitive; primitive.id = lane.id();
  autoware_planning_msgs::LaneletSegment segment; segment.primitives.push_back(primitive); segment.preferred_primitive = primitive; route.segments.push_back(segment);
  autoware_internal_planning_msgs::Scenario scenario; scenario.current_scenario = autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  autoware_adapi_v1_msgs::OperationModeState mode; mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  nav_msgs::Odometry odometry; odometry.header.frame_id = "map"; odometry.pose.pose = route.start_pose; odometry.twist.twist.linear.x = 2.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware_perception_msgs::PredictedObjects objects;
  nav_msgs::OccupancyGrid grid; grid.header.frame_id = "map";
  const auto deadline = ros::WallTime::now() + ros::WallDuration(4.0);
  while (ros::ok() && ros::WallTime::now() < deadline && !receiver.message_) {
    map_pub.publish(map_message); route_pub.publish(route); scenario_pub.publish(scenario); accel_pub.publish(acceleration);
    mode_pub.publish(mode); objects_pub.publish(objects); odom_pub.publish(odometry); grid_pub.publish(grid);
    ros::WallDuration(0.05).sleep();
  }
  // ASSERT_* returns immediately on failure.  Stop callback threads first so
  // that a failed transport assertion cannot race BPP module destruction.
  spinner.stop();
  ASSERT_GT(bpp_points.load(), 2U);
  ASSERT_TRUE(receiver.message_);
  EXPECT_EQ(receiver.message_->header.frame_id, "map");
  EXPECT_EQ(receiver.message_->points.size(), bpp_points.load());
  (void)bridge; (void)output;
}
}
int main(int argc, char ** argv) { ros::init(argc, argv, "bpp_bvp_pipeline_test"); ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
