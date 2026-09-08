#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <gtest/gtest.h>

#include <cstdlib>

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
}  // namespace

TEST(BehaviorPathPlannerCore, BlocksMissingInputsAndPropagatesNoRequestOutput)
{
  ros::NodeHandle private_node("bpp_core_missing_inputs");
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
  ASSERT_TRUE(core.isInitialized());
  auto data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  // SideShiftModule::updateData reads the ego pose before it decides that no
  // lateral-offset request exists.  Keep the fixture within the real node
  // contract instead of asserting a readiness-only synthetic state.
  data->self_odometry = boost::make_shared<nav_msgs::Odometry>();
  data->dynamic_object = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  autoware::behavior_path_planner::BehaviorModuleOutput input;
  auto missing = readyState();
  missing.route_received = false;
  EXPECT_FALSE(core.run(data, input, missing, 10.0, 1.0, false));
  EXPECT_EQ(core.lastReadinessReport().status, autoware::behavior_path_planner::DataReadyStatus::NOT_RECEIVED);
  const auto output = core.run(data, input, readyState(), 10.0, 1.0, false);
  ASSERT_TRUE(output);
  EXPECT_TRUE(output->path.points.empty());
  EXPECT_EQ(core.activeModuleCount(), 0U);
}

TEST(BehaviorPathPlannerCore, RunsReadyFrozenRouteThroughStaticSideShiftSlot)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  lanelet::ConstLanelets route_lanes;
  geometry_msgs::Pose start;
  geometry_msgs::Pose goal;
  for (const auto & lane : map->laneletLayer) {
    const auto centerline = lane.centerline();
    if (centerline.size() < 2) continue;
    const auto & first = centerline.front();
    const auto & last = centerline.back();
    start.position.x = first.x(); start.position.y = first.y(); start.position.z = first.z();
    goal.position.x = (first.x() + last.x()) * 0.5;
    goal.position.y = (first.y() + last.y()) * 0.5;
    const auto yaw = std::atan2(last.y() - first.y(), last.x() - first.x());
    start.orientation.z = std::sin(yaw * 0.5); start.orientation.w = std::cos(yaw * 0.5);
    goal.orientation = start.orientation;
    if (route_handler->planPathLaneletsBetweenCheckpoints(start, goal, &route_lanes)) break;
  }
  ASSERT_FALSE(route_lanes.empty());
  route_handler->setRouteLanelets(route_lanes);
  const auto reference = route_handler->getCenterLinePath(route_lanes, 0.0, 20.0, true);
  ASSERT_GT(reference.points.size(), 2U);

  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->route_handler = route_handler;
  planner_data->parameters.backward_path_length = 5.0;
  planner_data->parameters.forward_path_length = 10.0;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = start;
  odometry->twist.twist.linear.x = 2.0;
  planner_data->self_odometry = odometry;
  planner_data->self_acceleration = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  planner_data->dynamic_object = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  ASSERT_TRUE(route_handler->isMapMsgReady());
  ASSERT_TRUE(route_handler->isHandlerReady());
  lanelet::ConstLanelet current_lane;
  ASSERT_TRUE(route_handler->getClosestLaneletWithinRoute(odometry->pose.pose, &current_lane));
  const auto current_lanes = route_handler->getLaneletSequence(
    current_lane, odometry->pose.pose, planner_data->parameters.backward_path_length,
    planner_data->parameters.forward_path_length);
  ASSERT_FALSE(current_lanes.empty());
  autoware::behavior_path_planner::BehaviorModuleOutput input;
  input.path = reference;
  input.reference_path = reference;
  constexpr const char * kNamespace = "/bpp_core_static_scene";
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  const std::string load =
    std::string("rosparam load ") + STATIC_OBSTACLE_AVOIDANCE_CONFIG + " " + kNamespace;
  ASSERT_EQ(std::system(load.c_str()), 0);
#endif
  ros::NodeHandle private_node(kNamespace);
  private_node.setParam("side_shift/min_distance_to_start_shifting", 1.0);
  private_node.setParam("side_shift/min_shifting_distance", 2.0);
  private_node.setParam("side_shift/min_shifting_speed", 1.0);
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  private_node.setParam("enable_static_obstacle_avoidance_module", true);
#endif
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
  ASSERT_TRUE(core.isInitialized());
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  ASSERT_TRUE(core.staticObstacleAvoidanceEnabled());
#endif
  auto lateral_offset = boost::make_shared<tier4_planning_msgs::LateralOffset>();
  lateral_offset->stamp.sec = 10;
  lateral_offset->stamp.nanosec = 0;
  lateral_offset->lateral_offset = 0.5F;
  ASSERT_TRUE(core.updateLateralOffset(planner_data, lateral_offset));
  const auto output = core.run(planner_data, input, readyState(), 10.0, 1.0, false);
  ASSERT_TRUE(output);
  ASSERT_FALSE(output->path.points.empty());
  EXPECT_TRUE(output->drivable_area_info.is_already_expanded);
  EXPECT_EQ(core.activeModuleCount(), 1U);
}

#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
TEST(BehaviorPathPlannerCore, CommittedSourceFixtureDoesNotInventAvoidanceRequest)
{
  // The Static package's committed route fixture names these lanelets.  Keep
  // its object, odometry, and map coordinate system intact for this oracle.
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_STATIC_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  const lanelet::Ids route_ids{3008384, 3008383, 3008382};
  autoware_planning_msgs::LaneletRoute route;
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
  route_handler->setRoute(route);
  const auto route_lanes = route_handler->getLaneletsFromIds(route_ids);
  ASSERT_EQ(route_lanes.size(), route_ids.size());
  auto reference = route_handler->getCenterLinePath(route_lanes, 0.0, 120.0, true);
  // The compact fixture stitches its centerline into two endpoints.  BPP's
  // ROS1 ingress materializes the same one-metre reference used by the live
  // node before a scene provider may construct its detection polygons.
  ASSERT_GE(reference.points.size(), 2U);
  reference = autoware::behavior_path_planner::utils::resamplePathWithSpline(reference, 1.0, true);
  ASSERT_GT(reference.points.size(), 2U);

  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->route_handler = route_handler;
  planner_data->parameters.backward_path_length = 5.0;
  planner_data->parameters.forward_path_length = 120.0;
  // Same non-zero vehicle geometry supplied to the BPP pipeline fixtures.
  // Static target filtering uses both the scalar compatibility fields and
  // VehicleInfo's footprint offsets.
  auto & parameters = planner_data->parameters;
  parameters.wheel_base = 2.7;
  parameters.front_overhang = 1.0;
  parameters.rear_overhang = 1.1;
  parameters.wheel_tread = 1.6;
  parameters.vehicle_width = 2.0;
  parameters.vehicle_length = 4.8;
  parameters.vehicle_info.wheel_base_m = 2.7;
  parameters.vehicle_info.wheel_tread_m = 1.6;
  parameters.vehicle_info.front_overhang_m = 1.0;
  parameters.vehicle_info.rear_overhang_m = 1.1;
  parameters.vehicle_info.left_overhang_m = 0.2;
  parameters.vehicle_info.right_overhang_m = 0.2;
  parameters.vehicle_info.vehicle_width_m = 2.0;
  parameters.vehicle_info.vehicle_length_m = 4.8;
  parameters.vehicle_info.min_longitudinal_offset_m = -1.1;
  parameters.vehicle_info.max_longitudinal_offset_m = 3.7;
  parameters.vehicle_info.min_lateral_offset_m = -1.0;
  parameters.vehicle_info.max_lateral_offset_m = 1.0;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.position.x = 1034.4942626953125;
  odometry->pose.pose.position.y = 1065.66455078125;
  odometry->pose.pose.position.z = 100.0;
  odometry->pose.pose.orientation.z = -0.9999274638548634;
  odometry->pose.pose.orientation.w = 0.012044377475855648;
  planner_data->self_odometry = odometry;
  planner_data->self_acceleration = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  auto objects = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  autoware_perception_msgs::PredictedObject object;
  object.object_id.uuid[0] = 53;
  object.existence_probability = 0.75F;
  object.classification.resize(1);
  object.classification.front().label = autoware_perception_msgs::ObjectClassification::CAR;
  object.classification.front().probability = 1.0F;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 989.1463723544637;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 1062.2266363165359;
  object.kinematics.initial_pose_with_covariance.pose.position.z = 101.46911081115611;
  object.kinematics.initial_pose_with_covariance.pose.orientation.z = -0.9999308945908969;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 0.011756106610975299;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.4;
  object.shape.dimensions.y = 1.9694254099834387;
  object.shape.dimensions.z = 2.0629175266660886;
  objects->objects.push_back(object);
  planner_data->dynamic_object = objects;

  constexpr const char * kNamespace = "/bpp_static_obstacle_source_fixture";
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
  autoware::behavior_path_planner::BehaviorModuleOutput input;
  input.path = reference;
  input.reference_path = reference;
  const auto output = core.run(planner_data, input, readyState(), 10.0, 1.0, false);
  ASSERT_TRUE(output);
  EXPECT_FALSE(output->path.points.empty());
  // This upstream fixture is used by planning-factor tests, not an avoidance
  // activation oracle.  Its presence must never cause the ROS1 slot to invent
  // a maneuver merely to satisfy a transport integration test.
  EXPECT_EQ(core.activeModuleCount(), 0U);
}
#endif

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "behavior_path_planner_core_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
