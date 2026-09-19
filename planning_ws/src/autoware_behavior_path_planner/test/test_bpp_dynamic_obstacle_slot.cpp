#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>
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

TEST(BehaviorPathPlannerCore, DynamicObstacleCreatesDrivableAreaObstacleAfterEntryDebounce)
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
  const auto reference = route_handler->getCenterLinePath(route_lanes, 0.0, 30.0, true);
  ASSERT_GT(reference.points.size(), 2U);

  auto data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  data->route_handler = route_handler;
  data->parameters.backward_path_length = 5.0;
  data->parameters.forward_path_length = 20.0;
  data->parameters.vehicle_width = 2.0;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = start;
  odometry->twist.twist.linear.x = 2.0;
  data->self_odometry = odometry;
  data->self_acceleration = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();

  auto objects = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  autoware_perception_msgs::PredictedObject object;
  object.object_id.uuid[0] = 1;
  object.existence_probability = 1.0F;
  object.classification.resize(1);
  object.classification.front().label = autoware_perception_msgs::ObjectClassification::CAR;
  object.classification.front().probability = 1.0F;
  const auto & target = reference.points.at(reference.points.size() / 2).point;
  object.kinematics.initial_pose_with_covariance.pose = target.pose;
  // A same-direction CAR on the ego path is deliberately classified as a
  // follow target by the source policy.  Use an aligned oncoming CAR instead:
  // it remains a regulated target but must reach the avoid-polygon branch.
  object.kinematics.initial_pose_with_covariance.pose.orientation.z =
    target.pose.orientation.w;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w =
    -target.pose.orientation.z;
  // Twist is expressed in the object's reversed body frame, hence a positive
  // body-x velocity projects to a negative tangent velocity on the ego path.
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 1.0;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  // Keep the obstacle on the ego corridor but below the source policy's
  // full-lane-cover exclusion threshold, so this exercises avoidance polygon
  // generation instead of the distinct blocking-object policy branch.
  object.shape.dimensions.y = 0.5;
  object.shape.dimensions.z = 1.5;
  autoware_perception_msgs::PredictedPath predicted_path;
  predicted_path.confidence = 1.0F;
  predicted_path.path.push_back(target.pose);
  object.kinematics.predicted_paths.push_back(predicted_path);
  objects->objects.push_back(object);
  data->dynamic_object = objects;

  constexpr const char * kNamespace = "/bpp_dynamic_obstacle_slot";
  const std::string load = std::string("rosparam load ") +
    DYNAMIC_OBSTACLE_AVOIDANCE_CONFIG + " " + kNamespace;
  ASSERT_EQ(std::system(load.c_str()), 0);
  ros::NodeHandle private_node(kNamespace);
  private_node.setParam("enable_dynamic_obstacle_avoidance_module", true);
  private_node.setParam("dynamic_avoidance/common/enable_debug_info", true);
  private_node.setParam("side_shift/min_distance_to_start_shifting", 1.0);
  private_node.setParam("side_shift/min_shifting_distance", 2.0);
  private_node.setParam("side_shift/min_shifting_speed", 1.0);
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
  ASSERT_TRUE(core.isInitialized());
  ASSERT_TRUE(core.dynamicObstacleAvoidanceEnabled());

  autoware::behavior_path_planner::BehaviorModuleOutput input;
  input.path = reference;
  input.reference_path = reference;
  std::optional<autoware::behavior_path_planner::BehaviorModuleOutput> output;
  for (int i = 0; i < 6; ++i) {
    output = core.run(data, input, readyState(), 10.0 + 0.1 * i, 1.0, false);
    ASSERT_TRUE(output);
  }
  ASSERT_EQ(core.activeModuleCount(), 1U);
  EXPECT_FALSE(output->drivable_area_info.obstacles.empty());
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bpp_dynamic_obstacle_slot_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
