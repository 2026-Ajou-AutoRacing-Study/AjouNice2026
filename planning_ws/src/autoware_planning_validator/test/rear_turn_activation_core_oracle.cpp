#include "autoware/planning_validator/rear_turn_activation_core.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <angles/angles.h>
#include <gtest/gtest.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

TEST(RearTurnActivationCore, FailsClosedWithoutNodeOwnedRouteHandler)
{
  autoware_planning_msgs::Trajectory trajectory;
  for (double x : {0.0, 10.0}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    trajectory.points.push_back(point);
  }
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  autoware::planning_validator::RearMotionEnvelope envelope;
  const lanelet::ConstLanelets no_lanes;
  const auto result = autoware::planning_validator::evaluate_rear_turn_activation(
    {no_lanes, trajectory, odometry, vehicle, envelope, nullptr, false}, {});
  EXPECT_EQ(result.behavior, autoware::planning_validator::RearBehavior::NONE);
}

TEST(RearTurnActivationCore, ActivatesLeftTurnWithLoadedRouteHandlerGraph)
{
  // Match the established ROS1 route-handler fixture path: the fixture stores
  // local coordinates, while its latitude/longitude fields are only used to
  // establish the MGRS grid.
  lanelet::ErrorMessages errors;
  lanelet::projection::MGRSProjector projector;
  auto raw_map = lanelet::load(AW149_TURN_FIXTURE_PATH, projector, &errors);
  ASSERT_TRUE(errors.empty());
  ASSERT_TRUE(raw_map);
  const auto map = lanelet::LaneletMapPtr{raw_map.release()};
  for (auto point : map->pointLayer) {
    if (point.hasAttribute("local_x")) {
      point.x() = point.attribute("local_x").asDouble().value();
    }
    if (point.hasAttribute("local_y")) {
      point.y() = point.attribute("local_y").asDouble().value();
    }
  }
  lanelet::utils::overwriteLaneletsCenterline(map, 5.0, false);
  ASSERT_TRUE(map);
  autoware::route_handler::RouteHandler route_handler(map);
  ASSERT_TRUE(route_handler.getRoutingGraphPtr());
  lanelet::ConstLanelet preceding_lane;
  lanelet::ConstLanelet source_lane;
  for (const auto & lane : map->laneletLayer) {
    if (lane.centerline().size() < 2U) continue;
    for (const auto & previous : route_handler.getRoutingGraphPtr()->previous(lane)) {
      if (previous.centerline().size() >= 2U) {
        preceding_lane = previous;
        source_lane = lane;
        break;
      }
    }
    if (source_lane.id() != lanelet::InvalId) break;
  }
  ASSERT_NE(preceding_lane.id(), lanelet::InvalId);
  ASSERT_NE(source_lane.id(), lanelet::InvalId);
  // The original OSM fixture declares left turn_direction relations, but this
  // ROS1 Lanelet2 parser does not retain that relation attribute. Keep the
  // parsed geometry and RouteHandler graph real, while supplying the frozen
  // source metadata through the same lanelet input contract the core receives
  // from the node's route lane sequence.
  auto tagged_turn = map->laneletLayer.get(source_lane.id());
  tagged_turn.attributes()["turn_direction"] = "left";
  const lanelet::ConstLanelet left_turn = tagged_turn;
  ASSERT_EQ(std::string(left_turn.attributeOr("turn_direction", "")), "left");

  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & point : preceding_lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = point.x();
    trajectory_point.pose.position.y = point.y();
    trajectory_point.pose.position.z = point.z();
    trajectory_point.pose.orientation.w = 1.0;
    trajectory_point.longitudinal_velocity_mps = 1.0F;
    trajectory.points.push_back(trajectory_point);
  }
  for (const auto & point : left_turn.centerline()) {
    autoware_planning_msgs::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = point.x();
    trajectory_point.pose.position.y = point.y();
    trajectory_point.pose.position.z = point.z();
    trajectory_point.pose.orientation.w = 1.0;
    trajectory_point.longitudinal_velocity_mps = 1.0F;
    trajectory.points.push_back(trajectory_point);
  }
  nav_msgs::Odometry odometry;
  odometry.pose.pose = trajectory.points.front().pose;
  const auto & last = left_turn.centerline().back();
  const auto & before_last = left_turn.centerline()[left_turn.centerline().size() - 2U];
  const auto lane_yaw = std::atan2(last.y() - before_last.y(), last.x() - before_last.x());
  const auto ego_yaw = lane_yaw - 1.5707963267948966;
  odometry.pose.pose.orientation.z = std::sin(ego_yaw * 0.5);
  odometry.pose.pose.orientation.w = std::cos(ego_yaw * 0.5);
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  EXPECT_LT(
    angles::shortest_angular_distance(lane_yaw, tf2::getYaw(odometry.pose.pose.orientation)), -0.78);
  autoware::planning_validator::RearMotionEnvelope envelope;
  envelope.lookahead_distance = 1.0e6;
  envelope.predicted_stop_distance = 0.0;
  autoware::planning_validator::RearTurnActivationConfig config;
  config.check_on_unstoppable = true;
  EXPECT_FALSE(autoware::motion_utils::calcDistanceToForwardStopPoint(
    trajectory.points, odometry.pose.pose).has_value());
  const auto result = autoware::planning_validator::evaluate_rear_turn_activation(
    {{preceding_lane, left_turn}, trajectory, odometry, vehicle, envelope, &route_handler, false}, config);
  EXPECT_EQ(result.behavior, autoware::planning_validator::RearBehavior::TURN_LEFT);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
