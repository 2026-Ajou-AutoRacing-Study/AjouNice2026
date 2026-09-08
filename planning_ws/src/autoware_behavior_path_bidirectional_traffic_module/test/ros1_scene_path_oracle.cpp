#include <autoware/behavior_path_bidirectional_traffic_module/scene.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
std::shared_ptr<autoware::route_handler::RouteHandler> loadRouteHandler()
{
  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({35.228457, 138.803539}));
  auto loaded = lanelet::load(AW149_BIDIRECTIONAL_TEST_MAP, projector, &errors);
  if (!loaded) return {};
  std::shared_ptr<lanelet::LaneletMap> mutable_map(std::move(loaded));
  lanelet::LaneletMapConstPtr map = mutable_map;
  return std::make_shared<autoware::route_handler::RouteHandler>(map);
}
}  // namespace

TEST(Aw149BidirectionalScene, KeepLeftChangesBidirectionalPath)
{
  ros::Time::init();
  auto route_handler = loadRouteHandler();
  ASSERT_TRUE(route_handler);
  const auto lanes = autoware::behavior_path_planner::ConnectedBidirectionalLanelets::
    search_bidirectional_lanes_on_map(
      *route_handler->getLaneletMapPtr(),
      [&route_handler](const lanelet::ConstLanelet & lane) { return route_handler->getNextLanelets(lane); },
      [&route_handler](const lanelet::ConstLanelet & lane) { return route_handler->getPreviousLanelets(lane); });
  ASSERT_EQ(lanes.size(), 8U);
  const auto lane = lanes.front();
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = lane->get_center_line().compute(0.0);
  odometry->twist.twist.linear.x = 5.0;
  planner_data->self_odometry = odometry;
  planner_data->route_handler = route_handler;
  planner_data->dynamic_object = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  planner_data->parameters.base_link2front = 3.8;
  planner_data->parameters.base_link2rear = 1.1;
  planner_data->parameters.vehicle_width = 1.9;

  autoware::behavior_path_planner::BehaviorModuleOutput input;
  for (double s : {0.0, 5.0, 10.0, 15.0, 20.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose = lane->get_center_line().compute(s);
    point.point.longitudinal_velocity_mps = 5.0F;
    point.lane_ids.push_back(lane->get_lanelets().front().id());
    input.path.points.push_back(point);
    input.reference_path.points.push_back(point);
  }

  auto parameters = std::make_shared<autoware::behavior_path_planner::BidirectionalTrafficModuleParameters>();
  parameters->keep_left_distance_from_center_line = 0.5;
  autoware::behavior_path_planner::SceneModuleInterface::RTCInterfaceMap rtc_interfaces;
  autoware::behavior_path_planner::SceneModuleInterface::MarkerInterfaceMap marker_interfaces;
  autoware::behavior_path_planner::BidirectionalTrafficModule module(
    "bidirectional_traffic", parameters, rtc_interfaces, marker_interfaces, {});
  module.setData(planner_data);
  module.setPreviousModuleOutput(input);
  const auto output = module.run();
  ASSERT_EQ(output.path.points.size(), input.path.points.size());
  bool changed = false;
  for (size_t i = 0; i < output.path.points.size(); ++i) {
    const auto & before = input.path.points[i].point.pose.position;
    const auto & after = output.path.points[i].point.pose.position;
    changed = changed || std::hypot(before.x - after.x, before.y - after.y) > 0.05;
  }
  EXPECT_TRUE(changed);
}
