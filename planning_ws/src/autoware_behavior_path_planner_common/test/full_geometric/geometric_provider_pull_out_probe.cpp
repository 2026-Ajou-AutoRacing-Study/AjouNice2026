#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <iostream>
#include <memory>

namespace
{
lanelet::Lanelet make_lane(
  lanelet::Id id, lanelet::Id left_id, lanelet::Id right_id, double left_y, double right_y)
{
  const lanelet::LineString3d left(
    left_id, {lanelet::Point3d(left_id + 1, -50.0, left_y, 0.0),
              lanelet::Point3d(left_id + 2, 100.0, left_y, 0.0)});
  const lanelet::LineString3d right(
    right_id, {lanelet::Point3d(right_id + 1, -50.0, right_y, 0.0),
               lanelet::Point3d(right_id + 2, 100.0, right_y, 0.0)});
  return lanelet::Lanelet(id, left, right);
}

geometry_msgs::Pose pose(double x, double y)
{
  geometry_msgs::Pose value;
  value.position.x = x;
  value.position.y = y;
  value.orientation.w = 1.0;
  return value;
}
}  // namespace

int main()
{
  const auto road = make_lane(101, 1001, 1004, 2.0, -2.0);
  const auto shoulder = make_lane(102, 1011, 1014, 6.0, 2.0);
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road);
  map->add(shoulder);

  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  planner_data->parameters.wheel_base = 2.74;
  planner_data->parameters.wheel_tread = 1.63;
  planner_data->parameters.left_over_hang = 0.10;
  planner_data->parameters.front_overhang = 1.0;
  planner_data->parameters.base_link2front = 3.74;
  planner_data->parameters.base_link2rear = 1.0;
  planner_data->parameters.vehicle_width = 1.8;
  planner_data->parameters.forward_path_length = 25.0;

  autoware::behavior_path_planner::ParallelParkingParameters parking;
  parking.center_line_path_interval = 0.5;
  parking.pull_out_velocity = 1.0;
  parking.pull_out_lane_departure_margin = 0.0;
  parking.pull_out_arc_path_interval = 0.25;

  autoware::behavior_path_planner::GeometricParallelParking provider;
  provider.setParameters(parking);
  provider.setPlannerData(planner_data);
  provider.setTurningRadius(planner_data->parameters, 0.45);

  const lanelet::ConstLanelets road_lanes{road};
  const lanelet::ConstLanelets pull_over_lanes{shoulder};
  const bool planned = provider.planPullOut(
    pose(0.0, 4.0), pose(30.0, 0.0), road_lanes, pull_over_lanes, true, nullptr);
  if (!planned || provider.getPaths().size() < 2U || provider.getFullPath().points.empty()) {
    std::cerr << "AW149_START_PLANNER_GEOMETRIC_PULL_OUT_PROBE_FAIL planned=" << planned
              << " paths=" << provider.getPaths().size()
              << " points=" << provider.getFullPath().points.size() << std::endl;
    return 1;
  }
  std::cout << "AW149_START_PLANNER_GEOMETRIC_PULL_OUT_PROBE_PASS paths="
            << provider.getPaths().size() << " points=" << provider.getFullPath().points.size()
            << std::endl;
  return 0;
}
