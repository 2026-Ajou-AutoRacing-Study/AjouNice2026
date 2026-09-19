#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <cmath>
#include <iostream>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: route_handler_runtime_oracle <map.osm>" << std::endl;
    return 64;
  }
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(argv[1]);
  if (!map || map->laneletLayer.empty()) {
    std::cerr << "map loader returned no lanelets" << std::endl;
    return 1;
  }
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  autoware::route_handler::RouteHandler handler(map_message);
  if (!handler.isMapMsgReady() || !handler.getRoutingGraphPtr()) {
    std::cerr << "RouteHandler did not create routing graph from LaneletMapBin" << std::endl;
    return 2;
  }
  lanelet::ConstLanelets path_lanelets;
  for (const auto & candidate : map->laneletLayer) {
    const auto & centerline = candidate.centerline();
    if (centerline.size() < 2) continue;
    const auto & start = centerline.front();
    const auto & end = centerline.back();
  geometry_msgs::Pose start_pose;
  start_pose.position.x = start.x();
  start_pose.position.y = start.y();
  start_pose.position.z = start.z();
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = (start.x() + end.x()) * 0.5;
  goal_pose.position.y = (start.y() + end.y()) * 0.5;
  goal_pose.position.z = (start.z() + end.z()) * 0.5;
  const auto yaw = std::atan2(end.y() - start.y(), end.x() - start.x());
  start_pose.orientation.z = std::sin(yaw * 0.5);
  start_pose.orientation.w = std::cos(yaw * 0.5);
  goal_pose.orientation.z = std::sin(yaw * 0.5);
  goal_pose.orientation.w = std::cos(yaw * 0.5);
    if (handler.planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets)) break;
  }
  if (path_lanelets.empty()) {
    std::cerr << "RouteHandler could not plan a path on the fixture lanelet" << std::endl;
    return 4;
  }
  handler.setRouteLanelets(path_lanelets);
  if (!handler.isHandlerReady()) {
    std::cerr << "RouteHandler did not become ready from planned lanelets" << std::endl;
    return 6;
  }
  const auto map_segments = handler.createMapSegments(path_lanelets);
  if (map_segments.empty() || map_segments.front().preferred_primitive.id != path_lanelets.front().id()) {
    std::cerr << "RouteHandler did not create a main-lane route segment after route setup" << std::endl;
    return 5;
  }
  autoware_planning_msgs::LaneletSegment segment;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = path_lanelets.front().id();
  segment.primitives = {primitive};
  if (autoware::route_handler::RouteHandler::isRouteLooped({segment, segment})) {
    std::cout << "route_handler_loop_detector=PASS" << std::endl;
  } else {
    std::cerr << "RouteHandler failed to detect duplicate lanelet route loop" << std::endl;
    return 7;
  }
  std::cout << "route_handler_runtime_oracle=PASS path_lanelets=" << path_lanelets.size()
            << " segments=" << map_segments.size() << std::endl;
  return 0;
}
