#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>

#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <autoware/behavior_path_planner_common/utils/path_lanelet_owner.hpp>

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>

#include <algorithm>
#include <set>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

namespace autoware::behavior_path_planner::utils
{

double getSignedDistance(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
  const lanelet::ConstLanelets & lanelets)
{
  const auto current = autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelets, current_pose);
  const auto goal = autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelets, goal_pose);
  return goal.length - current.length;
}

double getDistanceToEndOfLane(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) return 0.0;
  const auto arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelets, current_pose);
  return lanelet::geometry::length3d(lanelet::LaneletSequence(lanelets)) - arc.length;
}

lanelet::ConstLanelets getCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_path_length,
  const double forward_path_length)
{
  if (!planner_data || !planner_data->route_handler || !planner_data->self_odometry ||
      !planner_data->route_handler->isMapMsgReady() ||
      !planner_data->route_handler->isHandlerReady()) {
    return {};
  }
  const auto current_pose = planner_data->self_odometry->pose.pose;
  lanelet::ConstLanelet current_lane;
  if (!planner_data->route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    return {};
  }
  return planner_data->route_handler->getLaneletSequence(
    current_lane, current_pose, backward_path_length, forward_path_length);
}

lanelet::ConstLanelets getCurrentLanes(const std::shared_ptr<const PlannerData> & planner_data)
{
  if (!planner_data) return {};
  return getCurrentLanes(
    planner_data, planner_data->parameters.backward_path_length,
    planner_data->parameters.forward_path_length);
}

namespace
{
lanelet::ConstLanelets extendPreviousRouteLane(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanes)
{
  if (lanes.empty()) return lanes;
  const auto previous_lanes = route_handler->getPreviousLanelets(lanes.front());
  auto extended = lanes;
  for (const auto & previous : previous_lanes) {
    if (previous.id() == lanes.back().id()) continue;
    if (route_handler->isRouteLanelet(previous)) {
      extended.insert(extended.begin(), previous);
      break;
    }
  }
  return extended;
}
}  // namespace

lanelet::ConstLanelets getCurrentLanesFromPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  if (path.points.empty() || !planner_data || !planner_data->route_handler ||
      !planner_data->self_odometry || !planner_data->route_handler->isMapMsgReady() ||
      !planner_data->route_handler->isHandlerReady()) {
    return {};
  }
  std::set<lanelet::Id> lane_ids;
  for (const auto & point : path.points) lane_ids.insert(point.lane_ids.begin(), point.lane_ids.end());
  lanelet::ConstLanelets reference_lanes;
  for (const auto id : lane_ids) reference_lanes.push_back(planner_data->route_handler->getLaneletsFromId(id));
  const auto current_lane = getPathLaneletOwner(
    reference_lanes, planner_data->self_odometry->pose.pose);
  if (!current_lane) return {};
  auto lanes = planner_data->route_handler->getLaneletSequence(
    *current_lane, planner_data->self_odometry->pose.pose,
    planner_data->parameters.backward_path_length, planner_data->parameters.forward_path_length);
  const auto & front_ids = path.points.front().lane_ids;
  const auto contains_front = [&]() {
    return std::any_of(lanes.begin(), lanes.end(), [&](const auto & lane) {
      return std::any_of(front_ids.begin(), front_ids.end(), [&](const auto id) { return id == lane.id(); });
    });
  };
  while (!contains_front()) {
    const auto extended = extendPreviousRouteLane(planner_data->route_handler, lanes);
    if (extended.size() == lanes.size()) break;
    lanes = extended;
  }
  return lanes;
}

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & pose, const double forward_length, const double backward_length,
  const double distance_threshold, const double yaw_threshold)
{
  if (!route_handler || !route_handler->isMapMsgReady() || !route_handler->isHandlerReady()) return {};
  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(
        pose, &current_lane, distance_threshold, yaw_threshold)) {
    return {};
  }
  return route_handler->getLaneletSequence(current_lane, pose, backward_length, forward_length);
}

autoware_internal_planning_msgs::PathWithLaneId getCenterLinePath(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::Pose & pose,
  const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameters)
{
  if (lanelet_sequence.empty()) return {};
  const auto arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelet_sequence, pose);
  const auto backward = std::max(0.0, arc.length - backward_path_length);
  auto forward = arc.length + forward_path_length;
  if (route_handler.isDeadEndLanelet(lanelet_sequence.back())) {
    forward = std::clamp(forward, 0.0, lanelet::geometry::length2d(lanelet::LaneletSequence(lanelet_sequence)));
  }
  if (route_handler.isInGoalRouteSection(lanelet_sequence.back())) {
    const auto goal_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
      lanelet_sequence, route_handler.getGoalPose());
    forward = std::clamp(forward, 0.0, goal_arc.length);
  }
  auto path = autoware::motion_utils::resamplePath(
    route_handler.getCenterLinePath(lanelet_sequence, backward, forward, true),
    parameters.input_path_interval, parameters.enable_akima_spline_first);
  if (parameters.enable_cog_on_centerline) {
    return autoware::motion_utils::convertToRearWheelCenter(
      path, parameters.vehicle_length * 0.5 - parameters.rear_overhang);
  }
  return path;
}

autoware_internal_planning_msgs::PathWithLaneId getCenterLinePathFromLanelet(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data)
{
  if (!planner_data || !planner_data->route_handler || !planner_data->self_odometry) return {};
  const auto & pose = planner_data->self_odometry->pose.pose;
  const auto & parameters = planner_data->parameters;
  const auto lanes = planner_data->route_handler->getLaneletSequence(
    lanelet, pose, parameters.backward_path_length, parameters.forward_path_length);
  return getCenterLinePath(
    *planner_data->route_handler, lanes, pose, parameters.backward_path_length,
    parameters.forward_path_length, parameters);
}

bool checkPathRelativeAngle(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const double angle_threshold)
{
  if (path.points.size() < 3) return true;
  for (size_t i = 0; i + 2 < path.points.size(); ++i) {
    const auto & p1 = path.points[i].point.pose.position;
    const auto & p2 = path.points[i + 1].point.pose.position;
    const auto & p3 = path.points[i + 2].point.pose.position;
    if (path.points[i].point.longitudinal_velocity_mps < 0.0F ||
        path.points[i + 1].point.longitudinal_velocity_mps < 0.0F ||
        path.points[i + 2].point.longitudinal_velocity_mps < 0.0F) continue;
    const auto theta = std::atan2(p2.y - p1.y, p2.x - p1.x);
    const auto relative = std::atan2(
      -(p3.x - p1.x) * std::sin(theta) + (p3.y - p1.y) * std::cos(theta),
      (p3.x - p1.x) * std::cos(theta) + (p3.y - p1.y) * std::sin(theta));
    if (std::abs(relative) > angle_threshold) return false;
  }
  return true;
}

namespace
{
lanelet::ConstLanelets extend_next_lane(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanes)
{
  if (!route_handler || lanes.empty()) return lanes;
  const auto next_lanes = route_handler->getNextLanelets(lanes.back());
  auto extended = lanes;
  for (const auto & next : next_lanes) {
    if (next.id() == lanes.front().id()) continue;
    if (route_handler->isRouteLanelet(next)) {
      extended.push_back(next);
      break;
    }
  }
  return extended;
}

lanelet::ConstLanelets extend_previous_lane(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanes)
{
  if (!route_handler || lanes.empty()) return lanes;
  const auto previous_lanes = route_handler->getPreviousLanelets(lanes.front());
  auto extended = lanes;
  for (const auto & previous : previous_lanes) {
    if (previous.id() == lanes.back().id()) continue;
    if (route_handler->isRouteLanelet(previous)) {
      extended.insert(extended.begin(), previous);
      break;
    }
  }
  return extended;
}
}  // namespace

lanelet::ConstLanelets extendLanes(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanes)
{
  return extend_previous_lane(route_handler, extend_next_lane(route_handler, lanes));
}

lanelet::ConstLanelets transformToLanelets(const DrivableLanes & drivable_lanes)
{
  lanelet::ConstLanelets lanes;
  const auto append_unique = [&lanes](const lanelet::ConstLanelet & lane) {
    if (std::none_of(lanes.begin(), lanes.end(), [&lane](const auto & existing) {
          return existing.id() == lane.id();
        })) {
      lanes.push_back(lane);
    }
  };
  append_unique(drivable_lanes.right_lane);
  append_unique(drivable_lanes.left_lane);
  for (const auto & lane : drivable_lanes.middle_lanes) append_unique(lane);
  return lanes;
}

lanelet::ConstLanelets transformToLanelets(const std::vector<DrivableLanes> & drivable_lanes)
{
  lanelet::ConstLanelets lanes;
  for (const auto & drivable_lane : drivable_lanes) {
    const auto flattened = transformToLanelets(drivable_lane);
    lanes.insert(lanes.end(), flattened.begin(), flattened.end());
  }
  return lanes;
}

}  // namespace autoware::behavior_path_planner::utils
