#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>

#include <autoware/behavior_path_planner_common/planner_data.hpp>

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/constants.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

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
  const auto current_lane = autoware::experimental::lanelet2_utils::get_closest_lanelet(
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

autoware_internal_planning_msgs::PathWithLaneId resampleCenterLinePathWithSupport(
  const autoware_internal_planning_msgs::PathWithLaneId & source,
  const geometry_msgs::Pose & pose, const double backward_path_length,
  const double forward_path_length, const BehaviorPathPlannerParameters & parameters)
{
  if (source.points.size() < 2) return source;
  const double interval = parameters.input_path_interval;
  if (!std::isfinite(interval) || interval <= 0.0 ||
      !std::isfinite(backward_path_length) || backward_path_length < 0.0 ||
      !std::isfinite(forward_path_length) || forward_path_length < 0.0) {
    throw std::invalid_argument("Invalid centerline resampling interval or window");
  }
  const auto arc = autoware::motion_utils::calcSignedArcLengthPartialSum(
    source.points, 0, source.points.size());
  if (!std::all_of(arc.begin(), arc.end(), [](double s) { return std::isfinite(s); })) {
    throw std::invalid_argument("Non-finite centerline geometry");
  }
  const auto segment = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    source.points, pose, parameters.ego_nearest_dist_threshold,
    parameters.ego_nearest_yaw_threshold);
  const double ego_s = autoware::motion_utils::calcSignedArcLength(
    source.points, 0, pose.position, segment);
  const double start = std::clamp(ego_s - backward_path_length, 0.0, arc.back());
  const double end = std::clamp(ego_s + forward_path_length, 0.0, arc.back());
  if (end - start < autoware::motion_utils::overlap_threshold) return {};

  // Keep ALL native knots when computing Akima's tangents (interval [i,i+1]
  // needs points i-2..i+3). A fixed distance margin cannot supply these on a
  // sparse map. Crop only the queries, never the interpolation input.
  // Anchor the sampling lattice to the source, not the moving ego window.
  std::vector<double> queries{start};
  for (double s = (std::floor(start / interval) + 1.0) * interval; s < end; s += interval) {
    if (s - queries.back() >= autoware::motion_utils::overlap_threshold) queries.push_back(s);
  }
  if (end - queries.back() < autoware::motion_utils::overlap_threshold) queries.back() = end;
  else queries.push_back(end);
  // The explicit-query resampler preserves lane-ID seams, but its interval
  // overload normally inserts the exact first stop. Preserve that contract too.
  std::vector<autoware_planning_msgs::PathPoint> points;
  points.reserve(source.points.size());
  for (const auto & p : source.points) points.push_back(p.point);
  const auto stop = autoware::motion_utils::calcDistanceToForwardStopPoint(points, 0);
  if (stop && *stop >= start && *stop <= end) {
    const auto nearby = std::find_if(queries.begin(), queries.end(), [&](double s) {
      return std::abs(s - *stop) < autoware::motion_utils::overlap_threshold;
    });
    if (nearby != queries.end()) *nearby = *stop;
    else queries.push_back(*stop);
    std::sort(queries.begin(), queries.end());
    queries.erase(std::unique(queries.begin(), queries.end()), queries.end());
  }
  return autoware::motion_utils::resamplePath(
    source, queries, parameters.enable_akima_spline_first);
}

autoware_internal_planning_msgs::PathWithLaneId getCenterLinePath(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::Pose & pose,
  const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameters)
{
  if (lanelet_sequence.empty()) return {};
  const bool limit_at_goal = route_handler.isInGoalRouteSection(lanelet_sequence.back());
  double forward_limit = forward_path_length;
  const auto forward_window = [&](const auto & path, double ego_s) {
    if (!limit_at_goal || path.points.size() < 2) return forward_limit;
    const auto goal = route_handler.getGoalPose();
    const auto segment = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, goal, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
    const double goal_s = autoware::motion_utils::calcSignedArcLength(
      path.points, 0, goal.position, segment);
    // Use the same native-waypoint arc as the queries, including custom detours.
    return std::min(forward_limit, std::max(0.0, goal_s - ego_s));
  };
  // Extract native waypoint support across the selected lanelet sequence.
  // Exact crop endpoints used to replace native knots BEFORE resampling; when
  // a knot left the window, a long straight could suddenly bow across lanes.
  auto support_lanes = lanelet_sequence;
  std::set<lanelet::Id> visited;
  for (const auto & lane : support_lanes) visited.insert(lane.id());
  auto source = route_handler.getCenterLinePath(
    support_lanes, 0.0, std::numeric_limits<double>::max(), true);
  if (source.points.size() < 2) {
    source.header = route_handler.getRouteHeader();
    return source;
  }
  const auto initial_segment = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    source.points, pose, parameters.ego_nearest_dist_threshold,
    parameters.ego_nearest_yaw_threshold);
  const double initial_ego_s = autoware::motion_utils::calcSignedArcLength(
    source.points, 0, pose.position, initial_segment);
  const double backward_limit = std::min(backward_path_length, std::max(0.0, initial_ego_s));
  forward_limit = std::min(forward_limit, std::max(
    0.0, autoware::motion_utils::calcArcLength(source.points) - initial_ego_s));
  // Added lanelets supply tangents only; they must not enlarge the caller's
  // original output range or extend its path beyond a goal/dead end.
  // A lanelet-sequence boundary also moves when the current lane changes.
  // Extend along route topology until the query intervals have their native
  // two-point support on both sides, or an actual route endpoint is reached.
  while (source.points.size() >= 2) {
    const auto native_arc = autoware::motion_utils::calcSignedArcLengthPartialSum(
      source.points, 0, source.points.size());
    const auto segment = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      source.points, pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
    const double ego_s = autoware::motion_utils::calcSignedArcLength(
      source.points, 0, pose.position, segment);
    const double start = std::max(0.0, ego_s - backward_limit);
    const double end = std::min(native_arc.back(), ego_s + forward_window(source, ego_s));
    const auto begin_interval = static_cast<size_t>(std::max<std::ptrdiff_t>(
      0, std::upper_bound(native_arc.begin(), native_arc.end(), start) - native_arc.begin() - 1));
    const auto end_interval = static_cast<size_t>(std::max<std::ptrdiff_t>(
      0, std::lower_bound(native_arc.begin(), native_arc.end(), end) - native_arc.begin() - 1));
    bool extended = false;
    if (begin_interval < 2) {
      lanelet::ConstLanelets predecessors;
      if (route_handler.getPreviousLaneletsWithinRoute(support_lanes.front(), &predecessors)) {
        // Match RouteHandler's backward-sequence ordering; do not choose a
        // nearby parallel lane or create a new lateral route connection.
        for (const auto & lane : predecessors) {
          if (visited.insert(lane.id()).second) {
            support_lanes.insert(support_lanes.begin(), lane);
            extended = true;
            break;
          }
        }
      }
    }
    if (end_interval + 3 >= source.points.size()) {
      lanelet::ConstLanelet successor;
      if (route_handler.getNextLaneletWithinRoute(support_lanes.back(), &successor) &&
          visited.insert(successor.id()).second) {
        support_lanes.push_back(successor);
        extended = true;
      }
    }
    if (!extended) break;
    source = route_handler.getCenterLinePath(
      support_lanes, 0.0, std::numeric_limits<double>::max(), true);
  }
  if (source.points.size() < 2) return source;
  const auto segment = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    source.points, pose, parameters.ego_nearest_dist_threshold,
    parameters.ego_nearest_yaw_threshold);
  const double ego_s = autoware::motion_utils::calcSignedArcLength(
    source.points, 0, pose.position, segment);
  auto path = resampleCenterLinePathWithSupport(
    source, pose, backward_limit, forward_window(source, ego_s), parameters);
  // RouteHandler's geometric centerline has no message header. Set it before
  // this path is copied into module candidates and checked against trajectories.
  path.header = route_handler.getRouteHeader();
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
