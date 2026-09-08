#ifndef AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_UTIL_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_UTIL_HPP_

#include <geometry_msgs/Pose.h>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <algorithm>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <cstdint>
#include <optional>
#include <set>
#include <unordered_map>
#include <optional>
#include <vector>

namespace autoware::route_handler { class RouteHandler; }

namespace autoware::behavior_velocity_planner
{
// Original common detection/partition contracts, without ROS2 dependencies.
struct DetectionRange
{
  bool use_right = true;
  bool use_left = true;
  double interval{0.0};
  double min_longitudinal_distance{0.0};
  double max_longitudinal_distance{0.0};
  double max_lateral_distance{0.0};
  double wheel_tread{0.0};
  double right_overhang{0.0};
  double left_overhang{0.0};
};
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;
using Polygons2d = std::vector<autoware_utils_geometry::Polygon2d>;
}

namespace autoware::behavior_velocity_planner::planning_utils
{
bool createDetectionAreaPolygons(
  Polygons2d & polygons, const autoware_internal_planning_msgs::PathWithLaneId & path,
  const geometry_msgs::Pose & target_pose, size_t target_seg_idx,
  const DetectionRange & range, double obstacle_velocity, double min_velocity = 1.0);
inline void extractClosePartition(
  const geometry_msgs::Point position, const BasicPolygons2d & all_partitions,
  BasicPolygons2d & close_partition, const double distance_thresh = 30.0)
{
  close_partition.clear();
  for (const auto & polygon : all_partitions) {
    if (boost::geometry::distance(
          autoware_utils_geometry::Point2d(position.x, position.y), polygon) < distance_thresh) {
      close_partition.emplace_back(polygon);
    }
  }
}
inline void getAllPartitionLanelets(const lanelet::LaneletMapConstPtr & map, BasicPolygons2d & polygons)
{
  const lanelet::ConstLineStrings3d partitions = lanelet::utils::query::getAllPartitions(map);
  for (const auto & partition : partitions) {
    lanelet::BasicLineString2d line;
    for (const auto & point : partition) line.emplace_back(lanelet::BasicPoint2d{point.x(), point.y()});
    boost::geometry::correct(line);
    polygons.emplace_back(line);
  }
}
// Original utilization/util.cpp:857-868. Restore points and both bounds;
// the source deliberately leaves the message header default-constructed.
inline autoware_internal_planning_msgs::PathWithLaneId fromTrajectory(
  const autoware::experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::PathPointWithLaneId> & path,
  const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound)
{
  autoware_internal_planning_msgs::PathWithLaneId path_msg;
  path_msg.points = path.restore();
  path_msg.left_bound = left_bound;
  path_msg.right_bound = right_bound;
  return path_msg;
}
inline void setVelocityFromIndex(const size_t begin_idx, const double vel, autoware_internal_planning_msgs::PathWithLaneId * input)
{
  for (size_t i = begin_idx; i < input->points.size(); ++i) {
    input->points.at(i).point.longitudinal_velocity_mps =
      std::min(static_cast<float>(vel), input->points.at(i).point.longitudinal_velocity_mps);
  }
}

// Original util.cpp insertion and downstream minimum-velocity contract.
inline void insertVelocity(
  autoware_internal_planning_msgs::PathWithLaneId & path,
  const autoware_internal_planning_msgs::PathPointWithLaneId & path_point,
  const double v, size_t & insert_index, const double min_distance = 0.001)
{
  bool already_has_path_point = false;
  int min_idx = std::max(0, static_cast<int>(insert_index - 1));
  int max_idx = std::min(static_cast<int>(insert_index + 1), static_cast<int>(path.points.size() - 1));
  for (int i = min_idx; i <= max_idx; i++) {
    if (autoware_utils_geometry::calc_distance2d(path.points.at(static_cast<size_t>(i)), path_point) < min_distance) {
      path.points.at(i).point.longitudinal_velocity_mps = static_cast<float>(v);
      already_has_path_point = true;
      insert_index = static_cast<size_t>(i);
      break;
    }
  }
  if (!already_has_path_point) {
    path.points.insert(path.points.begin() + static_cast<std::ptrdiff_t>(insert_index), path_point);
  }
  setVelocityFromIndex(insert_index, v, &path);
}

autoware_utils_geometry::Polygon2d generatePathPolygon(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  size_t start_idx, size_t end_idx, double width);
bool isOverLine(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const geometry_msgs::Pose & self_pose, const geometry_msgs::Pose & line_pose,
  double offset = 0.0);

inline std::set<lanelet::Id> getAssociativeIntersectionLanelets(
  const lanelet::ConstLanelet & lane, const lanelet::LaneletMapPtr lanelet_map,
  const lanelet::routing::RoutingGraphPtr routing_graph)
{
  const std::string turn_direction = lane.attributeOr("turn_direction", "else");
  if (turn_direction == "else") {
    return {};
  }

  const auto parents = routing_graph->previous(lane);
  std::set<lanelet::Id> parent_neighbors;
  for (const auto & parent : parents) {
    const auto neighbors = routing_graph->besides(parent);
    for (const auto & neighbor : neighbors) parent_neighbors.insert(neighbor.id());
  }
  std::set<lanelet::Id> associative_intersection_lanelets;
  associative_intersection_lanelets.insert(lane.id());
  for (const auto & parent_neighbor_id : parent_neighbors) {
    const auto parent_neighbor = lanelet_map->laneletLayer.get(parent_neighbor_id);
    const auto followings = routing_graph->following(parent_neighbor);
    for (const auto & following : followings) {
      if (following.attributeOr("turn_direction", "else") == turn_direction) {
        associative_intersection_lanelets.insert(following.id());
      }
    }
  }
  return associative_intersection_lanelets;
}

// Original utilization/util.cpp:822-831, preserving sorted input-id order.
inline lanelet::ConstLanelets getConstLaneletsFromIds(
  const lanelet::LaneletMapConstPtr & map, const std::set<lanelet::Id> & ids)
{
  lanelet::ConstLanelets ret{};
  for (const auto & id : ids) {
    const auto ll = map->laneletLayer.get(id);
    ret.push_back(ll);
  }
  return ret;
}

inline void toTrajectory(
  const autoware_internal_planning_msgs::PathWithLaneId & path_msg,
  autoware::experimental::trajectory::Trajectory<autoware_internal_planning_msgs::PathPointWithLaneId> &
    path)
{
  const auto path_opt = autoware::experimental::trajectory::pretty_build(path_msg.points);
  if (!path_opt) {
    std::cerr << "[behavior_velocity](toTrajectory) Failed to build trajectory" << std::endl;
    return;
  }
  path = *path_opt;
}

inline int64_t bitShift(const int64_t original_id)
{
  return original_id << (sizeof(int32_t) * 8 / 2);
}

bool isAheadOf(const geometry_msgs::Pose & target, const geometry_msgs::Pose & origin);
double calcJudgeLineDistWithAccLimit(double velocity, double max_stop_acceleration, double delay_response_time);
double calcJudgeLineDistWithJerkLimit(double velocity, double acceleration, double max_stop_acceleration, double max_stop_jerk, double delay_response_time);
double calcDecelerationVelocityFromDistanceToTarget(double max_slowdown_jerk, double max_slowdown_accel, double current_accel, double current_velocity, double distance_to_target);
std::optional<geometry_msgs::Pose> insertDecelPoint(
  const geometry_msgs::Point & stop_point, autoware_internal_planning_msgs::PathWithLaneId & output,
  float target_velocity);
std::optional<geometry_msgs::Pose> insertStopPoint(
  const geometry_msgs::Point & stop_point, autoware_internal_planning_msgs::PathWithLaneId & output);
std::optional<geometry_msgs::Pose> insertStopPoint(
  const geometry_msgs::Point & stop_point, size_t stop_seg_idx,
  autoware_internal_planning_msgs::PathWithLaneId & output);
geometry_msgs::Pose getAheadPose(
  size_t start_idx, double ahead_dist,
  const autoware_internal_planning_msgs::PathWithLaneId & path);
autoware_utils_geometry::Point2d calculateOffsetPoint2d(const geometry_msgs::Pose & pose, double offset_x, double offset_y);
size_t calcSegmentIndexFromPointIndex(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & point, size_t idx);
std::vector<int64_t> getSortedLaneIdsFromPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path);
std::vector<int64_t> getSubsequentLaneIdsSetOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path, int64_t base_lane_id);
std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose);
std::set<int64_t> getLaneIdSetOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose);
autoware_utils_geometry::LineString2d extendSegmentToBounds(
  const lanelet::BasicLineString2d & segment, const std::vector<geometry_msgs::Point> & bound1,
  const std::vector<geometry_msgs::Point> & bound2);
lanelet::Ids collectConnectedLaneIds(
  int64_t lane_id, const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);
std::optional<int64_t> getNearestLaneId(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose);
template <class T>
std::unordered_map<std::shared_ptr<const T>, lanelet::ConstLanelet> getRegElemMapOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose)
{
  std::unordered_map<std::shared_ptr<const T>, lanelet::ConstLanelet> result;
  if (!lanelet_map) return result;
  const auto nearest = getNearestLaneId(path, lanelet_map, current_pose);
  const auto lane_ids = nearest ? getSubsequentLaneIdsSetOnPath(path, *nearest) : getSortedLaneIdsFromPath(path);
  for (const auto lane_id : lane_ids) {
    if (!lanelet_map->laneletLayer.exists(lane_id)) continue;
    const auto lane = lanelet_map->laneletLayer.get(lane_id);
    for (const auto & element : lane.regulatoryElementsAs<const T>()) result.emplace(element, lane);
  }
  return result;
}

// Use the same route-relative regulatory-element selection for module lifetime
// decisions as for module creation. A raw lane-ID comparison is insufficient:
// one regulatory element can be attached to several lanelets.
template <class T>
std::set<int64_t> getRegElemIdSetOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose)
{
  std::set<int64_t> result;
  for (const auto & element_and_lane : getRegElemMapOnPath<T>(path, lanelet_map, current_pose)) {
    result.insert(element_and_lane.first->id());
  }
  return result;
}
template <class T>
std::set<int64_t> getLaneletIdSetOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map, const geometry_msgs::Pose & current_pose)
{
  std::set<int64_t> ids;
  for (const auto & entry : getRegElemMapOnPath<const T>(path, lanelet_map, current_pose)) {
    ids.insert(entry.second.id());
  }
  return ids;
}
}  // namespace autoware::behavior_velocity_planner::planning_utils

#endif
