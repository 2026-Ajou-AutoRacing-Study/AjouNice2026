#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/deprecated/boundary_departure_checker/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <boost/geometry.hpp>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>

namespace autoware::behavior_path_planner::utils
{
namespace
{
using Path = autoware_internal_planning_msgs::PathWithLaneId;
using PathPoint = autoware_internal_planning_msgs::PathPointWithLaneId;

template <class PointT>
geometry_msgs::Pose lanelet_point_to_pose(
  const PointT & src_point, const lanelet::ConstLanelet & target_lane)
{
  const auto point = autoware::experimental::lanelet2_utils::to_ros(src_point);
  const auto yaw = autoware::experimental::lanelet2_utils::get_lanelet_angle(
    target_lane, autoware::experimental::lanelet2_utils::from_ros(point).basicPoint());
  geometry_msgs::Pose pose;
  pose.position = point;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

template <class PolygonT>
autoware_utils_geometry::Polygon2d to_boost_polygon(const PolygonT & source)
{
  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : source) {
    converted.outer().emplace_back(point.x(), point.y());
  }
  if (!converted.outer().empty()) {
    converted.outer().push_back(converted.outer().front());
  }
  boost::geometry::correct(converted);
  return converted;
}

template <class PolygonT>
bool is_footprint_within_polygon(
  const PolygonT & lane_polygon, const geometry_msgs::Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin)
{
  const auto footprint = autoware_utils_geometry::to_footprint(
    current_pose, common_param.base_link2front, common_param.base_link2rear,
    common_param.vehicle_width);
  const auto polygon = to_boost_polygon(lane_polygon);
  return std::all_of(footprint.outer().begin(), footprint.outer().end(), [&](const auto & point) {
    return boost::geometry::distance(point, polygon) <= std::max(outer_margin, 0.0);
  });
}
}  // namespace

geometry_msgs::Pose to_geom_msg_pose(
  const lanelet::BasicPoint3d & src_point, const lanelet::ConstLanelet & target_lane)
{
  return lanelet_point_to_pose(src_point, target_lane);
}

geometry_msgs::Pose to_geom_msg_pose(
  const lanelet::BasicPoint2d & src_point, const lanelet::ConstLanelet & target_lane)
{
  return lanelet_point_to_pose(src_point, target_lane);
}

bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const geometry_msgs::Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin)
{
  const auto combined = autoware::experimental::lanelet2_utils::combine_lanelets_shape(current_lanes);
  if (!combined) return false;
  return is_footprint_within_polygon(
    combined->polygon2d(), current_pose, common_param, outer_margin);
}

bool isEgoWithinOriginalLane(
  const lanelet::BasicPolygon2d & lane_polygon, const geometry_msgs::Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin)
{
  return is_footprint_within_polygon(
    lane_polygon, current_pose, common_param, outer_margin);
}

double getSignedDistanceFromLaneBoundary(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::Point & position, const bool left_side)
{
  const auto lanelet_point = autoware::experimental::lanelet2_utils::from_ros(position);
  const auto & boundary = left_side ? lanelet.leftBound2d() : lanelet.rightBound2d();
  return lanelet::geometry::toArcCoordinates(
    boundary, lanelet::utils::to2D(lanelet_point).basicPoint()).distance;
}

double getDistanceToNextIntersection(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) return std::numeric_limits<double>::max();
  const auto current = autoware::experimental::lanelet2_utils::get_closest_lanelet(lanelets, current_pose);
  if (!current) return std::numeric_limits<double>::max();
  const auto arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelets, current_pose);
  double distance = 0.0;
  bool after_current = false;
  for (const auto & lane : lanelets) {
    after_current = after_current || lane.id() == current->id();
    if (after_current && lane.hasAttribute("turn_direction")) {
      const auto lane_change_allowed = [](const lanelet::ConstLineString3d & bound) {
        return bound.hasAttribute(lanelet::AttributeNamesString::LaneChange) &&
               bound.attribute(lanelet::AttributeNamesString::LaneChange).value() == "yes";
      };
      if (!lane_change_allowed(lane.leftBound()) && !lane_change_allowed(lane.rightBound())) {
        return distance - arc.length;
      }
    }
    distance += lanelet::geometry::length3d(lane);
  }
  return std::numeric_limits<double>::max();
}

double getDistanceToCrosswalk(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs)
{
  const auto & arc_coordinates =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelets, current_pose);

  const auto current_lanelet_opt =
    experimental::lanelet2_utils::get_closest_lanelet(lanelets, current_pose);
  if (!current_lanelet_opt) {
    return std::numeric_limits<double>::infinity();
  }
  const auto & current_lanelet = current_lanelet_opt.value();

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }

    if (is_after_current_lanelet) {
      const auto conflicting_crosswalks = overall_graphs.conflictingInGraph(llt, 1);
      if (!(conflicting_crosswalks.empty())) {
        // create centerline
        const lanelet::ConstLineString2d lanelet_centerline = llt.centerline2d();
        autoware_utils_geometry::LineString2d centerline;
        centerline.reserve(lanelet_centerline.size());
        for (const auto & point : lanelet_centerline) {
          boost::geometry::append(centerline, autoware_utils_geometry::Point2d(point.x(), point.y()));
        }

        // create crosswalk polygon and calculate distance
        double min_distance_to_crosswalk = std::numeric_limits<double>::infinity();
        for (const auto & crosswalk : conflicting_crosswalks) {
          lanelet::CompoundPolygon2d lanelet_crosswalk_polygon = crosswalk.polygon2d();
          autoware_utils_geometry::Polygon2d polygon;
          polygon.outer().reserve(lanelet_crosswalk_polygon.size() + 1);
          for (const auto & point : lanelet_crosswalk_polygon) {
            polygon.outer().emplace_back(point.x(), point.y());
          }
          polygon.outer().push_back(polygon.outer().front());

          std::vector<autoware_utils_geometry::Point2d> points_intersection;
          boost::geometry::intersection(centerline, polygon, points_intersection);

          for (const auto & point : points_intersection) {
            Pose pose_point;
            pose_point.position.x = point.x();
            pose_point.position.y = point.y();
            const lanelet::ArcCoordinates & arc_crosswalk =
              autoware::experimental::lanelet2_utils::get_arc_coordinates({llt}, pose_point);

            const double distance_to_crosswalk = arc_crosswalk.length;
            if (distance_to_crosswalk < min_distance_to_crosswalk) {
              min_distance_to_crosswalk = distance_to_crosswalk;
            }
          }
        }
        if (distance + min_distance_to_crosswalk > arc_coordinates.length) {
          return distance + min_distance_to_crosswalk - arc_coordinates.length;
        }
      }
    }
    distance += lanelet::geometry::length3d(llt);
  }

  return std::numeric_limits<double>::infinity();
}

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & target_lane,
  const geometry_msgs::Pose & pose)
{
  if (lanelet_sequence.empty()) return 0.0;
  const auto pose_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelet_sequence, pose);
  const auto center = target_lane.centerline().basicLineString();
  if (center.empty()) return 0.0;
  const auto front_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
    lanelet_sequence, to_geom_msg_pose(center.front(), target_lane));
  const auto back_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
    lanelet_sequence, to_geom_msg_pose(center.back(), target_lane));
  return std::max(std::min(front_arc.length - pose_arc.length, back_arc.length - pose_arc.length), 0.0);
}

autoware_utils_geometry::Polygon2d toPolygon2d(const lanelet::ConstLanelet & lanelet)
{
  return to_boost_polygon(lanelet.polygon2d().basicPolygon());
}

autoware_utils_geometry::Polygon2d toPolygon2d(const lanelet::BasicPolygon2d & polygon)
{
  return to_boost_polygon(polygon);
}

std::vector<lanelet::ConstLanelets> getPrecedingLanelets(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & target_lanes, const geometry_msgs::Pose & current_pose,
  const double backward_length)
{
  if (target_lanes.empty()) return {};
  const auto arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(target_lanes, current_pose);
  if (arc.length >= backward_length) return {};
  const auto & front = target_lanes.front();
  return route_handler.getPrecedingLaneletSequence(
    front, std::abs(backward_length - arc.length), {front});
}

bool isAllowedGoalModification(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  return route_handler &&
         (route_handler->isAllowedGoalModification() ||
          checkOriginalGoalIsInShoulder(route_handler));
}

bool checkOriginalGoalIsInShoulder(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  if (!route_handler) return false;
  const auto goal_pose = route_handler->getOriginalGoalPose();
  return !route_handler->getShoulderLaneletsAtPose(goal_pose).empty();
}

geometry_msgs::Pose refineGoal(
  const geometry_msgs::Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  const auto lanelet_point = autoware::experimental::lanelet2_utils::from_ros(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) return goal;

  const auto segment = autoware::experimental::lanelet2_utils::get_closest_segment(
    goal_lanelet.centerline(), lanelet_point.basicPoint());
  if (segment.empty()) return goal;

  const auto p1 = segment.front().basicPoint();
  const auto p2 = segment.back().basicPoint();
  const auto direction = (p2 - p1).normalized();
  const auto projected = p1 + direction * direction.dot(lanelet_point.basicPoint() - p1);

  geometry_msgs::Pose refined{};
  refined.position.x = projected.x();
  refined.position.y = projected.y();
  refined.position.z = projected.z();
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, std::atan2(direction.y(), direction.x()));
  refined.orientation = tf2::toMsg(orientation);
  return refined;
}

bool isInLanelets(
  const geometry_msgs::Pose & pose, const lanelet::ConstLanelets & lanes)
{
  return std::any_of(lanes.begin(), lanes.end(), [&pose](const auto & lane) {
    return autoware::experimental::lanelet2_utils::is_in_lanelet(pose, lane);
  });
}

double calcInterpolatedZ(
  const autoware_internal_planning_msgs::PathWithLaneId & input,
  const geometry_msgs::Point target_pos, const size_t seg_idx)
{
  const double closest_to_target_dist = autoware::motion_utils::calcSignedArcLength(
    input.points, input.points.at(seg_idx).point.pose.position,
    target_pos);  // TODO(murooka) implement calcSignedArcLength(points, idx, point)
  const double seg_dist =
    autoware::motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_z = input.points.at(seg_idx).point.pose.position.z;
  const double next_z = input.points.at(seg_idx + 1).point.pose.position.z;
  const double interpolated_z =
    std::abs(seg_dist) < 1e-6
      ? next_z
      : closest_z + (next_z - closest_z) * closest_to_target_dist / seg_dist;
  return interpolated_z;
}

std::optional<size_t> findIndexOutOfGoalSearchRange(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const Pose & goal, const int64_t goal_lane_id,
  const double max_dist = std::numeric_limits<double>::max())
{
  if (points.empty()) {
    return std::nullopt;
  }

  // find goal index
  size_t min_dist_index;
  {
    bool found = false;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & lane_ids = points.at(i).lane_ids;

      const double dist_to_goal = autoware_utils_geometry::calc_distance2d(points.at(i).point.pose, goal);
      const bool is_goal_lane_id_in_point =
        std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
      if (dist_to_goal < max_dist && dist_to_goal < min_dist && is_goal_lane_id_in_point) {
        min_dist_index = i;
        min_dist = dist_to_goal;
        found = true;
      }
    }
    if (!found) {
      return std::nullopt;
    }
  }

  // find index out of goal search range
  size_t min_dist_out_of_range_index = min_dist_index;
  for (int i = min_dist_index; 0 <= i; --i) {
    const double dist = autoware_utils_geometry::calc_distance2d(points.at(i).point, goal);
    min_dist_out_of_range_index = i;
    if (max_dist < dist) {
      break;
    }
  }

  return min_dist_out_of_range_index;
}

template <typename Iterator>
lanelet::ConstLanelets getUniqueLaneletsFromPath(
  Iterator begin, Iterator end,
  const std::function<lanelet::ConstLanelet(int64_t)> get_lanelet_by_id)
{
  std::set<int64_t> lanelet_ids;
  for (auto it = begin; it != end; ++it) {
    for (const auto & lane_id : it->lane_ids) {
      lanelet_ids.insert(lane_id);
    }
  }
  lanelet::ConstLanelets lanelets;
  for (const auto & lane_id : lanelet_ids) {
    lanelets.push_back(get_lanelet_by_id(lane_id));
  }
  return lanelets;
}

template <typename Iterator>
void fillLaneIdsFromMap(Iterator begin, Iterator end, const lanelet::ConstLanelets & lanelets)
{
  for (auto it = begin; it != end; ++it) {
    const auto point = it->point;
    if (const auto lanelet_opt =
          experimental::lanelet2_utils::get_closest_lanelet(lanelets, point.pose);
        lanelet_opt) {
      // TODO(hisaki): Writing "it->lane_ids = {lanelet.id()}" may cause a segmentation fault.
      // I'm not sure of the reason. (╥﹏╥)
      auto & ids = it->lane_ids;
      ids.clear();
      ids.push_back(lanelet_opt.value().id());
    }
  }
}

template <typename Iterator>
void fillLongitudinalVelocityFromInputPath(Iterator begin, Iterator end, PathWithLaneId input)
{
  if (input.points.size() < 2) {
    return;
  }
  input.points.pop_back();  // remove the last point because its velocity is 0.0
  // refine lane_ids and longitudinal velocity from input path
  for (auto it = begin; it != end; ++it) {
    auto idx = autoware::motion_utils::findNearestIndex(input.points, it->point.pose, 3.0, M_PI_4);
    if (idx) {
      const auto & input_point = input.points.at(idx.value());
      it->point.longitudinal_velocity_mps = input_point.point.longitudinal_velocity_mps;
    } else {
      const auto & input_point = input.points.back();
      it->point.longitudinal_velocity_mps = input_point.point.longitudinal_velocity_mps;
    }
  }
}

// goal does not have z
bool set_goal(
  const double search_radius_range, const double search_rad_range,
  const double output_path_interval, const PathWithLaneId & input, const Pose & goal,
  const int64_t goal_lane_id,
  const std::function<lanelet::ConstLanelet(int64_t)> & get_lanelet_by_id,
  PathWithLaneId * output_ptr)
{
  try {
    if (input.points.empty()) {
      return false;
    }

    // calculate refined_goal with interpolation
    // NOTE: goal does not have valid z, that will be calculated by interpolation here
    PathPointWithLaneId refined_goal{};
    const size_t closest_seg_idx_for_goal =
      findNearestSegmentIndex(input.points, goal, 3.0, M_PI_4);
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z =
      calcInterpolatedZ(input, goal.position, closest_seg_idx_for_goal);

    // Lambda function to create a refined goal point with interpolated z and velocity
    auto create_refined_goal_point = [&input,
                                      &goal](const double offset_distance) -> PathPointWithLaneId {
      PathPointWithLaneId refined_point{};
      refined_point.point.pose = autoware_utils_geometry::calc_offset_pose(goal, offset_distance, 0.0, 0.0);
      const size_t closest_seg_idx =
        findNearestSegmentIndex(input.points, refined_point.point.pose, 3.0, M_PI_4);
      refined_point.point.pose.position.z =
        calcInterpolatedZ(input, refined_point.point.pose.position, closest_seg_idx);
      return refined_point;
    };

    // calculate pre_refined_goal with interpolation
    // NOTE: z and velocity are filled
    constexpr double goal_to_pre_goal_distance = -1.0;
    const auto pre_refined_goal = create_refined_goal_point(goal_to_pre_goal_distance);

    // NOTE: add points for smooth spline interpolation between the input path and the
    // pre_refined_goal.
    constexpr double goal_to_pre_mid_goal_distance = -0.5;
    const auto pre_refined_mid_goal = create_refined_goal_point(goal_to_pre_mid_goal_distance);

    // find min_dist_out_of_circle_index whose distance to goal is longer than search_radius_range
    const auto min_dist_out_of_circle_index_opt =
      findIndexOutOfGoalSearchRange(input.points, goal, goal_lane_id, search_radius_range);
    if (!min_dist_out_of_circle_index_opt) {
      return false;
    }
    const size_t min_dist_out_of_circle_index = min_dist_out_of_circle_index_opt.value();

    // create output points
    output_ptr->points.reserve(output_ptr->points.size() + min_dist_out_of_circle_index + 5);
    for (size_t i = 0; i <= min_dist_out_of_circle_index; ++i) {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(pre_refined_mid_goal);
    output_ptr->points.push_back(refined_goal);

    // NOTE: this lane ids and longitudinal velocity will be overwritten by the input path in the
    // next step
    for (size_t i = output_ptr->points.size() - 3; i < output_ptr->points.size(); ++i) {
      output_ptr->points.at(i).lane_ids = input.points.back().lane_ids;
      output_ptr->points.at(i).point.longitudinal_velocity_mps =
        input.points.back().point.longitudinal_velocity_mps;
    }

    output_ptr->left_bound = input.left_bound;
    output_ptr->right_bound = input.right_bound;

    // NOTE: insert one more point before the start point to avoid the unexpected path change
    PathPointWithLaneId pre_start = output_ptr->points.front();
    pre_start.point.pose =
      autoware_utils_geometry::calc_offset_pose(pre_start.point.pose, -output_path_interval, 0.0, 0.0);
    output_ptr->points.insert(output_ptr->points.begin(), pre_start);
    *output_ptr = utils::resamplePathWithSpline(*output_ptr, output_path_interval, true);

    // NOTE: remove the first point to keep the original path length
    output_ptr->points.erase(output_ptr->points.begin());

    const auto lanelets = getUniqueLaneletsFromPath(
      input.points.begin() + min_dist_out_of_circle_index + 1, input.points.end(),
      get_lanelet_by_id);
    fillLaneIdsFromMap(
      output_ptr->points.begin() + min_dist_out_of_circle_index + 1, output_ptr->points.end(),
      lanelets);
    fillLongitudinalVelocityFromInputPath(
      output_ptr->points.begin() + min_dist_out_of_circle_index + 1, output_ptr->points.end(),
      input);

    output_ptr->points.back().point.longitudinal_velocity_mps = 0.0;
    return true;
  } catch (std::out_of_range & ex) {
    ROS_ERROR_STREAM(
      "failed to set goal: " << ex.what());
    return false;
  }
}


Path refinePathForGoal(
  const double search_radius_range, const double search_rad_range,
  const double output_path_interval, const Path & input, const geometry_msgs::Pose & goal,
  const int64_t goal_lane_id,
  const std::function<lanelet::ConstLanelet(int64_t)> & get_lanelet_by_id)
{
  Path filtered = input;
  filtered.points = autoware::motion_utils::removeOverlapPoints(filtered.points);
  if (!filtered.points.empty()) filtered.points.back().point.longitudinal_velocity_mps = 0.0;
  Path output;
  return set_goal(
           search_radius_range, search_rad_range, output_path_interval, filtered, goal, goal_lane_id,
           get_lanelet_by_id, &output)
           ? output
           : filtered;
}

std::optional<double> calc_point_dist_to_closest_lane_boundary(
  const autoware::route_handler::RouteHandler & route_handler,
  const geometry_msgs::Pose & ego_baselink_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::string & direction)
{
  if (direction != "left" && direction != "right") return std::nullopt;
  const auto vehicle_footprint = autoware_utils_geometry::transform_vector(
    vehicle_info.createFootprint(), autoware_utils_geometry::pose2transform(ego_baselink_pose));
  const auto corner_index = direction == "left"
    ? autoware::vehicle_info_utils::VehicleInfo::FrontLeftIndex
    : autoware::vehicle_info_utils::VehicleInfo::FrontRightIndex;
  geometry_msgs::Pose corner_pose;
  corner_pose.position = autoware_utils_geometry::to_msg(
    vehicle_footprint.at(corner_index).to_3d(ego_baselink_pose.position.z));
  corner_pose.orientation = ego_baselink_pose.orientation;
  lanelet::ConstLanelet lanelet;
  if (!route_handler.getClosestLaneletWithinRoute(corner_pose, &lanelet)) return std::nullopt;
  const auto & boundary = direction == "left" ? lanelet.leftBound() : lanelet.rightBound();
  return autoware::boundary_departure_checker::utils::calc_signed_lateral_distance_to_boundary(
    boundary, corner_pose);
}

std::optional<lanelet::Polygon3d> getPolygonByPoint(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstPoint3d & point, const std::string & polygon_name)
{
  if (!route_handler || !route_handler->getLaneletMapPtr()) return std::nullopt;
  const auto polygons = route_handler->getLaneletMapPtr()->polygonLayer.findUsages(point);
  for (const auto & polygon : polygons) {
    if (polygon.attributeOr(lanelet::AttributeName::Type, "none") == polygon_name) {
      return polygon;
    }
  }
  return std::nullopt;
}
}  // namespace autoware::behavior_path_planner::utils
