#include <autoware/behavior_path_planner_common/marker_utils/utils.hpp>

#include <autoware_utils_geometry/geometry.hpp>
#include <lanelet2_core/primitives/CompoundPolygon.h>

#include <iomanip>
#include <sstream>

namespace marker_utils
{
namespace
{
visualization_msgs::Marker makeMarker(
  const std::string & ns, const float r, const float g, const float b)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;
  marker.pose.orientation.w = 1.0;
  return marker;
}
}  // namespace

visualization_msgs::MarkerArray createShiftLineMarkerArray(
  const autoware::behavior_path_planner::ShiftLineArray & shift_lines, const double base_shift,
  std::string ns, const float r, const float g, const float b, const float width)
{
  auto local_shift_lines = shift_lines;
  if (local_shift_lines.empty()) local_shift_lines.emplace_back();

  visualization_msgs::MarkerArray markers;
  markers.markers.reserve(local_shift_lines.size() * 3);
  double current_shift = base_shift;
  int id = 0;
  for (const auto & shift_line : local_shift_lines) {
    const auto basic = makeMarker(ns, r, g, b);
    auto start = basic;
    start.id = ++id;
    start.pose = autoware_utils_geometry::calc_offset_pose(
      shift_line.start, 0.0, current_shift, 0.0);
    markers.markers.push_back(start);

    auto end = basic;
    end.id = ++id;
    end.pose = autoware_utils_geometry::calc_offset_pose(
      shift_line.end, 0.0, shift_line.end_shift_length, 0.0);
    markers.markers.push_back(end);

    auto line = basic;
    line.id = ++id;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = width;
    line.scale.y = 0.0;
    line.scale.z = 0.0;
    line.points = {start.pose.position, end.pose.position};
    markers.markers.push_back(line);
    current_shift = shift_line.end_shift_length;
  }
  return markers;
}

visualization_msgs::MarkerArray createPoseMarkerArray(
  const geometry_msgs::Pose & pose, std::string ns, const int32_t id, const float r,
  const float g, const float b)
{
  auto marker = makeMarker(ns, r, g, b);
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.7;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.pose = pose;
  visualization_msgs::MarkerArray result;
  result.markers.push_back(std::move(marker));
  return result;
}

visualization_msgs::MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, std::string ns, const int64_t lane_id, const float r,
  const float g, const float b)
{
  visualization_msgs::MarkerArray result;
  auto marker = makeMarker(ns, r, g, b);
  marker.id = static_cast<int32_t>(lane_id);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.15;
  marker.color.a = 0.999;
  marker.points.reserve(path.points.size());
  for (const auto & point : path.points) marker.points.push_back(point.point.pose.position);
  if (!marker.points.empty()) result.markers.push_back(std::move(marker));
  return result;
}

visualization_msgs::MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> & shift_distance, const PathWithLaneId & reference, std::string ns,
  const float r, const float g, const float b)
{
  visualization_msgs::MarkerArray result;
  if (shift_distance.size() != reference.points.size()) return result;
  auto marker = makeMarker(ns, r, g, b);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.a = 0.9;
  for (size_t i = 0; i < shift_distance.size(); ++i) {
    marker.points.push_back(autoware_utils_geometry::calc_offset_pose(
      reference.points.at(i).point.pose, 0.0, shift_distance.at(i), 0.0).position);
  }
  result.markers.push_back(std::move(marker));
  return result;
}

visualization_msgs::MarkerArray createShiftGradMarkerArray(
  const std::vector<double> & grad, const std::vector<double> & shift_distance,
  const PathWithLaneId & reference, std::string ns, const float r, const float g, const float b)
{
  visualization_msgs::MarkerArray result;
  if (grad.size() != reference.points.size() || shift_distance.size() != reference.points.size()) {
    return result;
  }
  int id = 0;
  for (size_t i = 0; i < grad.size(); ++i) {
    auto marker = makeMarker(ns, r, g, b);
    marker.id = id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.3;
    marker.pose = autoware_utils_geometry::calc_offset_pose(
      reference.points.at(i).point.pose, 0.0, shift_distance.at(i), 0.0);
    marker.text = "Grad:" + std::to_string(grad.at(i));
    result.markers.push_back(std::move(marker));
  }
  return result;
}

visualization_msgs::MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, std::string ns, const float r,
  const float g, const float b)
{
  visualization_msgs::MarkerArray result;
  for (const auto & lanelet : lanelets) {
    auto marker = makeMarker(ns, r, g, b);
    marker.id = static_cast<int32_t>(lanelet.id());
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;
    for (const auto & point : lanelet.polygon3d()) {
      geometry_msgs::Point output;
      output.x = point.x(); output.y = point.y(); output.z = point.z();
      marker.points.push_back(output);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
      result.markers.push_back(std::move(marker));
    }
  }
  return result;
}

visualization_msgs::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::Polygon & polygon, std::string ns, const int64_t lane_id, const float r,
  const float g, const float b, const float width)
{
  auto marker = makeMarker(ns, r, g, b);
  marker.id = static_cast<int32_t>(lane_id);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = width;
  marker.color.a = 0.8;
  for (const auto & point : polygon.points) {
    geometry_msgs::Point output;
    output.x = point.x; output.y = point.y; output.z = point.z;
    marker.points.push_back(output);
  }
  if (!marker.points.empty()) marker.points.push_back(marker.points.front());
  visualization_msgs::MarkerArray result;
  result.markers.push_back(std::move(marker));
  return result;
}

visualization_msgs::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::PredictedObjects & objects, std::string ns,
  const int64_t lane_id, const float r, const float g, const float b)
{
  visualization_msgs::MarkerArray result;
  int id = static_cast<int>(lane_id << 16);
  for (const auto & object : objects.objects) {
    auto marker = makeMarker(ns, r, g, b);
    marker.id = id++;
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker.scale.x = object.shape.dimensions.x > 0.0 ? object.shape.dimensions.x : 1.0;
    marker.scale.y = object.shape.dimensions.y > 0.0 ? object.shape.dimensions.y : 1.0;
    marker.scale.z = object.shape.dimensions.z > 0.0 ? object.shape.dimensions.z : 1.0;
    marker.color.a = 0.8;
    result.markers.push_back(std::move(marker));
  }
  return result;
}

visualization_msgs::MarkerArray createDrivableLanesMarkerArray(
  const std::vector<DrivableLanes> & drivable_lanes, std::string ns)
{
  visualization_msgs::MarkerArray result;
  auto append_lane = [&](const lanelet::ConstLanelet & lane, const float r, const float g,
                         const float b) {
    auto markers = createLaneletsAreaMarkerArray({lane}, ns, r, g, b);
    result.markers.insert(result.markers.end(), markers.markers.begin(), markers.markers.end());
  };
  for (const auto & lanes : drivable_lanes) {
    append_lane(lanes.right_lane, 1.0, 0.0, 0.0);
    append_lane(lanes.left_lane, 0.0, 1.0, 0.0);
    for (const auto & lane : lanes.middle_lanes) append_lane(lane, 0.0, 0.0, 1.0);
  }
  return result;
}

visualization_msgs::MarkerArray showPolygon(
  const CollisionCheckDebugMap & object_debug, std::string ns)
{
  visualization_msgs::MarkerArray result;
  int id = 0;
  for (const auto & entry : object_debug) {
    const auto & debug = entry.second;
    const float r = debug.is_safe ? 0.0F : 1.0F;
    const float g = debug.is_safe ? 1.0F : 0.0F;
    for (const auto * polygon : {&debug.extended_ego_polygon, &debug.extended_obj_polygon}) {
      auto marker = makeMarker(ns, r, g, 0.0F);
      marker.id = id++;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 0.2;
      for (const auto & point : polygon->outer()) {
        geometry_msgs::Point output;
        output.x = point.x(); output.y = point.y(); output.z = debug.current_obj_pose.position.z;
        marker.points.push_back(output);
      }
      if (!marker.points.empty()) result.markers.push_back(std::move(marker));
    }
  }
  return result;
}

visualization_msgs::MarkerArray showPredictedPath(
  const CollisionCheckDebugMap & object_debug, std::string ns)
{
  visualization_msgs::MarkerArray result;
  int id = 0;
  const auto append = [&](const auto & path, const float r, const float g, const float b) {
    for (const auto & stamped : path) {
      auto marker = makeMarker(ns, r, g, b);
      marker.id = id++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.scale.x = 1.0; marker.scale.y = 0.3; marker.scale.z = 0.3;
      marker.pose = stamped.pose;
      result.markers.push_back(std::move(marker));
    }
  };
  for (const auto & entry : object_debug) {
    append(entry.second.ego_predicted_path, 0.0, 1.0, 1.0);
    append(entry.second.obj_predicted_path, 1.0, 1.0, 0.0);
  }
  return result;
}

visualization_msgs::MarkerArray showSafetyCheckInfo(
  const CollisionCheckDebugMap & object_debug, std::string ns)
{
  visualization_msgs::MarkerArray result;
  int id = 0;
  for (const auto & entry : object_debug) {
    const auto & debug = entry.second;
    auto marker = makeMarker(ns, 0.0, 1.0, 1.0);
    marker.id = id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.5;
    marker.pose = debug.current_obj_pose;
    std::ostringstream text;
    text << "Unsafe reason: " << debug.unsafe_reason << "\nRSS dist: "
         << std::setprecision(4) << debug.rss_longitudinal << "\nSafe: "
         << (debug.is_safe ? "Yes" : "No");
    marker.text = text.str();
    result.markers.push_back(std::move(marker));
  }
  return result;
}
}  // namespace marker_utils
