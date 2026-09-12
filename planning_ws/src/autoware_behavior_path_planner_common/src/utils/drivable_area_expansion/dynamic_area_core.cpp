#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/dynamic_area_core.hpp>
#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/path_projection.hpp>

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <boost/geometry/algorithms/intersects.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
namespace
{
double cross_product(const Point2d & a, const Point2d & b, const Point2d & p)
{
  return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x());
}

bool segment_intersects(const Point2d & a, const Point2d & b, const Point2d & c, const Point2d & d)
{
  constexpr double epsilon = 1e-9;
  const auto between = [epsilon](const double value, const double first, const double second) {
    return value >= std::min(first, second) - epsilon && value <= std::max(first, second) + epsilon;
  };
  const auto on_segment = [&](const Point2d & first, const Point2d & second, const Point2d & point) {
    return std::abs(cross_product(first, second, point)) <= epsilon &&
           between(point.x(), first.x(), second.x()) && between(point.y(), first.y(), second.y());
  };
  const auto opposite = [](const double first, const double second) {
    return (first < 0.0 && second > 0.0) || (first > 0.0 && second < 0.0);
  };
  const double ab_c = cross_product(a, b, c), ab_d = cross_product(a, b, d);
  const double cd_a = cross_product(c, d, a), cd_b = cross_product(c, d, b);
  return (opposite(ab_c, ab_d) && opposite(cd_a, cd_b)) || on_segment(a, b, c) ||
         on_segment(a, b, d) || on_segment(c, d, a) || on_segment(c, d, b);
}

double segment_distance(const Point2d & a, const Point2d & b, const Point2d & c, const Point2d & d)
{
  if (segment_intersects(a, b, c, d)) return 0.0;
  return std::min({
    point_to_segment_projection(a, c, d).distance, point_to_segment_projection(b, c, d).distance,
    point_to_segment_projection(c, a, b).distance, point_to_segment_projection(d, a, b).distance});
}
}  // namespace

void expand_drivable_area(PathWithLaneId & path, const std::shared_ptr<const autoware::behavior_path_planner::PlannerData> & planner_data)
{
  if (path.points.size() < 3 || path.left_bound.empty() || path.right_bound.empty() || !planner_data ||
      !planner_data->route_handler || !planner_data->self_odometry || !planner_data->dynamic_object) return;
  const auto map = planner_data->route_handler->getLaneletMapPtr();
  if (!map) return;
  const auto & params = planner_data->drivable_area_expansion_parameters;
  const auto segments = extract_uncrossable_segments(*map, planner_data->self_odometry->pose.pose.position, params);
  const auto footprints = create_object_footprints(*planner_data->dynamic_object, params);
  const std::vector<Polygon2d> polygons{footprints.begin(), footprints.end()};
  auto poses = planner_data->drivable_area_expansion_prev_path_poses;
  auto curvatures = planner_data->drivable_area_expansion_prev_curvatures;
  reuse_previous_poses(path, poses, curvatures, planner_data->self_odometry->pose.pose.position, params);
  if (poses.size() < 3) return;
  add_bound_points(path.left_bound, path.right_bound, poses, params.min_bound_interval);
  try {
    if (poses.size() > curvatures.size()) {
      const auto new_curvatures = calculate_smoothed_curvatures(poses, params.curvature_average_window);
      curvatures.insert(curvatures.end(), new_curvatures.begin() + curvatures.size(), new_curvatures.end());
    }
  } catch (const std::exception &) {
    curvatures.assign(poses.size(), 0.0);
  }
  const auto expansion = calculate_expansion(poses, path.left_bound, path.right_bound, curvatures, params);
  const auto max_left = calculate_maximum_distance(path.left_bound, segments, polygons, params, LEFT);
  const auto max_right = calculate_maximum_distance(path.right_bound, segments, polygons, params, RIGHT);
  auto distances = expansion;
  calculate_expansion_distances(distances, max_left, max_right);
  apply_arc_length_range_smoothing(distances, path.left_bound, params.arc_length_range, LEFT);
  apply_arc_length_range_smoothing(distances, path.right_bound, params.arc_length_range, RIGHT);
  apply_bound_change_rate_limit(distances.left_distances, path.left_bound, params.max_bound_rate);
  apply_bound_change_rate_limit(distances.right_distances, path.right_bound, params.max_bound_rate);
  for (size_t index = 0; index < distances.left_distances.size() && index < max_left.size(); ++index) distances.left_distances[index] = std::min(distances.left_distances[index], max_left[index]);
  for (size_t index = 0; index < distances.right_distances.size() && index < max_right.size(); ++index) distances.right_distances[index] = std::min(distances.right_distances[index], max_right[index]);
  expand_bound(path.left_bound, poses, distances.left_distances);
  expand_bound(path.right_bound, poses, distances.right_distances);
  planner_data->drivable_area_expansion_prev_path_poses = std::move(poses);
  planner_data->drivable_area_expansion_prev_curvatures = std::move(curvatures);
}

void reuse_previous_poses(const PathWithLaneId & path, std::vector<geometry_msgs::Pose> & previous_poses, std::vector<double> & previous_curvatures, const geometry_msgs::Point & ego_point, const DrivableAreaExpansionParameters & params)
{
  std::vector<geometry_msgs::Pose> cropped_poses;
  std::vector<double> cropped_curvatures;
  const auto ego_is_behind = previous_poses.size() > 1 &&
    autoware::motion_utils::calcLongitudinalOffsetToSegment(previous_poses, 0, ego_point) < 0.0;
  // This intentionally preserves the source predicate, including its strict
  // negative-distance check.
  const auto ego_is_far = !previous_poses.empty() &&
    point_distance({ego_point.x, ego_point.y}, {previous_poses.front().position.x, previous_poses.front().position.y}) < 0.0;
  LineString2d left_bound, right_bound, previous_linestring;
  for (const auto & point : path.left_bound) left_bound.push_back({point.x, point.y});
  for (const auto & point : path.right_bound) right_bound.push_back({point.x, point.y});
  for (const auto & pose : previous_poses) previous_linestring.push_back({pose.position.x, pose.position.y});
  const auto previous_across_bounds = boost::geometry::intersects(left_bound, previous_linestring) ||
    boost::geometry::intersects(right_bound, previous_linestring);
  if (!ego_is_behind && !ego_is_far && previous_poses.size() > 1 && !previous_across_bounds) {
    const auto first_index = autoware::motion_utils::findNearestSegmentIndex(previous_poses, path.points.front().point.pose);
    const auto deviation = autoware::motion_utils::calcLateralOffset(previous_poses, path.points.front().point.pose.position);
    if (first_index && deviation < params.max_reuse_deviation) {
      LineString2d path_linestring;
      for (const auto & point : path.points) path_linestring.push_back({point.point.pose.position.x, point.point.pose.position.y});
      for (size_t index = *first_index; index < previous_poses.size(); ++index) {
        double lateral_offset = std::numeric_limits<double>::max();
        for (size_t segment = 0; segment + 1 < path_linestring.size(); ++segment) {
          lateral_offset = std::min(lateral_offset, point_to_line_projection(
            {previous_poses[index].position.x, previous_poses[index].position.y},
            path_linestring[segment], path_linestring[segment + 1]).distance);
        }
        if (lateral_offset > params.max_reuse_deviation) break;
        cropped_poses.push_back(previous_poses[index]);
        if (index < previous_curvatures.size()) cropped_curvatures.push_back(previous_curvatures[index]);
      }
    }
  }
  if (cropped_poses.empty()) {
    const auto resampled = autoware::motion_utils::resamplePath(path, params.resample_interval, true, true, false).points;
    const auto cropped = params.max_path_arc_length <= 0.0 ? resampled :
      autoware::motion_utils::cropForwardPoints(resampled, resampled.front().point.pose.position, 0, params.max_path_arc_length);
    for (const auto & point : cropped) cropped_poses.push_back(point.point.pose);
  } else {
    const auto initial_arc_length = autoware::motion_utils::calcArcLength(cropped_poses);
    const auto max_path_arc_length = autoware::motion_utils::calcArcLength(path.points);
    const auto first_arc_length = autoware::motion_utils::calcSignedArcLength(path.points, path.points.front().point.pose.position, cropped_poses.back().position);
    for (auto arc_length = first_arc_length + params.resample_interval;
      (params.max_path_arc_length <= 0.0 || initial_arc_length + (arc_length - first_arc_length) <= params.max_path_arc_length) && arc_length <= max_path_arc_length;
      arc_length += params.resample_interval) {
      cropped_poses.push_back(autoware::motion_utils::calcInterpolatedPose(path.points, arc_length));
    }
  }
  previous_poses = autoware::motion_utils::removeOverlapPoints(cropped_poses);
  previous_curvatures = cropped_curvatures;
}

double calculate_minimum_lane_width(const double curvature_radius, const DrivableAreaExpansionParameters & params)
{
  const auto k = curvature_radius;
  const auto a = params.vehicle_info.front_overhang_m + params.extra_front_overhang;
  const auto w = params.vehicle_info.vehicle_width_m + params.extra_width;
  const auto l = params.vehicle_info.wheel_base_m + params.extra_wheelbase;
  return (a * a + 2.0 * a * l + 2.0 * k * w + l * l + w * w) / (2.0 * k + w);
}
Expansion calculate_expansion(const std::vector<geometry_msgs::Pose> & poses, const std::vector<geometry_msgs::Point> & left_bound, const std::vector<geometry_msgs::Point> & right_bound, const std::vector<double> & curvatures, const DrivableAreaExpansionParameters & params)
{
  Expansion expansion;
  expansion.min_lane_widths.resize(poses.size(), 0.0);
  for (size_t index = 0; index < poses.size() && index < curvatures.size(); ++index) {
    if (curvatures[index] != 0.0) {
      expansion.min_lane_widths[index] = calculate_minimum_lane_width(1.0 / curvatures[index], params);
    }
  }
  calculate_bound_index_mappings(expansion, poses, left_bound, LEFT);
  calculate_bound_index_mappings(expansion, poses, right_bound, RIGHT);
  return expansion;
}
void apply_bound_change_rate_limit(std::vector<double> & distances, const std::vector<geometry_msgs::Point> & bound, const double max_rate)
{
  if (distances.empty()) return;
  const auto apply_max_vel = [&](auto from, auto to) {
    if (distances[from] > distances[to]) {
      const auto & a = bound[from]; const auto & b = bound[to];
      const double arc_length = std::hypot(a.x - b.x, a.y - b.y);
      distances[to] = std::max(distances[to], distances[from] - arc_length * max_rate);
    }
  };
  for (auto i = 0UL; i + 1 < distances.size(); ++i) apply_max_vel(i, i + 1);
  for (auto i = distances.size() - 1; i > 0; --i) apply_max_vel(i, i - 1);
}
void calculate_expansion_distances(Expansion & e, const std::vector<double> & left, const std::vector<double> & right)
{
  e.left_distances.resize(left.size(), 0.0); e.right_distances.resize(right.size(), 0.0);
  for (size_t i = 0; i < e.min_lane_widths.size(); ++i) {
    const auto l = e.left_bound_indexes[i], r = e.right_bound_indexes[i];
    const double extra = e.min_lane_widths[i] - e.left_projections[i].distance - e.right_projections[i].distance;
    if (extra <= 0.0) continue;
    const double ll = std::min(left[l], left[l + 1]), rl = std::min(right[r], right[r + 1]), half = extra / 2.0;
    if (half < ll && half < rl) { e.left_distances[l] = e.left_distances[l + 1] = e.right_distances[r] = e.right_distances[r + 1] = half; }
    else if (half >= ll) { e.left_distances[l] = e.left_distances[l + 1] = ll; const double v = std::min(rl, half + half - ll); e.right_distances[r] = e.right_distances[r + 1] = v; }
    else { e.right_distances[r] = e.right_distances[r + 1] = rl; const double v = std::min(ll, half + half - rl); e.left_distances[l] = e.left_distances[l + 1] = v; }
  }
}
void add_bound_points(std::vector<geometry_msgs::Point> & left, std::vector<geometry_msgs::Point> & right, const std::vector<geometry_msgs::Pose> & poses, const double minimum_interval)
{
  const auto add = [minimum_interval](auto & bound, const auto & pose) {
    if (bound.size() < 2) return;
    const Point2d p{pose.position.x, pose.position.y}; size_t best = 0; PointDistance projection; projection.distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < bound.size(); ++i) { const auto candidate = point_to_segment_projection(p, {bound[i].x,bound[i].y}, {bound[i+1].x,bound[i+1].y}); if (candidate.distance < projection.distance) { projection=candidate; best=i; } }
    geometry_msgs::Point inserted; inserted.x=projection.point.x(); inserted.y=projection.point.y(); inserted.z=bound[best].z;
    if (std::hypot(inserted.x-bound[best].x,inserted.y-bound[best].y)>minimum_interval && std::hypot(inserted.x-bound[best+1].x,inserted.y-bound[best+1].y)>minimum_interval) bound.insert(bound.begin()+best+1,inserted);
  };
  for (const auto & pose : poses) { add(left,pose); add(right,pose); }
}
std::vector<double> calculate_smoothed_curvatures(const std::vector<geometry_msgs::Pose> & poses, const size_t window)
{
  std::vector<double> raw(poses.size(),0.0), smooth(poses.size(),0.0); if (poses.size()<3) return smooth;
  for(size_t i=1;i+1<poses.size();++i){ const auto &a=poses[i-1].position,&b=poses[i].position,&c=poses[i+1].position; const double ab=std::hypot(a.x-b.x,a.y-b.y),bc=std::hypot(b.x-c.x,b.y-c.y),ca=std::hypot(c.x-a.x,c.y-a.y),area=std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/2.; raw[i]=(ab*bc*ca)==0.?0.:4.*area/(ab*bc*ca); }
  raw.front()=raw[1]; raw.back()=raw[raw.size()-2]; for(size_t i=0;i<raw.size();++i){ const size_t from=i>=window?i-window:0,to=std::min(raw.size()-1,i+window); for(size_t j=from;j<=to;++j)smooth[i]+=std::abs(raw[j]); smooth[i]/=static_cast<double>(to-from+1); } return smooth;
}
void calculate_bound_index_mappings(Expansion & e, const std::vector<geometry_msgs::Pose> & poses, const std::vector<geometry_msgs::Point> & bound, const Side side)
{
  auto & indexes = side == LEFT ? e.left_bound_indexes : e.right_bound_indexes;
  auto & projections = side == LEFT ? e.left_projections : e.right_projections;
  indexes.assign(poses.size(), 0); projections.assign(poses.size(), {{}, std::numeric_limits<double>::max()}); size_t lower = 0;
  for (size_t p = 0; p < poses.size(); ++p) { const Point2d point{poses[p].position.x, poses[p].position.y}; for (size_t b = lower; b + 1 < bound.size(); ++b) { const auto projection = point_to_segment_projection(point, {bound[b].x,bound[b].y}, {bound[b+1].x,bound[b+1].y}); if (projection.distance < projections[p].distance) { indexes[p]=b; projections[p]=projection; } } lower=indexes[p]; }
}
void apply_arc_length_range_smoothing(Expansion & e, const std::vector<geometry_msgs::Point> & bound, const double range, const Side side)
{
  const auto & indexes = side == LEFT ? e.left_bound_indexes : e.right_bound_indexes; const auto & projections = side == LEFT ? e.left_projections : e.right_projections; auto & distances = side == LEFT ? e.left_distances : e.right_distances; const auto original=distances;
  for(size_t p=0;p<indexes.size();++p){ const size_t index=indexes[p]; if(index+1>=bound.size()) continue; double arc=point_distance(projections[p].point,{bound[index+1].x,bound[index+1].y}); for(size_t b=index+2;b<bound.size()&&arc<=range;++b){arc+=std::hypot(bound[b].x-bound[b-1].x,bound[b].y-bound[b-1].y); distances[b]=std::max(distances[b],original[index]);} arc=point_distance(projections[p].point,{bound[index].x,bound[index].y}); for(size_t d=1;d<index&&arc<=range;++d){const size_t b=index-d;arc+=std::hypot(bound[b+1].x-bound[b].x,bound[b+1].y-bound[b].y);distances[b]=std::max(distances[b],original[index]);}}
}
void expand_bound(std::vector<geometry_msgs::Point> & bound, const std::vector<geometry_msgs::Pose> & poses, const std::vector<double> & expansions)
{
  if (bound.empty() || poses.size() < 2 || bound.size() != expansions.size()) return;
  LineString2d path; for (const auto & pose : poses) path.push_back({pose.position.x, pose.position.y});
  for (size_t i = 0; i < bound.size(); ++i) if (expansions[i] > 0.0) {
    const Point2d original{bound[i].x,bound[i].y}; const auto projection = point_to_linestring_projection(original,path);
    if (projection.distance == 0.0) continue;
    auto expanded = lerp_point(projection.projected_point,original,(expansions[i]+projection.distance)/projection.distance);
    const auto reprojected = point_to_linestring_projection(expanded,path);
    if (reprojected.distance > 0.0 && reprojected.distance < projection.distance) expanded=lerp_point(reprojected.projected_point,expanded,projection.distance/reprojected.distance);
    bound[i].x=expanded.x(); bound[i].y=expanded.y();
  }
  // Preserve the ROS2 loop-removal semantics: a shared endpoint is not a loop.
  std::vector<geometry_msgs::Point> no_loop_bound{bound.front()};
  for (size_t idx = 1; idx < bound.size(); ++idx) {
    bool is_intersecting = false;
    for (size_t succ_idx = idx + 1; succ_idx < bound.size(); ++succ_idx) {
      const auto intersection = autoware_utils_geometry::intersect(
        bound[idx - 1], bound[idx], bound[succ_idx - 1], bound[succ_idx]);
      if (intersection &&
          autoware_utils_geometry::calc_distance2d(*intersection, bound[idx - 1]) > 1e-3 &&
          autoware_utils_geometry::calc_distance2d(*intersection, bound[idx]) > 1e-3) {
        idx = succ_idx;
        is_intersecting = true;
      }
    }
    if (!is_intersecting) no_loop_bound.push_back(bound[idx]);
  }
  bound = std::move(no_loop_bound);
}
std::vector<double> calculate_maximum_distance_from_segments(const std::vector<geometry_msgs::Point> & bound, const SegmentRtree & segments, const DrivableAreaExpansionParameters & params, const Side side)
{
  return calculate_maximum_distance(bound, segments, {}, params, side);
}
std::vector<double> calculate_maximum_distance(const std::vector<geometry_msgs::Point> & bound, const SegmentRtree & segments, const std::vector<Polygon2d> & polygons, const DrivableAreaExpansionParameters & params, const Side side)
{
  std::vector<double> limits(bound.size(), std::numeric_limits<double>::max());
  for(size_t i=0;i+1<bound.size();++i){
    const Point2d a{bound[i].x,bound[i].y},b{bound[i+1].x,bound[i+1].y};
    const auto correct_side = [&](const Point2d & point) { return cross_product(a,b,point)*(side==LEFT?-1.:1.)<=0.; };
    double best=std::numeric_limits<double>::max();
    for(const auto & s:segments) if(correct_side(s.first)||correct_side(s.second)) best=std::min(best,segment_distance(a,b,s.first,s.second));
    for(const auto & polygon:polygons){
      const auto & ring=polygon.outer(); if(ring.size()<2) continue;
      bool intersects=false; double polygon_best=std::numeric_limits<double>::max();
      for(size_t vertex=0;vertex+1<ring.size();++vertex){ const Point2d first{ring[vertex].x(),ring[vertex].y()}, second{ring[vertex+1].x(),ring[vertex+1].y()}; intersects=intersects||segment_intersects(a,b,first,second); polygon_best=std::min(polygon_best,segment_distance(a,b,first,second)); }
      if(intersects){best=0.0; break;}
      if(std::all_of(ring.begin(),ring.end(),correct_side)) best=std::min(best,polygon_best);
    }
    if(best!=std::numeric_limits<double>::max()){const double value=std::max(0.,best-params.avoid_linestring_dist);limits[i]=limits[i+1]=value;}
  }
  if(params.max_expansion_distance>0.)for(auto &v:limits)v=std::min(v,params.max_expansion_distance); return limits;
}
}
