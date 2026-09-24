#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/velocity_smoother/smoother/smoother_base.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <tf2/utils.h>

#include <boost/geometry.hpp>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::motion_velocity_planner
{
std::optional<TrafficSignalStamped> PlannerData::get_traffic_signal(
  const lanelet::Id id, const bool keep_last_observation) const
{
  const auto & traffic_light_id_map =
    keep_last_observation ? traffic_light_id_map_last_observed_ : traffic_light_id_map_raw_;
  const auto it = traffic_light_id_map.find(id);
  if (it == traffic_light_id_map.end()) return std::nullopt;
  return it->second;
}

std::optional<double> PlannerData::calculate_min_deceleration_distance(
  const double target_velocity) const
{
  if (!velocity_smoother_) return std::nullopt;
  return autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    std::abs(current_odometry.twist.twist.linear.x), target_velocity,
    current_acceleration.accel.accel.linear.x, velocity_smoother_->getMinDecel(),
    std::abs(velocity_smoother_->getMinJerk()), velocity_smoother_->getMinJerk());
}

std::vector<StopPoint> PlannerData::calculate_map_stop_points(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory) const
{
  std::vector<StopPoint> stop_points;
  if (!route_handler || trajectory.size() < 2) return stop_points;
  autoware_utils_geometry::LineString2d trajectory_ls;
  for (const auto & point : trajectory) {
    trajectory_ls.emplace_back(point.pose.position.x, point.pose.position.y);
  }
  const auto candidates = route_handler->getLaneletMapPtr()->laneletLayer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));
  for (const auto & candidate : candidates) {
    for (const auto & stop_line : lanelet::utils::query::stopLinesLanelet(candidate)) {
      const auto stop_line_basic = lanelet::utils::to2D(stop_line).basicLineString();
      autoware_utils_geometry::LineString2d stop_line_2d;
      for (const auto & point : stop_line_basic) stop_line_2d.emplace_back(point.x(), point.y());
      autoware_utils_geometry::MultiPoint2d intersections;
      boost::geometry::intersection(trajectory_ls, stop_line_2d, intersections);
      for (const auto & intersection : intersections) {
        geometry_msgs::Point point;
        point.x = intersection.x();
        point.y = intersection.y();
        const double arc_length = autoware::motion_utils::calcSignedArcLength(trajectory, 0UL, point);
        StopPoint stop_point;
        stop_point.ego_trajectory_arc_length = arc_length - vehicle_info_.max_longitudinal_offset_m;
        if (stop_point.ego_trajectory_arc_length < 0.0) continue;
        stop_point.stop_line = stop_line_basic;
        stop_point.ego_stop_pose = autoware::motion_utils::calcInterpolatedPose(
          trajectory, stop_point.ego_trajectory_arc_length);
        stop_points.push_back(stop_point);
      }
    }
  }
  return stop_points;
}

void PlannerData::Pointcloud::preprocess_pointcloud(pcl::PointCloud<pcl::PointXYZ> input)
{
  pointcloud = std::move(input);
  auto filtered = pointcloud.makeShared();
  if (preprocess_params_.downsample_by_voxel_grid.enable_downsample && !filtered->empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(filtered);
    voxel_grid.setLeafSize(
      static_cast<float>(preprocess_params_.downsample_by_voxel_grid.voxel_size_x),
      static_cast<float>(preprocess_params_.downsample_by_voxel_grid.voxel_size_y),
      static_cast<float>(preprocess_params_.downsample_by_voxel_grid.voxel_size_z));
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_grid.filter(*downsampled);
    filtered = downsampled;
  }
  std::vector<pcl::PointIndices> clusters;
  if (preprocess_params_.euclidean_clustering.enable_clustering && !filtered->empty()) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(preprocess_params_.euclidean_clustering.cluster_tolerance);
    extraction.setMinClusterSize(preprocess_params_.euclidean_clustering.min_cluster_size);
    extraction.setMaxClusterSize(preprocess_params_.euclidean_clustering.max_cluster_size);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(filtered);
    extraction.extract(clusters);
  } else {
    clusters.resize(filtered->size());
    for (size_t i = 0; i < filtered->size(); ++i) clusters.at(i).indices.push_back(static_cast<int>(i));
  }
  filtered_pointcloud_ptr_ = filtered;
  cluster_indices_ = std::move(clusters);
}

void PlannerData::Pointcloud::preprocess_pointcloud(
  pcl::PointCloud<pcl::PointXYZ> input,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory,
  const nav_msgs::Odometry & current_odometry, const double min_deceleration_distance,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const double decimate_trajectory_step_length, const double goal_extended_trajectory_length,
  const bool enable_to_consider_current_pose, const double time_to_convergence)
{
  const auto & filter = preprocess_params_.filter_by_trajectory_polygon;
  if (raw_trajectory.empty() ||
      (!filter.enable_monolithic_crop_box && !filter.enable_multi_polygon_filtering)) {
    preprocess_pointcloud(std::move(input));
    return;
  }
  auto decimated = utils::decimate_trajectory_points_from_ego(
    raw_trajectory, current_odometry.pose.pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold, decimate_trajectory_step_length, goal_extended_trajectory_length);
  const double trim_length = filter.min_trajectory_length +
    min_deceleration_distance * filter.braking_distance_scale_factor;
  if (autoware::motion_utils::isDrivingForward(raw_trajectory).value_or(true) && !decimated.empty()) {
    decimated = autoware::motion_utils::cropForwardPoints(
      decimated, decimated.front().pose.position, 0U, trim_length);
  }
  const auto polygons = polygon_utils::createOneStepPolygons(
    decimated, vehicle_info, current_odometry.pose.pose, filter.lateral_margin,
    enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length);
  pcl::PointCloud<pcl::PointXYZ> filtered;
  filtered.header = input.header;
  for (const auto & point : input.points) {
    const autoware_utils_geometry::Point2d point_2d(point.x, point.y);
    bool keep = false;
    for (const auto & polygon : polygons) {
      if (boost::geometry::within(point_2d, polygon)) { keep = true; break; }
    }
    if (keep) filtered.push_back(point);
  }
  // The monolithic crop is a safe outer filter; exact polygon membership remains the final
  // condition whenever multi-polygon filtering is enabled.
  if (filter.enable_monolithic_crop_box && !polygons.empty()) {
    pcl::PointCloud<pcl::PointXYZ> cropped;
    double min_x = std::numeric_limits<double>::max(), min_y = min_x;
    double max_x = std::numeric_limits<double>::lowest(), max_y = max_x;
    for (const auto & polygon : polygons) for (const auto & p : polygon.outer()) {
      min_x = std::min(min_x, p.x()); max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y()); max_y = std::max(max_y, p.y());
    }
    for (const auto & point : (filter.enable_multi_polygon_filtering ? filtered.points : input.points)) {
      if (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y) cropped.push_back(point);
    }
    filtered = std::move(cropped);
  }
  preprocess_pointcloud(std::move(filtered));
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr PlannerData::Pointcloud::get_filtered_pointcloud_ptr() const
{
  if (!filtered_pointcloud_ptr_) {
    throw std::runtime_error("Filtered pointcloud pointer is not set. Please call preprocess_pointcloud() first.");
  }
  return *filtered_pointcloud_ptr_;
}

const std::vector<pcl::PointIndices> PlannerData::Pointcloud::get_cluster_indices() const
{
  if (!cluster_indices_) {
    throw std::runtime_error("Cluster indices are not set. Please call preprocess_pointcloud() first.");
  }
  return *cluster_indices_;
}

bool PlannerData::Pointcloud::has_processed_pointcloud() const noexcept
{
  return filtered_pointcloud_ptr_.has_value() && cluster_indices_.has_value();
}

pcl::PointCloud<pcl::PointXYZ> PlannerData::Pointcloud::extract_clustered_points() const
{
  const auto source = get_filtered_pointcloud_ptr();
  const auto clusters = get_cluster_indices();
  std::vector<int> indices;
  for (const auto & cluster : clusters) indices.insert(indices.end(), cluster.indices.begin(), cluster.indices.end());
  pcl::PointCloud<pcl::PointXYZ> extracted;
  pcl::copyPointCloud(*source, indices, extracted);
  return extracted;
}

double PlannerData::Object::get_dist_to_traj_poly(
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys) const
{
  if (!dist_to_traj_poly_) {
    const auto object_polygon = autoware_utils_geometry::to_polygon2d(predicted_object);
    double distance = std::numeric_limits<double>::max();
    for (const auto & trajectory_polygon : decimated_traj_polys) {
      distance = std::min(distance, boost::geometry::distance(trajectory_polygon, object_polygon));
    }
    dist_to_traj_poly_ = distance;
  }
  return *dist_to_traj_poly_;
}

double PlannerData::Object::get_dist_to_traj_lateral(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const
{
  if (!dist_to_traj_lateral_) {
    dist_to_traj_lateral_ = autoware::motion_utils::calcLateralOffset(
      trajectory_points, predicted_object.kinematics.initial_pose_with_covariance.pose.position);
  }
  return *dist_to_traj_lateral_;
}

double PlannerData::Object::get_dist_from_ego_longitudinal(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points,
  const geometry_msgs::Point & ego_position) const
{
  if (!dist_from_ego_longitudinal_) {
    dist_from_ego_longitudinal_ = autoware::motion_utils::calcSignedArcLength(
      trajectory_points, ego_position,
      predicted_object.kinematics.initial_pose_with_covariance.pose.position);
  }
  return *dist_from_ego_longitudinal_;
}

double PlannerData::Object::get_lon_vel_relative_to_traj(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const
{
  if (!lon_vel_relative_to_traj_) calc_vel_relative_to_traj(trajectory_points);
  return lon_vel_relative_to_traj_.value_or(std::numeric_limits<double>::quiet_NaN());
}

double PlannerData::Object::get_lat_vel_relative_to_traj(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const
{
  if (!lat_vel_relative_to_traj_) calc_vel_relative_to_traj(trajectory_points);
  return lat_vel_relative_to_traj_.value_or(std::numeric_limits<double>::quiet_NaN());
}

void PlannerData::Object::calc_vel_relative_to_traj(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const
{
  const auto & object_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
  const auto & object_twist = predicted_object.kinematics.initial_twist_with_covariance.twist;
  const size_t object_index = autoware::motion_utils::findNearestIndex(
    trajectory_points, object_pose.position);
  const auto & nearest = trajectory_points.at(object_index);
  const double trajectory_yaw = tf2::getYaw(nearest.pose.orientation);
  const double object_yaw = tf2::getYaw(object_pose.orientation);
  const Eigen::Rotation2Dd rotation(autoware_utils_math::normalize_radian(object_yaw - trajectory_yaw));
  const Eigen::Vector2d trajectory_direction(std::cos(trajectory_yaw), std::sin(trajectory_yaw));
  const Eigen::Vector2d trajectory_to_object(
    object_pose.position.x - nearest.pose.position.x,
    object_pose.position.y - nearest.pose.position.y);
  const int lateral_sign =
    (trajectory_direction.x() * trajectory_to_object.y() -
     trajectory_direction.y() * trajectory_to_object.x()) > 0.0 ? -1 : 1;
  const Eigen::Vector2d object_velocity(object_twist.linear.x, object_twist.linear.y);
  const Eigen::Vector2d projected = rotation * object_velocity;
  lon_vel_relative_to_traj_ = projected.x();
  lat_vel_relative_to_traj_ = lateral_sign * projected.y();
}

geometry_msgs::Pose PlannerData::Object::get_predicted_current_pose(
  const ros::Time & current_stamp, const ros::Time & predicted_objects_stamp) const
{
  if (!predicted_pose_) {
    predicted_pose_ = calc_predicted_pose(current_stamp, predicted_objects_stamp);
  }
  return *predicted_pose_;
}

geometry_msgs::Pose PlannerData::Object::calc_predicted_pose(
  const ros::Time & specified_time, const ros::Time & predicted_objects_stamp) const
{
  const auto & predicted_paths = predicted_object.kinematics.predicted_paths;
  if (predicted_paths.empty() || specified_time < predicted_objects_stamp) {
    return predicted_object.kinematics.initial_pose_with_covariance.pose;
  }
  const auto best = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.confidence < rhs.confidence; });
  const auto predicted = autoware::object_recognition_utils::calcInterpolatedPose(
    *best, (specified_time - predicted_objects_stamp).toSec());
  return predicted ? *predicted : predicted_object.kinematics.initial_pose_with_covariance.pose;
}

void PlannerData::process_predicted_objects(
  const autoware_perception_msgs::PredictedObjects & predicted_objects)
{
  predicted_objects_header = predicted_objects.header;
  objects.clear();
  objects.reserve(predicted_objects.objects.size());
  for (const auto & object : predicted_objects.objects) {
    objects.push_back(std::make_shared<Object>(object));
  }
}
}  // namespace autoware::motion_velocity_planner
