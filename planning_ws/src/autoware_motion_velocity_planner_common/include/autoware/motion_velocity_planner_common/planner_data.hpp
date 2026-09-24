#pragma once

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_perception_msgs/TrafficLightGroup.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>

#include <lanelet2_core/primitives/LineString.h>

#include <memory>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace autoware::velocity_smoother
{
class SmootherBase;
}
namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::motion_velocity_planner
{
struct StopPoint
{
  double ego_trajectory_arc_length{};
  geometry_msgs::Pose ego_stop_pose;
  lanelet::BasicLineString2d stop_line;
};

// The ROS1 message generation keeps the traffic-light payload separate from
// the array timestamp.  Keep the source contract explicit so consumers do not
// infer a fresh observation from a default-constructed signal.
struct TrafficSignalStamped
{
  ros::Time stamp;
  autoware_perception_msgs::TrafficLightGroup signal;
};

// PlannerData carries the optional inputs requested by loaded MVP plugins.  Each input
// is populated only by the owning node transport; a default Pointcloud is not used as
// a substitute for a received no-ground pointcloud.
struct PlannerData
{
  PlannerData() = default;
  explicit PlannerData(ros::NodeHandle & node);
  struct TrajectoryPolygonCollisionCheck
  {
    double decimate_trajectory_step_length{1.0};
    double goal_extended_trajectory_length{0.0};
    bool enable_to_consider_current_pose{false};
    double time_to_convergence{0.0};
  };
  struct PointcloudPreprocessParams
  {
    PointcloudPreprocessParams() = default;
    explicit PointcloudPreprocessParams(ros::NodeHandle & node);
    struct DownsampleByVoxelGrid
    {
      bool enable_downsample{false};
      double voxel_size_x{0.5};
      double voxel_size_y{0.5};
      double voxel_size_z{0.5};
    } downsample_by_voxel_grid;
    struct FilterByTrajectoryPolygon
    {
      bool enable_monolithic_crop_box{false};
      bool enable_multi_polygon_filtering{false};
      double min_trajectory_length{0.0};
      double braking_distance_scale_factor{0.0};
      double lateral_margin{0.0};
      double height_margin{0.0};
    } filter_by_trajectory_polygon;
    struct EuclideanClustering
    {
      bool enable_clustering{false};
      double cluster_tolerance{0.5};
      int min_cluster_size{1};
      int max_cluster_size{100000};
    } euclidean_clustering;
  };

  class Pointcloud
  {
  public:
    Pointcloud() = default;
    explicit Pointcloud(ros::NodeHandle & node):preprocess_params_(node) {}
    explicit Pointcloud(PointcloudPreprocessParams preprocess_params)
    : preprocess_params_(std::move(preprocess_params)) {}

    void preprocess_pointcloud(pcl::PointCloud<pcl::PointXYZ> pointcloud);
    void preprocess_pointcloud(
      pcl::PointCloud<pcl::PointXYZ> pointcloud,
      const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory,
      const nav_msgs::Odometry & current_odometry, double min_deceleration_distance,
      const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
      double ego_nearest_dist_threshold, double ego_nearest_yaw_threshold,
      double decimate_trajectory_step_length, double goal_extended_trajectory_length,
      bool enable_to_consider_current_pose, double time_to_convergence);
    void preprocess_pointcloud(
      pcl::PointCloud<pcl::PointXYZ> input,
      const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
      const nav_msgs::Odometry & odometry,double distance,
      const autoware::vehicle_info_utils::VehicleInfo & info,
      const TrajectoryPolygonCollisionCheck & collision,double nearest_distance,double nearest_yaw)
    {
      preprocess_pointcloud(std::move(input),trajectory,odometry,distance,info,
        nearest_distance,nearest_yaw,collision.decimate_trajectory_step_length,
        collision.goal_extended_trajectory_length,collision.enable_to_consider_current_pose,
        collision.time_to_convergence);
    }
    const pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_pointcloud_ptr() const;
    const std::vector<pcl::PointIndices> get_cluster_indices() const;
    bool has_processed_pointcloud() const noexcept;
    pcl::PointCloud<pcl::PointXYZ> extract_clustered_points() const;

    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    PointcloudPreprocessParams preprocess_params_;

  private:
    std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_pointcloud_ptr_;
    std::optional<std::vector<pcl::PointIndices>> cluster_indices_;
  };

  class Object
  {
  public:
    Object() = default;
    explicit Object(const autoware_perception_msgs::PredictedObject & object) : predicted_object(object) {}

    autoware_perception_msgs::PredictedObject predicted_object;

    double get_dist_to_traj_poly(
      const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys) const;
    double get_dist_to_traj_lateral(
      const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const;
    double get_dist_from_ego_longitudinal(
      const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points,
      const geometry_msgs::Point & ego_position) const;
    double get_lon_vel_relative_to_traj(
      const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const;
    double get_lat_vel_relative_to_traj(
      const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const;
    geometry_msgs::Pose get_predicted_current_pose(
      const ros::Time & current_stamp, const ros::Time & predicted_objects_stamp) const;
    geometry_msgs::Pose calc_predicted_pose(
      const ros::Time & specified_time, const ros::Time & predicted_objects_stamp) const;

  private:
    void calc_vel_relative_to_traj(
      const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const;

    mutable std::optional<double> dist_to_traj_poly_;
    mutable std::optional<double> dist_to_traj_lateral_;
    mutable std::optional<double> dist_from_ego_longitudinal_;
    mutable std::optional<double> lon_vel_relative_to_traj_;
    mutable std::optional<double> lat_vel_relative_to_traj_;
    mutable std::optional<geometry_msgs::Pose> predicted_pose_;
  };

  void process_predicted_objects(const autoware_perception_msgs::PredictedObjects & predicted_objects);

  size_t find_index(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points,
    const geometry_msgs::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      trajectory_points, pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  }
  size_t find_segment_index(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points,
    const geometry_msgs::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      trajectory_points, pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  }

  std::vector<StopPoint> calculate_map_stop_points(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory) const;
  std::optional<double> calculate_min_deceleration_distance(double target_velocity) const;
  std::optional<TrafficSignalStamped> get_traffic_signal(
    lanelet::Id id, bool keep_last_observation = false) const;

  nav_msgs::Odometry current_odometry;
  nav_msgs::OccupancyGrid occupancy_grid_map;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler;
  geometry_msgs::AccelWithCovarianceStamped current_acceleration;
  std_msgs::Header predicted_objects_header;
  std::vector<std::shared_ptr<Object>> objects;
  Pointcloud no_ground_pointcloud;
  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.57};
  TrajectoryPolygonCollisionCheck trajectory_polygon_collision_check{};
  // Raw observation and the manager-maintained last-known map are deliberately
  // separate, matching the ROS2 planner input semantics.
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_raw_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_last_observed_;
  // R024: modules that calculate jerk-constrained stopping distance consume the
  // configured source-equivalent smoother. The owning MVP node must establish it.
  std::shared_ptr<autoware::velocity_smoother::SmootherBase> velocity_smoother_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_{};
  bool is_driving_forward{true};
};
// Original public names, without changing the existing native provider layout.
using TrajectoryPolygonCollisionCheck = PlannerData::TrajectoryPolygonCollisionCheck;
using PointcloudPreprocessParams = PlannerData::PointcloudPreprocessParams;
}  // namespace autoware::motion_velocity_planner
