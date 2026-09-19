#ifndef AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_PARAMETERS_ROS1_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_PARAMETERS_ROS1_HPP_

#include <ros/ros.h>

#include <string>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
// Candidate replacement for the generated ROS2 parameter-library type.  It
// deliberately has no behavior defaults: all 35 leaves must come from the
// rendered ROS1 parameter tree before the checker can become ready.
struct Params
{
  struct RightTurn { bool enable; bool check_traffic_signal; bool check_crossing_lanes; bool check_turning_lanes; double crossing_lane_angle_th; } right_turn;
  struct LeftTurn { bool enable; bool check_traffic_signal; bool check_turning_lanes; } left_turn;
  struct Voxel { double x; double y; double z; int min_size; } voxel_grid_filter;
  struct Clustering { double tolerance; double min_height; int min_size; int max_size; } clustering;
  struct Velocity { double max_acceleration; double max_velocity; double observation_time; double max_history_time; int buffer_size; } velocity_estimation;
  struct PointCloud { double height_buffer; double min_height; Voxel voxel_grid_filter; Clustering clustering; Velocity velocity_estimation; double latency; } pointcloud;
  struct Filter { double min_velocity; double moving_time; } filter;
  bool enable;
  double detection_range;
  double ttc_threshold;
  double ego_deceleration;
  double min_time_horizon;
  double on_time_buffer;
  double off_time_buffer;
  double close_distance_th;
  double close_time_th;
};

template <class T>
inline bool require(ros::NodeHandle & nh, const std::string & key, T & value, std::string & error)
{
  if (nh.getParam(key, value)) return true;
  error = "missing required ICC parameter: " + key;
  return false;
}

inline bool load(ros::NodeHandle & nh, Params & p, std::string & error)
{
  return require(nh, "icc_parameters/enable", p.enable, error) &&
    require(nh, "icc_parameters/detection_range", p.detection_range, error) &&
    require(nh, "icc_parameters/ttc_threshold", p.ttc_threshold, error) &&
    require(nh, "icc_parameters/ego_deceleration", p.ego_deceleration, error) &&
    require(nh, "icc_parameters/min_time_horizon", p.min_time_horizon, error) &&
    require(nh, "icc_parameters/on_time_buffer", p.on_time_buffer, error) &&
    require(nh, "icc_parameters/off_time_buffer", p.off_time_buffer, error) &&
    require(nh, "icc_parameters/close_distance_th", p.close_distance_th, error) &&
    require(nh, "icc_parameters/close_time_th", p.close_time_th, error) &&
    require(nh, "icc_parameters/right_turn/enable", p.right_turn.enable, error) &&
    require(nh, "icc_parameters/right_turn/check_traffic_signal", p.right_turn.check_traffic_signal, error) &&
    require(nh, "icc_parameters/right_turn/check_crossing_lanes", p.right_turn.check_crossing_lanes, error) &&
    require(nh, "icc_parameters/right_turn/check_turning_lanes", p.right_turn.check_turning_lanes, error) &&
    require(nh, "icc_parameters/right_turn/crossing_lane_angle_th", p.right_turn.crossing_lane_angle_th, error) &&
    require(nh, "icc_parameters/left_turn/enable", p.left_turn.enable, error) &&
    require(nh, "icc_parameters/left_turn/check_traffic_signal", p.left_turn.check_traffic_signal, error) &&
    require(nh, "icc_parameters/left_turn/check_turning_lanes", p.left_turn.check_turning_lanes, error) &&
    require(nh, "icc_parameters/pointcloud/height_buffer", p.pointcloud.height_buffer, error) &&
    require(nh, "icc_parameters/pointcloud/min_height", p.pointcloud.min_height, error) &&
    require(nh, "icc_parameters/pointcloud/voxel_grid_filter/x", p.pointcloud.voxel_grid_filter.x, error) &&
    require(nh, "icc_parameters/pointcloud/voxel_grid_filter/y", p.pointcloud.voxel_grid_filter.y, error) &&
    require(nh, "icc_parameters/pointcloud/voxel_grid_filter/z", p.pointcloud.voxel_grid_filter.z, error) &&
    require(nh, "icc_parameters/pointcloud/voxel_grid_filter/min_size", p.pointcloud.voxel_grid_filter.min_size, error) &&
    require(nh, "icc_parameters/pointcloud/clustering/tolerance", p.pointcloud.clustering.tolerance, error) &&
    require(nh, "icc_parameters/pointcloud/clustering/min_height", p.pointcloud.clustering.min_height, error) &&
    require(nh, "icc_parameters/pointcloud/clustering/min_size", p.pointcloud.clustering.min_size, error) &&
    require(nh, "icc_parameters/pointcloud/clustering/max_size", p.pointcloud.clustering.max_size, error) &&
    require(nh, "icc_parameters/pointcloud/velocity_estimation/max_acceleration", p.pointcloud.velocity_estimation.max_acceleration, error) &&
    require(nh, "icc_parameters/pointcloud/velocity_estimation/max_velocity", p.pointcloud.velocity_estimation.max_velocity, error) &&
    require(nh, "icc_parameters/pointcloud/velocity_estimation/observation_time", p.pointcloud.velocity_estimation.observation_time, error) &&
    require(nh, "icc_parameters/pointcloud/velocity_estimation/max_history_time", p.pointcloud.velocity_estimation.max_history_time, error) &&
    require(nh, "icc_parameters/pointcloud/velocity_estimation/buffer_size", p.pointcloud.velocity_estimation.buffer_size, error) &&
    require(nh, "icc_parameters/pointcloud/latency", p.pointcloud.latency, error) &&
    require(nh, "icc_parameters/filter/min_velocity", p.filter.min_velocity, error) &&
    require(nh, "icc_parameters/filter/moving_time", p.filter.moving_time, error);
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1

#endif
