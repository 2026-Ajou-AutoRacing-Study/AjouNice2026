#pragma once
#include <autoware/motion_velocity_planner_common/utils.hpp>

#include "type_alias.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
geometry_msgs::Point to_geometry_point(const pcl::PointXYZ & point);
geometry_msgs::Point to_geometry_point(const autoware_utils_geometry::Point2d & point);
std::optional<double> calc_distance_to_front_object(
  const std::vector<TrajectoryPoint> & traj_points, size_t ego_idx,
  const geometry_msgs::Point & obstacle_pos);
std::vector<uint8_t> get_target_object_type(ros::NodeHandle & node, const std::string & param_prefix);





visualization_msgs::Marker get_object_marker(
  const geometry_msgs::Pose & pose, size_t index, const std::string & ns, double r, double g,
  double b);
double calc_possible_min_dist_from_obj_to_traj_poly(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info);
double get_dist_to_traj_poly(
  const geometry_msgs::Point & point, const std::vector<Polygon2d> & trajectory_polygons);
double calc_dist_to_traj_poly(
  const PredictedObject & object, const std::vector<Polygon2d> & trajectory_polygons);
}  // namespace autoware::motion_velocity_planner::utils
