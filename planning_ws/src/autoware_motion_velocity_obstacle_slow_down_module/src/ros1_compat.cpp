#include "ros1_compat.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <boost/geometry.hpp>

#include <unordered_map>

namespace autoware::motion_velocity_planner::utils
{




std::optional<double> calc_distance_to_front_object(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::Point & obstacle_pos)
{
  const auto obstacle_idx = autoware::motion_utils::findNearestIndex(traj_points, obstacle_pos);
  const auto distance = autoware::motion_utils::calcSignedArcLength(traj_points, ego_idx, obstacle_idx);
  return distance < 0.0 ? std::nullopt : std::optional<double>(distance);
}

std::vector<uint8_t> get_target_object_type(ros::NodeHandle & node, const std::string & param_prefix)
{
  const std::unordered_map<std::string, uint8_t> labels{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK}, {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};
  std::vector<uint8_t> enabled;
  for (const auto & label : labels) {
    bool selected = false;
    node.param(autoware_utils_rclcpp::to_ros1_parameter_name(param_prefix + label.first), selected, false);
    if (selected) enabled.push_back(label.second);
  }
  return enabled;
}





double get_dist_to_traj_poly(
  const geometry_msgs::Point & point, const std::vector<Polygon2d> & trajectory_polygons)
{
  const autoware_utils_geometry::Point2d point_2d(point.x, point.y);
  double distance = std::numeric_limits<double>::infinity();
  for (const auto & polygon : trajectory_polygons) distance = std::min(distance, boost::geometry::distance(polygon, point_2d));
  return distance;
}

double calc_dist_to_traj_poly(
  const PredictedObject & object, const std::vector<Polygon2d> & trajectory_polygons)
{
  const auto object_polygon = autoware_utils::to_polygon2d(
    object.kinematics.initial_pose_with_covariance.pose, object.shape);
  double distance = std::numeric_limits<double>::infinity();
  for (const auto & polygon : trajectory_polygons) distance = std::min(distance, boost::geometry::distance(polygon, object_polygon));
  return distance;
}
}  // namespace autoware::motion_velocity_planner::utils
