#pragma once

#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <autoware_perception_msgs/PredictedPath.h>
#include <autoware_perception_msgs/Shape.h>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <geometry_msgs/Pose.h>
#include <ros/time.h>

#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::polygon_utils
{
struct PointWithStamp
{
  ros::Time stamp;
  geometry_msgs::Point point;
};

// P3-COLLISION-001: predicted-path collision surface required by Cruise and
// other MVP safety modules. This revision is candidate-only.
std::vector<PointWithStamp> getCollisionPoints(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const std::vector<autoware_utils_geometry::Polygon2d> & trajectory_polygons,
  const ros::Time & obstacle_stamp, const autoware_perception_msgs::PredictedPath & predicted_path,
  const autoware_perception_msgs::Shape & shape, const ros::Time & current_time,
  bool is_driving_forward, std::vector<size_t> & collision_index,
  double max_dist = std::numeric_limits<double>::max(),
  double max_prediction_time_for_collision_check = std::numeric_limits<double>::max());

std::optional<std::pair<geometry_msgs::Point, double>> getCollisionPoint(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const std::vector<autoware_utils_geometry::Polygon2d> & trajectory_polygons,
  const geometry_msgs::Point & object_position,
  const autoware_utils_geometry::Polygon2d & object_polygon, double x_offset_to_bumper);

std::vector<double> calcFrontOuterWheelOffTracking(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

std::vector<geometry_msgs::Pose> calculateErrorPoses(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const geometry_msgs::Pose & current_ego_pose, double time_to_convergence);

std::vector<autoware_utils_geometry::Polygon2d> createOneStepPolygons(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::Pose & current_ego_pose, double lateral_margin,
  bool enable_to_consider_current_pose, double time_to_convergence,
  double additional_front_outer_wheel_off_track_scale = 0.0);
// Original eight-argument contract: decimation is explicitly unused upstream;
// it is not the off-tracking scale argument of the existing native API.
inline std::vector<autoware_utils_geometry::Polygon2d> create_one_step_polygons(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const autoware::vehicle_info_utils::VehicleInfo & info,
  const geometry_msgs::Pose & pose,double margin,bool consider_pose,double convergence,
  [[maybe_unused]] double decimation,double off_track_scale=0.0)
{
  return createOneStepPolygons(points,info,pose,margin,consider_pose,convergence,off_track_scale);
}
}
