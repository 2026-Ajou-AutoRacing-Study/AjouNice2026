#pragma once

#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <optional>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
// Original common helper: preserve concatenation order and move ownership.
template<class T>
std::vector<T> concat_vectors(std::vector<T> first_vector,std::vector<T> second_vector)
{
  first_vector.insert(first_vector.end(),std::make_move_iterator(second_vector.begin()),
    std::make_move_iterator(second_vector.end()));
  return first_vector;
}
geometry_msgs::Point to_geometry_point(const pcl::PointXYZ & point);
geometry_msgs::Point to_geometry_point(const autoware_utils_geometry::Point2d & point);
double calc_object_possible_max_dist_from_center(const autoware_perception_msgs::Shape & shape);
visualization_msgs::Marker get_object_marker(const geometry_msgs::Pose & pose,size_t index,const std::string & ns,double r,double g,double b);
double calc_possible_min_dist_from_obj_to_traj_poly(const std::shared_ptr<PlannerData::Object> object,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,const autoware::vehicle_info_utils::VehicleInfo & info);
template <typename T, typename U>
std::optional<T> get_obstacle_from_uuid(const std::vector<T> & obstacles, const U & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}
std::vector<autoware_planning_msgs::TrajectoryPoint> get_extended_trajectory_points(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & input_points,
  double extend_distance, double step_length);
std::vector<autoware_planning_msgs::TrajectoryPoint> decimate_trajectory_points_from_ego(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points,
  const geometry_msgs::Pose & current_pose, double ego_nearest_dist_threshold,
  double ego_nearest_yaw_threshold, double decimate_trajectory_step_length,
  double goal_extended_trajectory_length);
void insertStop(std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory, const geometry_msgs::Point & stop_point);
void insertSlowdown(std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory, const SlowdownInterval & slowdown_interval);
void applyPlanningResult(std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory, const VelocityPlanningResult & planning_result);

// Exact upstream point-selection rule used by the obstacle-cruise PID planner.
template <class T>
size_t get_index_with_longitudinal_offset(
  const T & points, const double longitudinal_offset, std::optional<size_t> start_idx)
{
  if (points.empty()) throw std::logic_error("points is empty.");
  if (start_idx) {
    if (points.size() <= *start_idx) throw std::out_of_range("start_idx is out of range.");
  } else {
    start_idx = longitudinal_offset > 0.0 ? 0U : points.size() - 1U;
  }
  double sum_length = 0.0;
  if (longitudinal_offset > 0.0) {
    for (size_t i = *start_idx; i < points.size() - 1U; ++i) {
      const double segment_length = autoware_utils_geometry::calc_distance2d(points.at(i), points.at(i + 1U));
      sum_length += segment_length;
      if (sum_length >= longitudinal_offset) {
        const double back_length = sum_length - longitudinal_offset;
        const double front_length = segment_length - back_length;
        return front_length < back_length ? i : i + 1U;
      }
    }
    return points.size() - 1U;
  }
  for (size_t i = *start_idx; 0U < i; --i) {
    const double segment_length = autoware_utils_geometry::calc_distance2d(points.at(i - 1U), points.at(i));
    sum_length += segment_length;
    if (sum_length >= -longitudinal_offset) {
      const double back_length = sum_length + longitudinal_offset;
      const double front_length = segment_length - back_length;
      return front_length < back_length ? i : i - 1U;
    }
  }
  return 0U;
}
}  // namespace autoware::motion_velocity_planner::utils
