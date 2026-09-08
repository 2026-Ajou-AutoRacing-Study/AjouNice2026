// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pcl/point_types.h>
namespace autoware::motion_velocity_planner::utils {
using autoware_perception_msgs::Shape;
using autoware_planning_msgs::TrajectoryPoint;
using autoware::vehicle_info_utils::VehicleInfo;
geometry_msgs::Point to_geometry_point(const pcl::PointXYZ & point)
{
  geometry_msgs::Point geom_point;
  geom_point.x = point.x;
  geom_point.y = point.y;
  geom_point.z = point.z;
  return geom_point;
}

geometry_msgs::Point to_geometry_point(const autoware_utils_geometry::Point2d & point)
{
  geometry_msgs::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}

double calc_object_possible_max_dist_from_center(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in motion_velocity_planner_common.");
}
visualization_msgs::Marker get_object_marker(
  const geometry_msgs::Pose & obj_pose, size_t idx, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = ros::Time::now();

  auto marker = autoware_utils_visualization::create_default_marker(
    "map", current_time, ns, static_cast<int32_t>(idx), visualization_msgs::Marker::SPHERE,
    autoware_utils_visualization::create_marker_scale(2.0, 2.0, 2.0),
    autoware_utils_visualization::create_marker_color(
      static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.8));

  marker.pose = obj_pose;

  return marker;
}

double calc_possible_min_dist_from_obj_to_traj_poly(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info)
{
  const double object_possible_max_dist =
    calc_object_possible_max_dist_from_center(object->predicted_object.shape);
  // The minimum lateral distance to the trajectory polygon is estimated by assuming that the
  // ego-vehicle's front right or left corner is the furthest from the trajectory, in the very worst
  // case
  const double ego_possible_max_dist =
    std::hypot(vehicle_info.max_longitudinal_offset_m, vehicle_info.vehicle_width_m / 2.0);
  const double possible_min_dist_to_traj_poly =
    std::abs(object->get_dist_to_traj_lateral(traj_points)) - ego_possible_max_dist -
    object_possible_max_dist;
  return possible_min_dist_to_traj_poly;
}

}
