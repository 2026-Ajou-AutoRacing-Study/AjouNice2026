// Copyright 2020-2024 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS_GEOMETRY__GEOMETRY_HPP_
#define AUTOWARE_UTILS_GEOMETRY__GEOMETRY_HPP_

#include "autoware_utils_geometry/boost_geometry.hpp"
#include "autoware_utils_geometry/msg/covariance.hpp"
#include "autoware_utils_math/constants.hpp"
#include "autoware_utils_math/normalization.hpp"

#include <exception>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <tf2/utils.h>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// TODO(wep21): Remove these apis
//              after they are implemented in ros2 geometry2.
namespace tf2
{
void fromMsg(const geometry_msgs::PoseStamped & msg, tf2::Stamped<tf2::Transform> & out);
#ifdef ROS_DISTRO_GALACTIC
// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline geometry_msgs::Point & toMsg(const tf2::Vector3 & in, geometry_msgs::Point & out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline void fromMsg(const geometry_msgs::Point & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

template <>
inline void doTransform(
  const geometry_msgs::Point & t_in, geometry_msgs::Point & t_out,
  const geometry_msgs::TransformStamped & transform)
{
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in;
  fromMsg(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  toMsg(v_out, t_out);
}

template <>
inline void doTransform(
  const geometry_msgs::Pose & t_in, geometry_msgs::Pose & t_out,
  const geometry_msgs::TransformStamped & transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out);
}
#endif
}  // namespace tf2

namespace autoware_utils_geometry
{

// TODO(Takagi, Isamu): Move to each function.
using autoware_utils_math::normalize_radian;
using autoware_utils_math::pi;

template <class T>
geometry_msgs::Point get_point(const T & p)
{
  return [] (const auto & value) { geometry_msgs::Point result{}; result.x = value.x; result.y = value.y; result.z = value.z; return result; }(p);
}

template <>
inline geometry_msgs::Point get_point(const geometry_msgs::Point & p)
{
  return p;
}

template <>
inline geometry_msgs::Point get_point(const geometry_msgs::Pose & p)
{
  return p.position;
}

template <>
inline geometry_msgs::Point get_point(const geometry_msgs::PoseStamped & p)
{
  return p.pose.position;
}

template <>
inline geometry_msgs::Point get_point(const geometry_msgs::PoseWithCovarianceStamped & p)
{
  return p.pose.pose.position;
}

template <>
inline geometry_msgs::Point get_point(const autoware_planning_msgs::PathPoint & p)
{
  return p.pose.position;
}

template <>
inline geometry_msgs::Point get_point(
  const autoware_internal_planning_msgs::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}

template <>
inline geometry_msgs::Point get_point(const autoware_planning_msgs::TrajectoryPoint & p)
{
  return p.pose.position;
}

template <class T>
geometry_msgs::Pose get_pose([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of get_pose can be used.");
  throw std::logic_error("Only specializations of get_pose can be used.");
}

template <>
inline geometry_msgs::Pose get_pose(const geometry_msgs::Pose & p)
{
  return p;
}

template <>
inline geometry_msgs::Pose get_pose(const geometry_msgs::PoseStamped & p)
{
  return p.pose;
}

template <>
inline geometry_msgs::Pose get_pose(const autoware_planning_msgs::PathPoint & p)
{
  return p.pose;
}

template <>
inline geometry_msgs::Pose get_pose(
  const autoware_internal_planning_msgs::PathPointWithLaneId & p)
{
  return p.point.pose;
}

template <>
inline geometry_msgs::Pose get_pose(const autoware_planning_msgs::TrajectoryPoint & p)
{
  return p.pose;
}

template <class T>
double get_longitudinal_velocity([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getVelocity can be used.");
  throw std::logic_error("Only specializations of getVelocity can be used.");
}

template <>
inline double get_longitudinal_velocity(const autoware_planning_msgs::PathPoint & p)
{
  return p.longitudinal_velocity_mps;
}

template <>
inline double get_longitudinal_velocity(
  const autoware_internal_planning_msgs::PathPointWithLaneId & p)
{
  return p.point.longitudinal_velocity_mps;
}

template <>
inline double get_longitudinal_velocity(const autoware_planning_msgs::TrajectoryPoint & p)
{
  return p.longitudinal_velocity_mps;
}

template <class T>
void set_pose([[maybe_unused]] const geometry_msgs::Pose & pose, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of get_pose can be used.");
  throw std::logic_error("Only specializations of get_pose can be used.");
}

template <>
inline void set_pose(const geometry_msgs::Pose & pose, geometry_msgs::Pose & p)
{
  p = pose;
}

template <>
inline void set_pose(const geometry_msgs::Pose & pose, geometry_msgs::PoseStamped & p)
{
  p.pose = pose;
}

template <>
inline void set_pose(
  const geometry_msgs::Pose & pose, autoware_planning_msgs::PathPoint & p)
{
  p.pose = pose;
}

template <>
inline void set_pose(
  const geometry_msgs::Pose & pose,
  autoware_internal_planning_msgs::PathPointWithLaneId & p)
{
  p.point.pose = pose;
}

template <>
inline void set_pose(
  const geometry_msgs::Pose & pose, autoware_planning_msgs::TrajectoryPoint & p)
{
  p.pose = pose;
}

template <class T>
inline void set_orientation(const geometry_msgs::Quaternion & orientation, T & p)
{
  auto pose = get_pose(p);
  pose.orientation = orientation;
  set_pose(pose, p);
}

template <class T>
void set_longitudinal_velocity([[maybe_unused]] const float velocity, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getLongitudinalVelocity can be used.");
  throw std::logic_error("Only specializations of getLongitudinalVelocity can be used.");
}

template <>
inline void set_longitudinal_velocity(
  const float velocity, autoware_planning_msgs::TrajectoryPoint & p)
{
  p.longitudinal_velocity_mps = velocity;
}

template <>
inline void set_longitudinal_velocity(
  const float velocity, autoware_planning_msgs::PathPoint & p)
{
  p.longitudinal_velocity_mps = velocity;
}

template <>
inline void set_longitudinal_velocity(
  const float velocity, autoware_internal_planning_msgs::PathPointWithLaneId & p)
{
  p.point.longitudinal_velocity_mps = velocity;
}

inline geometry_msgs::Point create_point(const double x, const double y, const double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Vector3 get_rpy(const geometry_msgs::Quaternion & quat);

geometry_msgs::Vector3 get_rpy(const geometry_msgs::Pose & pose);

geometry_msgs::Vector3 get_rpy(const geometry_msgs::PoseStamped & pose);

geometry_msgs::Vector3 get_rpy(const geometry_msgs::PoseWithCovarianceStamped & pose);

geometry_msgs::Quaternion create_quaternion(
  const double x, const double y, const double z, const double w);

geometry_msgs::Vector3 create_translation(const double x, const double y, const double z);

// Revival of tf::create_quaternion_from_rpy
// https://answers.ros.org/question/304397/recommended-way-to-construct-quaternion-from-rollpitchyaw-with-tf2/
geometry_msgs::Quaternion create_quaternion_from_rpy(
  const double roll, const double pitch, const double yaw);

geometry_msgs::Quaternion create_quaternion_from_yaw(const double yaw);

template <class Point1, class Point2>
double calc_distance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = get_point(point1);
  const auto p2 = get_point(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class Point1, class Point2>
double calc_squared_distance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = get_point(point1);
  const auto p2 = get_point(point2);
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}

template <class Point1, class Point2>
double calc_distance3d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = get_point(point1);
  const auto p2 = get_point(point2);
  // To be replaced by std::hypot(dx, dy, dz) in C++17
  return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

/**
 * @brief calculate elevation angle of two points.
 * @details This function returns the elevation angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If the two points are in the same position, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi/2 <= elevation angle <= pi/2
 */
double calc_elevation_angle(
  const geometry_msgs::Point & p_from, const geometry_msgs::Point & p_to);

/**
 * @brief calculate azimuth angle of two points.
 * @details This function returns the azimuth angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If x and y of the two points are the same, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi < azimuth angle < pi.
 */
double calc_azimuth_angle(
  const geometry_msgs::Point & p_from, const geometry_msgs::Point & p_to);

geometry_msgs::Pose transform2pose(const geometry_msgs::Transform & transform);

geometry_msgs::PoseStamped transform2pose(
  const geometry_msgs::TransformStamped & transform);

geometry_msgs::Transform pose2transform(const geometry_msgs::Pose & pose);

geometry_msgs::TransformStamped pose2transform(
  const geometry_msgs::PoseStamped & pose, const std::string & child_frame_id);

template <class Point1, class Point2>
tf2::Vector3 point_2_tf_vector(const Point1 & src, const Point2 & dst)
{
  const auto src_p = get_point(src);
  const auto dst_p = get_point(dst);

  double dx = dst_p.x - src_p.x;
  double dy = dst_p.y - src_p.y;
  double dz = dst_p.z - src_p.z;
  return tf2::Vector3(dx, dy, dz);
}

Point3d transform_point(const Point3d & point, const geometry_msgs::Transform & transform);

Point2d transform_point(const Point2d & point, const geometry_msgs::Transform & transform);

Eigen::Vector3d transform_point(
  const Eigen::Vector3d & point, const geometry_msgs::Pose & pose);

geometry_msgs::Point transform_point(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & pose);

geometry_msgs::Point32 transform_point(
  const geometry_msgs::Point32 & point32, const geometry_msgs::Pose & pose);

template <class T>
T transform_vector(const T & points, const geometry_msgs::Transform & transform)
{
  T transformed;
  for (const auto & point : points) {
    transformed.push_back(transform_point(point, transform));
  }
  return transformed;
}

geometry_msgs::Pose transform_pose(
  const geometry_msgs::Pose & pose, const geometry_msgs::TransformStamped & transform);

geometry_msgs::Pose transform_pose(
  const geometry_msgs::Pose & pose, const geometry_msgs::Transform & transform);

geometry_msgs::Pose transform_pose(
  const geometry_msgs::Pose & pose, const geometry_msgs::Pose & pose_transform);

// Transform pose in world coordinates to local coordinates
/*
geometry_msgs::Pose inverse_transform_pose(
  const geometry_msgs::Pose & pose, const geometry_msgs::TransformStamped & transform);
*/

// Transform pose in world coordinates to local coordinates
geometry_msgs::Pose inverse_transform_pose(
  const geometry_msgs::Pose & pose, const geometry_msgs::Transform & transform);

// Transform pose in world coordinates to local coordinates
geometry_msgs::Pose inverse_transform_pose(
  const geometry_msgs::Pose & pose, const geometry_msgs::Pose & transform_pose);

// Transform point in world coordinates to local coordinates
Eigen::Vector3d inverse_transform_point(
  const Eigen::Vector3d & point, const geometry_msgs::Pose & pose);

// Transform point in world coordinates to local coordinates
geometry_msgs::Point inverse_transform_point(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & pose);

double calc_curvature(
  const geometry_msgs::Point & p1, const geometry_msgs::Point & p2,
  const geometry_msgs::Point & p3);

template <class Pose1, class Pose2>
bool is_driving_forward(const Pose1 & src_pose, const Pose2 & dst_pose)
{
  // check the first point direction
  const double src_yaw = tf2::getYaw(get_pose(src_pose).orientation);
  const double pose_direction_yaw = calc_azimuth_angle(get_point(src_pose), get_point(dst_pose));
  return std::fabs(normalize_radian(src_yaw - pose_direction_yaw)) < pi / 2.0;
}

/**
 * @brief Calculate offset pose. The offset values are defined in the local coordinate of the input
 * pose.
 */
geometry_msgs::Pose calc_offset_pose(
  const geometry_msgs::Pose & p, const double x, const double y, const double z,
  const double yaw = 0.0);

/**
 * @brief Calculate a point by linear interpolation.
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @return interpolated point
 */
template <class Point1, class Point2>
geometry_msgs::Point calc_interpolated_point(
  const Point1 & src, const Point2 & dst, const double ratio)
{
  const auto src_point = get_point(src);
  const auto dst_point = get_point(dst);

  tf2::Vector3 src_vec;
  src_vec.setX(src_point.x);
  src_vec.setY(src_point.y);
  src_vec.setZ(src_point.z);

  tf2::Vector3 dst_vec;
  dst_vec.setX(dst_point.x);
  dst_vec.setY(dst_point.y);
  dst_vec.setZ(dst_point.z);

  // Get pose by linear interpolation
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const auto & vec = tf2::lerp(src_vec, dst_vec, clamped_ratio);

  geometry_msgs::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();

  return point;
}

/**
 * @brief Calculate a pose by linear interpolation.
 * Note that if dist(src_pose, dst_pose)<=0.01
 * the orientation of the output pose is same as the orientation
 * of the dst_pose
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @param set_orientation_from_position_direction set position by spherical interpolation if false
 * @return interpolated point
 */
template <class Pose1, class Pose2>
geometry_msgs::Pose calc_interpolated_pose(
  const Pose1 & src_pose, const Pose2 & dst_pose, const double ratio,
  const bool set_orientation_from_position_direction = true)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  geometry_msgs::Pose output_pose;
  output_pose.position =
    calc_interpolated_point(get_point(src_pose), get_point(dst_pose), clamped_ratio);

  if (set_orientation_from_position_direction) {
    const double input_poses_dist = calc_distance2d(get_point(src_pose), get_point(dst_pose));
    const bool is_driving_forward_flag = is_driving_forward(src_pose, dst_pose);

    // Get orientation from interpolated point and src_pose
    if ((is_driving_forward_flag && clamped_ratio > 1.0 - (1e-6)) || input_poses_dist < 1e-3) {
      output_pose.orientation = get_pose(dst_pose).orientation;
    } else if (!is_driving_forward_flag && clamped_ratio < 1e-6) {
      output_pose.orientation = get_pose(src_pose).orientation;
    } else {
      const auto & base_pose = is_driving_forward_flag ? dst_pose : src_pose;
      const double pitch = calc_elevation_angle(get_point(output_pose), get_point(base_pose));
      const double yaw = calc_azimuth_angle(get_point(output_pose), get_point(base_pose));
      output_pose.orientation = create_quaternion_from_rpy(0.0, pitch, yaw);
    }
  } else {
    // Get orientation by spherical linear interpolation
    tf2::Transform src_tf;
    tf2::Transform dst_tf;
    tf2::fromMsg(get_pose(src_pose), src_tf);
    tf2::fromMsg(get_pose(dst_pose), dst_tf);
    const auto & quaternion = tf2::slerp(src_tf.getRotation(), dst_tf.getRotation(), clamped_ratio);
    output_pose.orientation = tf2::toMsg(quaternion);
  }

  return output_pose;
}

inline geometry_msgs::Vector3 create_vector3(const double x, double y, double z)
{
  return [] (const double vx, const double vy, const double vz) { geometry_msgs::Vector3 result{}; result.x = vx; result.y = vy; result.z = vz; return result; }(x, y, z);
}

inline geometry_msgs::Twist create_twist(
  const geometry_msgs::Vector3 & velocity, geometry_msgs::Vector3 & angular)
{
  return [] (const geometry_msgs::Vector3 & linear, const geometry_msgs::Vector3 & angular_value) { geometry_msgs::Twist result{}; result.linear = linear; result.angular = angular_value; return result; }(velocity, angular);
}

inline double calc_norm(const geometry_msgs::Vector3 & v)
{
  return std::hypot(v.x, v.y, v.z);
}

/**
 * @brief Judge whether twist covariance is valid.
 *
 * @param twist_with_covariance source twist with covariance
 * @return If all element of covariance is 0, return false.
 */
//
bool is_twist_covariance_valid(
  const geometry_msgs::TwistWithCovariance & twist_with_covariance);

// NOTE: much faster than boost::geometry::intersects()
std::optional<geometry_msgs::Point> intersect(
  const geometry_msgs::Point & p1, const geometry_msgs::Point & p2,
  const geometry_msgs::Point & p3, const geometry_msgs::Point & p4);

/**
 * @brief Check if 2 convex polygons intersect using the GJK algorithm
 * @details much faster than boost::geometry::intersects()
 */
bool intersects_convex(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2);

}  // namespace autoware_utils_geometry

#endif  // AUTOWARE_UTILS_GEOMETRY__GEOMETRY_HPP_
