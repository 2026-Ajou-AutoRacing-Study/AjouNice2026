// Copyright 2021 Apex.AI, Inc.
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
/// \brief This file defines some helper functions used by ObjectPolygonDisplayBase class
#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_

#include "autoware_perception_rviz_plugin/visibility_control.hpp"

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <ros/ros.h>

#include <autoware_perception_msgs/DetectedObject.h>
#include <autoware_perception_msgs/ObjectClassification.h>
#include <autoware_perception_msgs/PredictedPath.h>
#include <autoware_perception_msgs/Shape.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <boost/make_shared.hpp>
#include <optional>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
namespace detail
{
// Struct to define all the configurable visual properties of an object of a particular
// classification type
struct ObjectPropertyValues
{
  // Classified type of the object
  std::string label;
  // Color for the type of the object
  std::array<int, 3> color;
  // Alpha values for the type of the object
  float alpha{0.999F};
};

// Control object marker visualization
enum class ObjectFillType { Skeleton, Fill };

// Map defining colors according to value of label field in ObjectClassification msg
const std::map<
  autoware_perception_msgs::ObjectClassification::_label_type, ObjectPropertyValues>
  // Color map is based on cityscapes color
  kDefaultObjectPropertyValues = {
    {autoware_perception_msgs::ObjectClassification::UNKNOWN, {"UNKNOWN", {255, 255, 255}}},
    {autoware_perception_msgs::ObjectClassification::CAR, {"CAR", {30, 144, 255}}},
    {autoware_perception_msgs::ObjectClassification::BUS, {"BUS", {30, 144, 255}}},
    {autoware_perception_msgs::ObjectClassification::PEDESTRIAN,
     {"PEDESTRIAN", {255, 192, 203}}},
    {autoware_perception_msgs::ObjectClassification::BICYCLE, {"CYCLIST", {119, 11, 32}}},
    {autoware_perception_msgs::ObjectClassification::MOTORCYCLE,
     {"MOTORCYCLE", {119, 11, 32}}},
    {autoware_perception_msgs::ObjectClassification::TRAILER, {"TRAILER", {30, 144, 255}}},
    {autoware_perception_msgs::ObjectClassification::TRUCK, {"TRUCK", {30, 144, 255}}}};

/// \brief Convert the given polygon into a marker representing the shape in 3d
/// \param shape_msg Shape msg to be converted. Corners should be in object-local frame
/// \param centroid Centroid position of the shape in Object.header.frame_id frame
/// \param orientation Orientation of the shape in Object.header.frame_id frame
/// \param color_rgba Color and alpha values to use for the marker
/// \param line_width Line thickness around the object
/// \return Marker ptr. Id and header will have to be set by the caller
AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_shape_marker_ptr(
  const autoware_perception_msgs::Shape & shape_msg,
  const geometry_msgs::Point & centroid, const geometry_msgs::Quaternion & orientation,
  const std_msgs::ColorRGBA & color_rgba, const double & line_width,
  const bool & is_orientation_available = true,
  const ObjectFillType fill_type = ObjectFillType::Skeleton);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_2d_shape_marker_ptr(
  const autoware_perception_msgs::Shape & shape_msg,
  const geometry_msgs::Point & centroid, const geometry_msgs::Quaternion & orientation,
  const std_msgs::ColorRGBA & color_rgba, const double & line_width,
  const bool & is_orientation_available = true);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_mesh_marker_ptr(
  const autoware_perception_msgs::Shape & shape_msg,
  const geometry_msgs::Point & centroid, const geometry_msgs::Quaternion & orientation,
  const std::vector<autoware_perception_msgs::ObjectClassification> & classification);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::MarkerArray::Ptr
get_indicator_marker_ptr(
  const autoware_perception_msgs::Shape & shape_msg,
  const geometry_msgs::Point & centroid, const geometry_msgs::Quaternion & orientation,
  const std::vector<autoware_perception_msgs::ObjectClassification> & classification);

/// \brief Convert the given polygon into a marker representing the shape in 3d
/// \param centroid Centroid position of the shape in Object.header.frame_id frame
/// \return Marker ptr. Id and header will have to be set by the caller
AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_label_marker_ptr(
  const geometry_msgs::Point & centroid, const geometry_msgs::Quaternion & orientation,
  const std::string & label, const std_msgs::ColorRGBA & color_rgba);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_existence_probability_marker_ptr(
  const geometry_msgs::Point & centroid, const geometry_msgs::Quaternion & orientation,
  const float existence_probability, const std_msgs::ColorRGBA & color_rgba);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_uuid_marker_ptr(
  const std::string & uuid, const geometry_msgs::Point & centroid,
  const std_msgs::ColorRGBA & color_rgba);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_pose_covariance_marker_ptr(
  const geometry_msgs::PoseWithCovariance & pose_with_covariance,
  const double & confidence_interval_coefficient);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_yaw_covariance_marker_ptr(
  const geometry_msgs::PoseWithCovariance & pose_with_covariance, const double & length,
  const double & confidence_interval_coefficient, const double & line_width);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_velocity_text_marker_ptr(
  const geometry_msgs::Twist & twist, const geometry_msgs::Point & vis_pos,
  const std_msgs::ColorRGBA & color_rgba);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_acceleration_text_marker_ptr(
  const geometry_msgs::Accel & accel, const geometry_msgs::Point & vis_pos,
  const std_msgs::ColorRGBA & color_rgba);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_twist_marker_ptr(
  const geometry_msgs::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::TwistWithCovariance & twist_with_covariance, const double & line_width);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_twist_covariance_marker_ptr(
  const geometry_msgs::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::TwistWithCovariance & twist_with_covariance,
  const double & confidence_interval_coefficient);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_yaw_rate_marker_ptr(
  const geometry_msgs::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::TwistWithCovariance & twist_with_covariance, const double & line_width);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_yaw_rate_covariance_marker_ptr(
  const geometry_msgs::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::TwistWithCovariance & twist_with_covariance,
  const double & confidence_interval_coefficient, const double & line_width);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_predicted_path_marker_ptr(
  const autoware_perception_msgs::Shape & shape,
  const autoware_perception_msgs::PredictedPath & predicted_path,
  const std_msgs::ColorRGBA & predicted_path_color, const bool is_simple = false);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::Marker::Ptr
get_path_confidence_marker_ptr(
  const autoware_perception_msgs::PredictedPath & predicted_path,
  const std_msgs::ColorRGBA & path_confidence_color);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_arc_line_strip(
  const double start_angle, const double end_angle, const double radius,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_line_list_from_points(
  const double point_list[][3], const int point_pairs[][2], const int & num_pairs,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_covariance_eigen_vectors(
  const Eigen::Matrix2d & matrix, double & sigma1, double & sigma2, double & yaw);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_bounding_box_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_bounding_box_direction_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_bounding_box_orientation_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_bounding_box_bottom_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_bounding_box_bottom_direction_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_bounding_box_bottom_orientation_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_cylinder_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_cylinder_bottom_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_circle_line_list(
  const geometry_msgs::Point center, const double radius,
  std::vector<geometry_msgs::Point> & points, const int n);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_polygon_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_polygon_bottom_line_list(
  const autoware_perception_msgs::Shape & shape,
  std::vector<geometry_msgs::Point> & points);

AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_path_line_list(
  const autoware_perception_msgs::PredictedPath & paths,
  std::vector<geometry_msgs::Point> & points, const bool is_simple = false);

/// \brief Convert Point32 to Point
/// \param val Point32 to be converted
/// \return Point type
inline AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC geometry_msgs::Point to_point(
  const geometry_msgs::Point32 & val)
{
  geometry_msgs::Point ret;
  ret.x = static_cast<double>(val.x);
  ret.y = static_cast<double>(val.y);
  ret.z = static_cast<double>(val.z);
  return ret;
}

/// \brief Convert to Pose from Point and Quaternion
/// \param point
/// \param orientation
/// \return Pose type
inline AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC geometry_msgs::Pose to_pose(
  const geometry_msgs::Point & point, const geometry_msgs::Quaternion & orientation)
{
  geometry_msgs::Pose ret;
  ret.position = point;
  ret.orientation = orientation;
  return ret;
}

inline AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC geometry_msgs::Pose initPose()
{
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

/// \brief Get the best classification from the list of classifications based on max probability
/// \tparam ClassificationContainerT List type with ObjectClassificationMsg
/// \param labels List of ObjectClassificationMsg objects
/// \param logger_name Name to use for logger in case of a warning (if labels is empty)
/// \return Id of the best classification, Unknown if there is no best label
template <typename ClassificationContainerT>
AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC
  autoware_perception_msgs::ObjectClassification::_label_type
  get_best_label(ClassificationContainerT labels, const std::string & logger_name)
{
  const auto best_class_label =
    std::max_element(labels.begin(), labels.end(), [](const auto & a, const auto & b) -> bool {
      return a.label >= 100 || (a.probability < b.probability && b.label < 100);
    });  // label > 100 is reserved for status labels
  if (best_class_label == labels.end()) {
    ROS_WARN_NAMED(
      logger_name.c_str(),
      "Empty classification field. "
      "Treating as unknown");
    return autoware_perception_msgs::ObjectClassification::UNKNOWN;
  }
  return best_class_label->label;
}

}  // namespace detail
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_
