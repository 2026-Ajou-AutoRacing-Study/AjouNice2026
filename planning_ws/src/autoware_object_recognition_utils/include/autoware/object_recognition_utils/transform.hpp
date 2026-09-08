// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__OBJECT_RECOGNITION_UTILS__TRANSFORM_HPP_
#define AUTOWARE__OBJECT_RECOGNITION_UTILS__TRANSFORM_HPP_

#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/optional.hpp>
#include <ros/ros.h>

#include <string>

namespace detail
{
[[maybe_unused]] inline boost::optional<geometry_msgs::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const ros::Time & time)
{
  try {
    geometry_msgs::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, ros::Duration(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    ROS_WARN_STREAM(ex.what());
    return boost::none;
  }
}

[[maybe_unused]] inline boost::optional<Eigen::Matrix4f> getTransformMatrix(
  const std::string & in_target_frame, const std_msgs::Header & in_cloud_header,
  const tf2_ros::Buffer & tf_buffer)
{
  try {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
      in_target_frame, in_cloud_header.frame_id, in_cloud_header.stamp,
      ros::Duration(1.0));
    Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    return mat;
  } catch (tf2::TransformException & e) {
    ROS_WARN_STREAM(e.what());
    return boost::none;
  }
}
}  // namespace detail

namespace autoware::object_recognition_utils
{
template <class T>
bool transformObjects(
  const T & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  T & output_msg)
{
  output_msg = input_msg;

  // transform to world coordinate
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    {
      const auto ros_target2objects_world = detail::getTransform(
        tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      tf2::Transform tf_objects_world2objects;
      auto & pose_with_cov = object.kinematics.pose_with_covariance;
      tf2::fromMsg(pose_with_cov.pose, tf_objects_world2objects);
      tf2::Transform tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      // transform pose, frame difference and object pose
      tf2::toMsg(tf_target2objects, pose_with_cov.pose);
      // transform covariance, only the frame difference
      pose_with_cov.covariance =
        tf2::transformCovariance(pose_with_cov.covariance, tf_target2objects_world);
    }
  }
  return true;
}
template <class T>
bool transformObjectsWithFeature(
  const T & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  T & output_msg)
{
  output_msg = input_msg;
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world = tf2::Transform::getIdentity();
    const auto ros_target2objects_world = detail::getTransform(
      tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
    if (!ros_target2objects_world) {
      return false;
    }
    tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    const auto tf_matrix = detail::getTransformMatrix(target_frame_id, input_msg.header, tf_buffer);
    if (!tf_matrix) {
      ROS_WARN("failed to get transformed matrix");
      return false;
    }
    for (auto & feature_object : output_msg.feature_objects) {
      tf2::Transform tf_objects_world2objects;
      // transform object
      tf2::fromMsg(
        feature_object.object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf2::Transform tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, feature_object.object.kinematics.pose_with_covariance.pose);

      // transform cluster
      sensor_msgs::PointCloud2 transformed_cluster;
      pcl_ros::transformPointCloud(*tf_matrix, feature_object.feature.cluster, transformed_cluster);
      transformed_cluster.header.frame_id = target_frame_id;
      feature_object.feature.cluster = transformed_cluster;
    }
    output_msg.header.frame_id = target_frame_id;
    return true;
  }
  return true;
}
}  // namespace autoware::object_recognition_utils

#endif  // AUTOWARE__OBJECT_RECOGNITION_UTILS__TRANSFORM_HPP_
