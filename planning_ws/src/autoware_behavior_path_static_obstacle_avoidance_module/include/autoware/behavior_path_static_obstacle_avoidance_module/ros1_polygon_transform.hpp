// Candidate-only ROS 1 compatibility for geometry_msgs::Polygon transforms.
// ROS 2 provides this conversion through its geometry transform stack, while
// ROS 1 tf2_geometry_msgs has no Polygon specialization.
#pragma once

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2
{
template <>
inline void doTransform<geometry_msgs::Polygon>(
  const geometry_msgs::Polygon & in, geometry_msgs::Polygon & out,
  const geometry_msgs::TransformStamped & transform)
{
  tf2::Transform tf;
  fromMsg(transform.transform, tf);
  out.points.resize(in.points.size());
  for (std::size_t i = 0; i < in.points.size(); ++i) {
    const auto transformed = tf * tf2::Vector3(in.points[i].x, in.points[i].y, in.points[i].z);
    out.points[i].x = static_cast<float>(transformed.x());
    out.points[i].y = static_cast<float>(transformed.y());
    out.points[i].z = static_cast<float>(transformed.z());
  }
}
}  // namespace tf2
