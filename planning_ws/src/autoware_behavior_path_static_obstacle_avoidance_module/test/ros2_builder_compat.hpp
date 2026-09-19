#pragma once

#include <autoware_perception_msgs/ObjectClassification.h>
#include <autoware_perception_msgs/Shape.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

#include <algorithm>
#include <initializer_list>

namespace geometry_msgs
{
template <class T> struct Builder;
template <> struct Builder<Pose> { Pose value{}; Builder & position(const Point & v) { value.position = v; return *this; } Builder & orientation(const Quaternion & v) { value.orientation = v; return *this; } operator Pose() const { return value; } };
template <> struct Builder<PoseWithCovariance> { PoseWithCovariance value{}; Builder & pose(const Pose & v) { value.pose = v; return *this; } Builder & covariance(std::initializer_list<double> values) { std::copy(values.begin(), values.end(), value.covariance.begin()); return *this; } operator PoseWithCovariance() const { return value; } };
template <class T> Builder<T> build() { return {}; }
}  // namespace geometry_msgs

using Odometry = nav_msgs::Odometry;

namespace autoware_perception_msgs
{
template <class T> struct Builder;
template <> struct Builder<ObjectClassification> { ObjectClassification value{}; Builder & label(uint8_t v) { value.label = v; return *this; } Builder & probability(float v) { value.probability = v; return *this; } operator ObjectClassification() const { return value; } };
template <> struct Builder<Shape> { Shape value{}; Builder & type(uint8_t v) { value.type = v; return *this; } Builder & footprint(const geometry_msgs::Polygon & v) { value.footprint = v; return *this; } Builder & dimensions(const geometry_msgs::Vector3 & v) { value.dimensions = v; return *this; } operator Shape() const { return value; } };
template <class T> Builder<T> build() { return {}; }
}  // namespace autoware_perception_msgs
