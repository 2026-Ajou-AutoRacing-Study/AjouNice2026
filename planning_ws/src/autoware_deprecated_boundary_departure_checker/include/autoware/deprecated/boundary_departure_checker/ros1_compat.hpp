// ROS1 names for the frozen ROS2 message and duration surface used by this package.
#ifndef AUTOWARE__DEPRECATED__BOUNDARY_DEPARTURE_CHECKER__ROS1_COMPAT_HPP_
#define AUTOWARE__DEPRECATED__BOUNDARY_DEPARTURE_CHECKER__ROS1_COMPAT_HPP_

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <autoware_vehicle_msgs/SteeringReport.h>
#include <builtin_interfaces/Duration.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>

namespace autoware_internal_planning_msgs::msg
{
using PathWithLaneId = ::autoware_internal_planning_msgs::PathWithLaneId;
}
namespace autoware_planning_msgs::msg
{
using LaneletRoute = ::autoware_planning_msgs::LaneletRoute;
using Trajectory = ::autoware_planning_msgs::Trajectory;
using TrajectoryPoint = ::autoware_planning_msgs::TrajectoryPoint;
}
namespace autoware_vehicle_msgs::msg
{
using SteeringReport = ::autoware_vehicle_msgs::SteeringReport;
}
namespace geometry_msgs::msg
{
using Point = ::geometry_msgs::Point;
using Pose = ::geometry_msgs::Pose;
using PoseWithCovariance = ::geometry_msgs::PoseWithCovariance;
}
namespace nav_msgs::msg
{
using Odometry = ::nav_msgs::Odometry;
}
namespace std_msgs::msg
{
using ColorRGBA = ::std_msgs::ColorRGBA;
}
namespace visualization_msgs::msg
{
using Marker = ::visualization_msgs::Marker;
using MarkerArray = ::visualization_msgs::MarkerArray;
}
namespace autoware::boundary_departure_checker::ros1_compat
{
class Time : public ::ros::Time
{
public:
  Time() = default;
  explicit Time(const ::ros::Time & value) : ::ros::Time(value) {}
  double seconds() const { return toSec(); }
};

class Duration
{
public:
  explicit Duration(const builtin_interfaces::Duration & value)
  : seconds_(static_cast<double>(value.sec) + static_cast<double>(value.nanosec) * 1.0e-9)
  {
  }

  explicit Duration(const double seconds) : seconds_(seconds) {}
  double seconds() const { return seconds_; }
  Duration operator-(const Duration & other) const { return Duration(seconds_ - other.seconds_); }

private:
  double seconds_;
};

class Clock
{
public:
  using SharedPtr = std::shared_ptr<Clock>;
  Time now() const { return Time(::ros::Time::now()); }
};
}  // namespace autoware::boundary_departure_checker::ros1_compat

#endif  // AUTOWARE__DEPRECATED__BOUNDARY_DEPARTURE_CHECKER__ROS1_COMPAT_HPP_
