#ifndef AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_
#define AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace autoware_utils_tf
{
class TransformListener
{
public:
  explicit TransformListener(const ros::NodeHandle & nh)
  : tf_buffer_(), tf_listener_(tf_buffer_, nh, true)
  {
  }

  geometry_msgs::TransformStamped::ConstPtr get_latest_transform(
    const std::string & from, const std::string & to) const
  {
    try {
      return geometry_msgs::TransformStamped::ConstPtr(
        new geometry_msgs::TransformStamped(tf_buffer_.lookupTransform(from, to, ros::Time(0))));
    } catch (const tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(5.0, "failed to get transform from %s to %s: %s", from.c_str(), to.c_str(), ex.what());
      return geometry_msgs::TransformStamped::ConstPtr();
    }
  }

  geometry_msgs::TransformStamped::ConstPtr get_transform(
    const std::string & from, const std::string & to, const ros::Time & time,
    const ros::Duration & timeout) const
  {
    try {
      return geometry_msgs::TransformStamped::ConstPtr(
        new geometry_msgs::TransformStamped(tf_buffer_.lookupTransform(from, to, time, timeout)));
    } catch (const tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(5.0, "failed to get transform from %s to %s: %s", from.c_str(), to.c_str(), ex.what());
      return geometry_msgs::TransformStamped::ConstPtr();
    }
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace autoware_utils_tf

#endif  // AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_
