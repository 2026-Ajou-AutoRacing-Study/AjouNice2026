#ifndef AUTOWARE_UTILS_TF__SELF_POSE_LISTENER_HPP_
#define AUTOWARE_UTILS_TF__SELF_POSE_LISTENER_HPP_

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_tf/transform_listener.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace autoware_utils_tf
{
class SelfPoseListener
{
public:
  explicit SelfPoseListener(const ros::NodeHandle & nh) : transform_listener_(nh) {}

  void wait_for_first_pose()
  {
    ros::WallRate poll_rate(0.2);
    while (ros::ok()) {
      if (get_current_pose()) {
        return;
      }
      ROS_INFO("waiting for self pose...");
      poll_rate.sleep();
    }
  }

  geometry_msgs::PoseStamped::ConstPtr get_current_pose() const
  {
    const geometry_msgs::TransformStamped::ConstPtr tf =
      transform_listener_.get_latest_transform("map", "base_link");
    if (!tf) {
      return geometry_msgs::PoseStamped::ConstPtr();
    }
    return geometry_msgs::PoseStamped::ConstPtr(
      new geometry_msgs::PoseStamped(autoware_utils_geometry::transform2pose(*tf)));
  }

private:
  TransformListener transform_listener_;
};
}  // namespace autoware_utils_tf

#endif  // AUTOWARE_UTILS_TF__SELF_POSE_LISTENER_HPP_
