#include <cmath>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include "purepursuit_pi/controller.hpp"

namespace purepursuit_pi {
bool Controller::updatePose(const ros::Time& stamp, EgoState* state) const {
  try {
    const auto transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, stamp, ros::Duration(0.05));
    state->stamp = stamp;
    state->x = transform.transform.translation.x;
    state->y = transform.transform.translation.y;
    state->z = transform.transform.translation.z;
    state->yaw = tf2::getYaw(transform.transform.rotation);
    state->valid = true;
    return true;
  } catch (const tf2::TransformException& error) {
    ROS_WARN_THROTTLE(1.0, "purepursuit_pi: missing %s -> %s TF at input stamp: %s", map_frame_.c_str(), base_frame_.c_str(), error.what());
    state->valid = false;
    return false;
  }
}

void Controller::moraiCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& message) {
  EgoState state;
  if (!updatePose(message->header.stamp, &state)) return;
  // MORAI EgoVehicleStatus velocity is consumed as the ego body-frame velocity source.
  state.long_vel = message->velocity.x;
  state.lat_vel = message->velocity.y;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ego_ = state;
  }
  publishEgoState(state);
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
  EgoState state;
  if (!updatePose(message->header.stamp, &state)) return;
  state.long_vel = message->twist.twist.linear.x;
  state.lat_vel = message->twist.twist.linear.y;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ego_ = state;
  }
  publishEgoState(state);
}
}  // namespace purepursuit_pi
