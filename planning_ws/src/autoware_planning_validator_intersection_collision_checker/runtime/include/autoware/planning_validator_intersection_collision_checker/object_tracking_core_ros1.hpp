#ifndef AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_OBJECT_TRACKING_CORE_ROS1_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_OBJECT_TRACKING_CORE_ROS1_HPP_

#include <ros/time.h>

#include <cmath>
#include <deque>
#include <limits>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
// Direct ROS1-time rendering of PCDObject's source state transition.  The
// surrounding lanelet/PCL plugin decides reliability and collision policy;
// this core preserves only the original per-object history semantics.
class ObjectTrackingCore
{
public:
  explicit ObjectTrackingCore(const ros::Time & stamp, const double distance)
  : last_update_time_(stamp), distance_to_overlap_(distance), delay_compensated_distance_to_overlap_(distance)
  {
    distance_history_.push_back(distance);
    timestamp_history_.push_back(0.0);
  }

  void set_buffer_size(const size_t size) { buffer_size_ = size; }
  double velocity() const { return velocity_; }
  double track_duration() const { return track_duration_; }
  double distance_to_overlap() const { return distance_to_overlap_; }
  const ros::Time & last_update_time() const { return last_update_time_; }
  size_t history_size() const { return distance_history_.size(); }

  void update(
    const ros::Time & stamp, const double distance, const double raw_velocity_threshold,
    const double acceleration_threshold, const double min_distance_threshold)
  {
    const double dt = (stamp - last_update_time_).toSec();
    if (dt < 1e-6) return;
    if (is_reliable_ && delay_compensated_distance_to_overlap_ < min_distance_threshold) {
      update_history(distance, dt);
      track_duration_ += dt;
      last_update_time_ = stamp;
      distance_to_overlap_ = distance;
      return;
    }
    const double raw_velocity = (distance_to_overlap_ - distance) / dt;
    if (std::abs(raw_velocity) > raw_velocity_threshold) {
      reset();
      distance_history_.push_back(distance);
      timestamp_history_.push_back(0.0);
      last_update_time_ = stamp;
      distance_to_overlap_ = distance;
      return;
    }
    const double previous_velocity = velocity_;
    update_history(distance, dt);
    update_velocity();
    track_duration_ += dt;
    if (distance_history_.size() >= 3U && std::abs((velocity_ - previous_velocity) / dt) > acceleration_threshold) {
      reset();
      distance_history_.push_back(distance);
      timestamp_history_.push_back(0.0);
    }
    last_update_time_ = stamp;
    distance_to_overlap_ = distance;
  }

  void set_reliable(const bool reliable) { is_reliable_ = reliable; }

private:
  void update_history(const double distance, const double dt)
  {
    const double last = timestamp_history_.empty() ? 0.0 : timestamp_history_.back();
    distance_history_.push_back(distance);
    timestamp_history_.push_back(last + dt);
    if (distance_history_.size() > buffer_size_) distance_history_.pop_front();
    if (timestamp_history_.size() > buffer_size_) timestamp_history_.pop_front();
  }
  void update_velocity()
  {
    if (distance_history_.size() < 2U) return;
    const auto n = distance_history_.size();
    const double x0 = timestamp_history_.front();
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double x = timestamp_history_[i] - x0;
      const double y = distance_history_[i];
      sum_x += x; sum_y += y; sum_xy += x * y; sum_x2 += x * x;
    }
    const double denominator = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) return;
    velocity_ = -1.0 * (n * sum_xy - sum_x * sum_y) / denominator;
  }
  void reset()
  {
    velocity_ = 0.0; track_duration_ = 0.0; distance_history_.clear(); timestamp_history_.clear(); is_reliable_ = false;
  }

  size_t buffer_size_{10U};
  ros::Time last_update_time_;
  std::deque<double> distance_history_, timestamp_history_;
  double track_duration_{0.0}, distance_to_overlap_{0.0}, delay_compensated_distance_to_overlap_{0.0}, velocity_{0.0};
  bool is_reliable_{false};
};
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1

#endif
