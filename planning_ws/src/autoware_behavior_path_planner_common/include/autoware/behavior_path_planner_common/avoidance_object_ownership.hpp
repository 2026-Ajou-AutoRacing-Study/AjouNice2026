#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__AVOIDANCE_OBJECT_OWNERSHIP_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__AVOIDANCE_OBJECT_OWNERSHIP_HPP_

#include <autoware_perception_msgs/PredictedObjects.h>
#include <unique_identifier_msgs/UUID.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>

namespace autoware::behavior_path_planner
{
// Speed-only Schmitt trigger. No dwell, observation counter, position-derived
// velocity, synthetic prediction, or collision-geometry/margin modification.
class AvoidanceObjectOwnership
{
public:
  explicit AvoidanceObjectOwnership(double dynamic_enter_speed = 0.5, double static_enter_speed = 0.2)
  : dynamic_enter_speed_(dynamic_enter_speed), static_enter_speed_(static_enter_speed)
  {
    if (!std::isfinite(dynamic_enter_speed_) || !std::isfinite(static_enter_speed_) ||
        static_enter_speed_ < 0.0 || static_enter_speed_ >= dynamic_enter_speed_) {
      throw std::invalid_argument("invalid avoidance ownership speed thresholds");
    }
  }

  void update(const autoware_perception_msgs::PredictedObjects & objects)
  {
    if (initialized_ && objects.header.stamp == last_stamp_) return;
    if (initialized_ && objects.header.stamp < last_stamp_) clear();
    for (const auto & object : objects.objects) {
      const auto & v = object.kinematics.initial_twist_with_covariance.twist.linear;
      const double speed = std::hypot(v.x, v.y);
      const auto id = key(object.object_id);
      // Invalid velocity must not silently release a dynamic collision constraint.
      if (!std::isfinite(speed) || speed >= dynamic_enter_speed_) {
        dynamic_[id] = true;
      } else if (speed <= static_enter_speed_) {
        dynamic_[id] = false;
      } else if (dynamic_.count(id) == 0) {
        dynamic_[id] = false;
      }
    }
    last_stamp_ = objects.header.stamp;
    initialized_ = true;
  }

  std::optional<bool> isDynamic(const unique_identifier_msgs::UUID & id) const
  {
    const auto found = dynamic_.find(key(id));
    if (found == dynamic_.end()) return std::nullopt;
    return found->second;
  }

  void clear()
  {
    dynamic_.clear();
    initialized_ = false;
  }

private:
  static std::array<uint8_t, 16> key(const unique_identifier_msgs::UUID & id)
  {
    std::array<uint8_t, 16> result{};
    std::copy(id.uuid.begin(), id.uuid.end(), result.begin());
    return result;
  }
  double dynamic_enter_speed_;
  double static_enter_speed_;
  // Retain only ownership through missing detections; never synthesize occupancy.
  // A route change or replay clock rewind clears this registry.
  std::map<std::array<uint8_t, 16>, bool> dynamic_;
  ros::Time last_stamp_;
  bool initialized_{false};
};
}  // namespace autoware::behavior_path_planner

#endif
