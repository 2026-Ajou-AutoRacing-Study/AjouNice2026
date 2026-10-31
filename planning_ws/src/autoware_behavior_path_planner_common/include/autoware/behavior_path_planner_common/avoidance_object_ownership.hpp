#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__AVOIDANCE_OBJECT_OWNERSHIP_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__AVOIDANCE_OBJECT_OWNERSHIP_HPP_

#include <autoware_perception_msgs/PredictedObjects.h>
#include <unique_identifier_msgs/UUID.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
  : AvoidanceObjectOwnership(
      dynamic_enter_speed, static_enter_speed, dynamic_enter_speed, static_enter_speed)
  {
  }

  AvoidanceObjectOwnership(
    double dynamic_enter_speed, double static_enter_speed,
    double pedestrian_dynamic_enter_speed, double pedestrian_static_enter_speed)
  : dynamic_enter_speed_(dynamic_enter_speed), static_enter_speed_(static_enter_speed),
    pedestrian_dynamic_enter_speed_(pedestrian_dynamic_enter_speed),
    pedestrian_static_enter_speed_(pedestrian_static_enter_speed)
  {
    if (!validThresholds(dynamic_enter_speed_, static_enter_speed_) ||
        !validThresholds(pedestrian_dynamic_enter_speed_, pedestrian_static_enter_speed_)) {
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
      // Apply the slower entry only to an unambiguous, valid pedestrian class.
      // Missing/invalid/tied classification retains the generic faster entry.
      const bool pedestrian = isUnambiguousPedestrian(object);
      const double dynamic_enter = pedestrian ? pedestrian_dynamic_enter_speed_ : dynamic_enter_speed_;
      const double static_enter = pedestrian ? pedestrian_static_enter_speed_ : static_enter_speed_;
      // Invalid velocity must not silently release a dynamic collision constraint.
      if (!std::isfinite(speed) || speed >= dynamic_enter) {
        dynamic_[id] = true;
      } else if (speed <= static_enter) {
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
  static bool validThresholds(double dynamic_enter, double static_enter)
  {
    return std::isfinite(dynamic_enter) && std::isfinite(static_enter) &&
      static_enter >= 0.0 && static_enter < dynamic_enter;
  }

  static bool isUnambiguousPedestrian(const autoware_perception_msgs::PredictedObject & object)
  {
    using Classification = autoware_perception_msgs::ObjectClassification;
    double highest_probability = 0.0;
    uint8_t highest_label = Classification::UNKNOWN;
    bool tied = false;
    for (const auto & classification : object.classification) {
      const double probability = classification.probability;
      if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0 ||
          classification.label > Classification::PEDESTRIAN) {
        return false;
      }
      if (probability > highest_probability) {
        highest_probability = probability;
        highest_label = classification.label;
        tied = false;
      } else if (probability == highest_probability) {
        tied = true;
      }
    }
    return highest_probability > 0.0 && !tied && highest_label == Classification::PEDESTRIAN;
  }

  static std::array<uint8_t, 16> key(const unique_identifier_msgs::UUID & id)
  {
    std::array<uint8_t, 16> result{};
    std::copy(id.uuid.begin(), id.uuid.end(), result.begin());
    return result;
  }
  double dynamic_enter_speed_;
  double static_enter_speed_;
  double pedestrian_dynamic_enter_speed_;
  double pedestrian_static_enter_speed_;
  // Retain only ownership through missing detections; never synthesize occupancy.
  // A route change or replay clock rewind clears this registry.
  std::map<std::array<uint8_t, 16>, bool> dynamic_;
  ros::Time last_stamp_;
  bool initialized_{false};
};
}  // namespace autoware::behavior_path_planner

#endif
