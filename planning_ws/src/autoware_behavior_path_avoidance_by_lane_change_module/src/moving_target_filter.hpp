#pragma once

#include <autoware/behavior_path_static_obstacle_avoidance_module/utils.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>

// Existing exported static-avoidance predicate; its declaration is private to
// utils.cpp upstream. Reuse that implementation and this module's parameters.
namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance::filtering_utils
{
bool isMovingObject(
  const ObjectData &, const std::shared_ptr<AvoidanceParameters> &);
}

namespace autoware::behavior_path_planner::avoidance_lane_change_detail
{
inline bool excludeMovingTarget(
  const ObjectData & object, const std::shared_ptr<AvoidanceParameters> & parameters,
  ObjectDataArray & other_objects)
{
  if (!utils::static_obstacle_avoidance::filtering_utils::isMovingObject(object, parameters)) {
    return false;
  }
  auto ignored = object;
  ignored.info = ObjectInfo::MOVING_OBJECT;
  // Lost-target compensation explicitly checks other_objects. Dropping the
  // object alone would allow its previously stored target to be resurrected.
  other_objects.push_back(std::move(ignored));
  return true;
}

inline bool excludeMovingCenterFilteredObject(
  const PredictedObject & raw, ObjectDataArray & stopped_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters, ObjectDataArray & other_objects)
{
  // createObjectData's sole nullopt branch is before its moving-time update.
  // Update exactly once here; otherwise a cached off-center target can return
  // through lost-target compensation after moving onto the lane center.
  const auto type = autoware::object_recognition_utils::getHighestProbLabel(raw.classification);
  const auto & velocity = raw.kinematics.initial_twist_with_covariance.twist.linear;
  if (!(std::hypot(velocity.x, velocity.y) >
        parameters->object_parameters.at(type).moving_speed_threshold)) {
    return false;  // Preserve historical center-filtered stationary bookkeeping.
  }
  ObjectData object;
  object.object = raw;
  utils::static_obstacle_avoidance::fillObjectMovingTime(object, stopped_objects, parameters);
  return excludeMovingTarget(object, parameters, other_objects);
}
}
