#pragma once

namespace autoware::behavior_path_planner
{
// Re-acquire, never synthesize, the latest ego messages after the other input
// queues were serviced. Call only before construction of per-cycle module data.
// This is not synchronization: genuine timestamp skew is still checked by the
// existing consumers. The object snapshot and all source timestamps are intact.
template<class OdometrySubscriber, class AccelerationSubscriber, class OdometryPtr, class AccelerationPtr>
bool final_ego_poll(
  const bool enabled, OdometrySubscriber & odometry_subscriber,
  AccelerationSubscriber & acceleration_subscriber, OdometryPtr & odometry,
  AccelerationPtr & acceleration)
{
  if (!enabled) return false;
  const auto old_odometry = odometry;
  const auto old_acceleration = acceleration;
  if (const auto message = odometry_subscriber.take_data()) odometry = message;
  if (const auto message = acceleration_subscriber.take_data()) acceleration = message;
  return odometry != old_odometry || acceleration != old_acceleration;
}
}  // namespace autoware::behavior_path_planner
