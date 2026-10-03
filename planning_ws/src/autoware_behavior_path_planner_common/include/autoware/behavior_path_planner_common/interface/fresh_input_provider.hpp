#pragma once

#include <ros/time.h>
#include <unique_identifier_msgs/UUID.h>
#include <functional>
#include <memory>
#include <optional>
#include <cstdint>

namespace autoware::behavior_path_planner
{
struct PlannerData;
namespace fresh_commit
{
// Real messages only. Const ownership does not make the shared RouteHandler
// immutable: consumers must also verify the captured route/map identity.
struct Snapshot
{
  std::shared_ptr<const PlannerData> planner_data;
  ros::Time acquired_at;
  ros::WallTime acquired_wall;
  uint64_t cycle{0};
  const void * owner{nullptr};
  // RouteHandler::getLaneletMapPtr().get(), NOT the LaneletMapBin message address.
  const void * map_identity{nullptr};
  const void * route_message_identity{nullptr};
  const void * route_handler{nullptr};
  unique_identifier_msgs::UUID route_uuid;
  ros::Time route_stamp;
};
using Provider = std::function<std::optional<Snapshot>()>;

// Thread-local, stack-scoped registration in the COMMON DSO, not one registry
// per plugin. No callback survives manager.run(), including exceptional exits.
class ScopedProvider
{
public:
  explicit ScopedProvider(Provider provider);
  ~ScopedProvider();
  ScopedProvider(const ScopedProvider &) = delete;
  ScopedProvider & operator=(const ScopedProvider &) = delete;
  uint64_t cycle() const noexcept { return cycle_; }
private:
  friend std::optional<Snapshot> acquire();
  Provider provider_;
  ScopedProvider * previous_;
  uint64_t cycle_;
  bool invoking_{false};
};
// Missing, recursive, or throwing providers are unavailable, never stale data.
std::optional<Snapshot> acquire();
}  // namespace fresh_commit
}  // namespace autoware::behavior_path_planner
