#ifndef AUTOWARE_LC_BOUNDED_OUTPUT_POLICY_HPP
#define AUTOWARE_LC_BOUNDED_OUTPUT_POLICY_HPP
#include <cmath>
#include <cstdint>
#include <string>
#include <tuple>
namespace autoware::behavior_path_planner::utils::lane_change {
// Identity, not a remembered safety verdict. A matching output is checked again.
struct SelectedOutputIdentity {
  uint64_t cycle{}, odom_stamp{}, object_stamp{}, route_stamp{}, acceleration_stamp{};
  uintptr_t odom{}, objects{}, acceleration{}, route{};
  std::string route_uuid;
  double measured_acceleration{};
  bool diagnostic_only{};
  bool matches(const SelectedOutputIdentity & other) const {
    return odom_stamp && object_stamp && std::isfinite(measured_acceleration) &&
      std::isfinite(other.measured_acceleration) &&
      std::tie(cycle,odom_stamp,object_stamp,route_stamp,acceleration_stamp,odom,objects,
        acceleration,route,route_uuid,measured_acceleration,diagnostic_only)==
      std::tie(other.cycle,other.odom_stamp,other.object_stamp,other.route_stamp,
        other.acceleration_stamp,other.odom,other.objects,other.acceleration,other.route,
        other.route_uuid,other.measured_acceleration,other.diagnostic_only);
  }
};
inline bool owns_bounded_longitudinal_output(
  bool enabled,bool grid_enabled,bool aborting,bool stopping)
{
  return enabled && grid_enabled && !aborting && !stopping;
}
inline bool may_publish_original_longitudinal(bool original_safe,bool bounded_owner)
{
  return original_safe && !bounded_owner;
}
// One attempt per output decision, including exceptional grid/fallback paths.
// No module state changes here: the caller must first validate the exact
// upstream output that the manager will publish after cancellation.
struct ValidatedCancelOrder {
  bool attempted=false;
  bool claim(bool bounded_owner,bool original_safe,bool after_grid) {
    if(attempted || original_safe || bounded_owner!=after_grid)return false;
    attempted=true;
    return true;
  }
};
}
#endif
