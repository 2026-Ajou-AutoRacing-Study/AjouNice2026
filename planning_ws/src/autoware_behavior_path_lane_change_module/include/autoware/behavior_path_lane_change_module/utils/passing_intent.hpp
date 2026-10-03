#pragma once

// Simulator-only arbitration. This kernel never approves a maneuver or changes
// its geometry/RSS checks; it can only defer a NEW convenience-return request.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>

namespace autoware::behavior_path_planner::passing_intent
{
inline constexpr const char * map_sha =
  "5111ae16ba788b059e40e88201b51b0cb4d1d662dcd8456f43cda9705066be58";
inline constexpr const char * route_uuid = "532d0a63426a59aabfdfac4aa566c177";

// Registered candidates are reused without calling isExecutionRequested again.
// Refresh observation only; this gate grants no activation or suppression.
inline bool refresh_waiting_candidate_ack(
  const bool enabled, const bool external_request, const bool waiting_approval,
  const bool activated, const bool aborting)
{
  return enabled && external_request && waiting_approval && !activated && !aborting;
}

struct Intent
{
  std::string id, outbound_uuid, lead_uuid, route, boot, map, direction, phase;
  double route_stamp{0}, stamp{0}, ttl{0};
  std::set<int64_t> source, target;
};
struct Context
{
  bool enabled{false}, route_verified{false}, corridor_verified{false};
  bool normal{false}, running{false}, aborting{false}, near_terminal{true};
  bool distances_valid{false};
  std::string route, boot, direction;
  double now{0}, route_stamp{0}, ownership_stamp{0}, intent_wall_age{0};
  double remaining{0}, terminal_distance{0}, lateral_distance{0}, return_buffer{0}, speed{0};
  double acceleration{0}, deceleration{0}, jerk{0}, front_margin{0};
  int64_t current_lane{0}, target_lane{0};
  std::set<int64_t> corridor;
};
struct Decision
{
  bool acknowledged{false}, suppress{false};
  double reserve{0};
  std::string reason{"disabled"};
};
inline double stop_distance(double speed, double acceleration, double deceleration, double jerk)
{
  // Constant negative-jerk ramp to the active braking limit, then constant
  // braking. Retain v^2/(2b) as a floor even if already braking harder.
  const double a=std::max(acceleration,-deceleration);
  const double ramp=std::max(0.0,(a+deceleration)/jerk);
  const double zero=(a+std::sqrt(a*a+2*jerk*speed))/jerk;
  const double t=std::min(ramp,zero);
  const double distance=speed*t+0.5*a*t*t-jerk*t*t*t/6;
  const double remaining_velocity=std::max(0.0,speed+a*t-0.5*jerk*t*t);
  return std::max(speed*speed/(2*deceleration),distance+remaining_velocity*remaining_velocity/(2*deceleration));
}
inline bool subset(const std::set<int64_t> & a, const std::set<int64_t> & b)
{
  return !a.empty() && std::includes(b.begin(), b.end(), a.begin(), a.end());
}
inline Decision decide(const Intent & i, const Context & c, bool outbound_seen)
{
  Decision d;
  auto reject = [&](const char * why) { d.reason = why; return d; };
  if (!c.enabled) return d;
  if (!std::isfinite(i.route_stamp) || !std::isfinite(c.route_stamp) ||
      !c.route_verified || !c.corridor_verified || i.route != route_uuid ||
      i.route != c.route || std::abs(i.route_stamp-c.route_stamp) > 1e-6 ||
      i.boot.empty() || i.boot != c.boot || i.map != map_sha)
    return reject("route_map_epoch_mismatch");
  if (i.id.empty() || i.outbound_uuid.empty() || i.lead_uuid.empty() ||
      (i.direction != "left" && i.direction != "right") ||
      (i.phase != "ARM" && i.phase != "OUTBOUND" && i.phase != "PASSING"))
    return reject("invalid_intent");
  if (!std::isfinite(c.now) || !std::isfinite(c.ownership_stamp) || !std::isfinite(c.intent_wall_age) ||
      !std::isfinite(i.stamp) || !std::isfinite(i.ttl) ||
      !(i.ttl > 0 && i.ttl <= 0.5) || c.now-i.stamp < 0 || c.now-i.stamp > i.ttl ||
      c.intent_wall_age < 0 || c.intent_wall_age > i.ttl ||
      c.now-c.ownership_stamp < 0 || c.now-c.ownership_stamp > 0.4)
    return reject("stale_intent_or_ownership");
  if (!subset(i.source,c.corridor) || !subset(i.target,c.corridor))
    return reject("unverified_lane_chain");
  for (const auto lane : i.source) if (i.target.count(lane)) return reject("overlapping_lane_chains");
  if (!c.distances_valid || !std::isfinite(c.remaining) || !std::isfinite(c.terminal_distance) ||
      !std::isfinite(c.lateral_distance) || !std::isfinite(c.return_buffer) || !std::isfinite(c.speed) ||
      !std::isfinite(c.acceleration) || !std::isfinite(c.deceleration) || !std::isfinite(c.jerk) ||
      !std::isfinite(c.front_margin) || c.front_margin<0 || c.deceleration<=0 || c.jerk<=0 ||
      c.return_buffer < 0 || c.speed < 0)
    return reject("invalid_return_reserve");
  // return_buffer already includes prepare; do not add max_prepare again.
  // Continue current positive acceleration during the intent's expiry interval.
  const double positive_acceleration=std::max(c.acceleration,0.0);
  const double reaction=c.speed*i.ttl+0.5*positive_acceleration*i.ttl*i.ttl;
  const double braking=stop_distance(c.speed+positive_acceleration*i.ttl,
                                   c.acceleration,c.deceleration,c.jerk);
  d.reserve=std::max(150.0,std::max(c.return_buffer,braking)+reaction+c.front_margin);
  const double available=std::min({c.remaining,c.terminal_distance,c.lateral_distance});
  if (c.near_terminal || available <= d.reserve)
    return reject("mandatory_route_priority");
  if (!i.source.count(c.current_lane) && !i.target.count(c.current_lane))
    return reject("outside_intent_lanes");
  d.acknowledged = true;
  d.reason = "armed";
  if (c.running || c.aborting) return reject("running_owner_unchanged");
  if (!c.normal || !outbound_seen || !i.target.count(c.current_lane)) return d;
  if (i.phase == "ARM") return d;
  const auto return_direction = i.direction == "left" ? "right" : "left";
  if (c.direction != return_direction || !i.source.count(c.target_lane)) return d;
  d.suppress = true;
  d.reason = "defer_convenience_return_for_pass";
  return d;
}
}  // namespace autoware::behavior_path_planner::passing_intent
