#ifndef AUTOWARE_LC_PRE_ADJACENCY_POLICY_HPP
#define AUTOWARE_LC_PRE_ADJACENCY_POLICY_HPP
#include "autoware/behavior_path_lane_change_module/utils/passing_intent.hpp"
namespace autoware::behavior_path_planner::utils::lane_change {
struct PreAdjacencyContext {
  bool enabled{false}, normal{false}, approved{true}, topology_verified{false};
  bool fresh_inputs{false}, chain_valid{false}, distances_valid{false}, near_terminal{true};
  double target_start{}, remaining{}, required_lc{}, speed{}, acceleration{};
  double deceleration{}, jerk{}, reaction{}, front_margin{};
};
struct PreAdjacencyDecision {bool defer{false};double reserve{0};};
inline PreAdjacencyDecision defer_pre_adjacency(const PreAdjacencyContext & c)
{
  PreAdjacencyDecision d;
  if(!c.enabled || !c.normal || c.approved || !c.topology_verified || !c.fresh_inputs ||
     !c.chain_valid || !c.distances_valid || c.near_terminal)return d;
  for(const double x:{c.target_start,c.remaining,c.required_lc,c.speed,c.acceleration,
      c.deceleration,c.jerk,c.reaction,c.front_margin})if(!std::isfinite(x))return d;
  if(c.target_start<=0 || c.remaining<=0 || c.required_lc<0 || c.speed<0 ||
     c.deceleration<=0 || c.jerk<=0 || c.reaction<0 || c.front_margin<0)return d;
  const auto stop=passing_intent::stop_distance(c.speed,c.acceleration,c.deceleration,c.jerk);
  d.reserve=c.target_start+std::max(c.required_lc,stop)+
    c.speed*c.reaction+.5*std::max(0.0,c.acceleration)*c.reaction*c.reaction+c.front_margin;
  if(!std::isfinite(d.reserve))return d;
  d.defer=c.remaining>d.reserve;
  return d;
}
}
#endif
