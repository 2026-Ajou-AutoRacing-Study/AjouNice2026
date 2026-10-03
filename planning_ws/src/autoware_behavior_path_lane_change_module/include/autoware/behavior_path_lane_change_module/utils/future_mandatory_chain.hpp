#ifndef AUTOWARE_LC_FUTURE_MANDATORY_CHAIN_HPP
#define AUTOWARE_LC_FUTURE_MANDATORY_CHAIN_HPP
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change::future_chain
{
using Id = int64_t;
// route_order is the live route section order. Only real vehicle-graph
// following edges may be supplied; no lateral edges or inferred XY links.
struct Node {
  double length{};
  int route_order{};
  bool route{}, forward{};
  std::vector<Id> following;
};
struct Limits {size_t expansions{512}, depth{64}; double distance{1000};};
struct Proof {bool valid{}; std::string reason; std::vector<Id> chain; size_t expansions{};};
struct Gate {
  bool enabled{}, normal{}, approved{}, aborting{}, stopping{}, fresh_route{};
  bool legal_adjacent{}, same_direction{}, future_mandatory{};
  double target_distance{}, return_reserve{}, required_lc{}, motion_ahead{};
};
template<class Pointer, class Uuid, class Stamp>
inline bool same_route_epoch(const Pointer & input, const Uuid & uuid, const Stamp & stamp)
{
  return input && input->uuid==uuid && input->header.stamp==stamp;
}
template<class Pointer>
inline bool same_route_generation(const Pointer & captured, const Pointer & current)
{
  return captured && captured==current;
}
inline bool applicable(const Gate & g)
{
  if (!g.enabled || !g.normal || g.approved || g.aborting || g.stopping ||
      !g.fresh_route || !g.legal_adjacent || !g.same_direction || !g.future_mandatory) return false;
  for (const auto v : {g.target_distance,g.return_reserve,g.required_lc,g.motion_ahead})
    if (!std::isfinite(v) || v<0) return false;
  const double horizon=g.return_reserve+g.required_lc+g.motion_ahead;
  return std::isfinite(horizon) && g.target_distance>0 && g.target_distance<=horizon;
}
inline Proof prove(const std::map<Id,Node> & graph, Id neighbor, Id future, const Limits & lim={})
{
  Proof out;
  if (!lim.expansions || !lim.depth || !std::isfinite(lim.distance) || lim.distance<=0) {
    out.reason="invalid_budget"; return out;
  }
  const auto valid=[&](Id id) {
    const auto p=graph.find(id);
    return p!=graph.end() && p->second.route && p->second.forward &&
      std::isfinite(p->second.length) && p->second.length>0;
  };
  if (!valid(neighbor) || !valid(future) || neighbor==future) {
    out.reason="invalid_endpoints"; return out;
  }
  struct State {std::vector<Id> path; double length;};
  std::vector<State> stack{{{neighbor},graph.at(neighbor).length}};
  bool incomplete=false;
  size_t solutions=0;
  while (!stack.empty()) {
    if (out.expansions++>=lim.expansions) {incomplete=true; break;}
    auto state=std::move(stack.back()); stack.pop_back();
    const auto id=state.path.back();
    if (state.length>lim.distance) {incomplete=true; continue;}
    if (id==future) {
      out.chain=state.path;
      if (++solutions>1) {out.chain.clear();out.reason="ambiguous_route_chain";return out;}
      continue;
    }
    if (state.path.size()>=lim.depth) {incomplete=true;continue;}
    std::set<Id> unique;
    for (Id next:graph.at(id).following) {
      if (!unique.insert(next).second || !valid(next)) continue;
      if (graph.at(next).route_order<graph.at(id).route_order) continue;
      if (std::find(state.path.begin(),state.path.end(),next)!=state.path.end()) continue;
      auto path=state.path; path.push_back(next);
      stack.push_back({std::move(path),state.length+graph.at(next).length});
    }
  }
  if (incomplete) {out.chain.clear();out.reason="proof_budget_exhausted";return out;}
  out.valid=solutions==1;
  out.reason=out.valid?"unique_route_following_chain":"no_proved_chain";
  return out;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::future_chain
#endif
