#ifndef AUTOWARE_LC_FUTURE_HANDOFF_HPP
#define AUTOWARE_LC_FUTURE_HANDOFF_HPP
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/Point.h>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change::future_handoff
{
using Id=int64_t;
struct Result {bool allowed{};std::string reason;};
inline Result check(const std::vector<Id>&proved,Id future,Id native_owner,
                    const std::set<Id>&current_containing,
                    const std::set<Id>&target_containing,
                    const std::vector<Id>&native_sequence)
{
  if(proved.empty()||proved.size()>128||native_sequence.empty()||native_sequence.size()>128)
    return {false,"invalid_or_unbounded_proof"};
  if(!current_containing.empty())return {false,"end_still_overlaps_current"};
  if(!target_containing.count(native_owner))return {false,"native_owner_not_target"};
  const auto begin=std::find(proved.begin(),proved.end(),native_owner);
  const auto future_at=std::find(proved.begin(),proved.end(),future);
  if(begin==proved.end()||future_at==proved.end())return {false,"owner_outside_proved_suffix"};
  if(std::set<Id>(proved.begin(),proved.end()).size()!=proved.size())return {false,"ambiguous_proof"};
  // A candidate ending beyond the future target has already traversed its
  // proved suffix. Never demand that native following travel backwards to it.
  const auto end=std::max(begin,future_at);
  const auto count=static_cast<size_t>(end-begin)+1;
  if(native_sequence.size()<count)return {false,"native_sequence_incomplete"};
  if(!std::equal(begin,end+1,native_sequence.begin()))return {false,"native_following_diverges"};
  for(Id owner:target_containing)
    if(std::find(proved.begin(),proved.end(),owner)==proved.end())return {false,"ambiguous_target_owner"};
  return {true,"native_handoff_preserves_proved_suffix"};
}

// Same eligible IDs and closest-lane selector as getCurrentLanesFromPath;
// global closest-route-lane selection is NOT substituted here. Pure input
// geometry only: no approval, speed/profile change, or lifecycle mutation.
template<class Path,class Pose,class Handler>
Result evaluate(const Path&path,const Pose&end_pose,const lanelet::ConstLanelets&current,
                const lanelet::ConstLanelets&target,Id future,const Handler&handler)
{
  if(path.points.empty()||current.empty()||target.empty()||target.size()>128)
    return {false,"missing_handoff_geometry"};
  for(double v:{end_pose.position.x,end_pose.position.y,end_pose.position.z,
      end_pose.orientation.x,end_pose.orientation.y,end_pose.orientation.z,end_pose.orientation.w})
    if(!std::isfinite(v))return {false,"nonfinite_handoff_pose"};
  std::set<Id> ids;
  for(const auto&point:path.points)ids.insert(point.lane_ids.begin(),point.lane_ids.end());
  if(ids.empty()||ids.size()>256)return {false,"invalid_handoff_lane_ids"};
  lanelet::ConstLanelets reference;
  for(Id id:ids) {
    const auto lane=handler.getLaneletsFromId(id);
    if(!handler.isRouteLanelet(lane))return {false,"handoff_lane_outside_route"};
    reference.push_back(lane);
  }
  const auto closest=autoware::experimental::lanelet2_utils::get_closest_lanelet(reference,end_pose);
  if(!closest)return {false,"native_owner_unavailable"};
  const lanelet::BasicPoint2d point(end_pose.position.x,end_pose.position.y);
  std::set<Id> current_hits,target_hits;
  std::vector<Id> proof;
  for(const auto&lane:current)
    if(boost::geometry::covered_by(point,lane.polygon2d().basicPolygon()))current_hits.insert(lane.id());
  for(const auto&lane:target) {
    proof.push_back(lane.id());
    if(boost::geometry::covered_by(point,lane.polygon2d().basicPolygon()))target_hits.insert(lane.id());
  }
  // Check every eligible containing target owner, not only a lucky closest
  // tie. Current overlap rejects before ownership can be handed back.
  auto owners=target_hits;owners.insert(closest->id());
  for(Id owner:owners) {
    std::vector<Id> native{owner};std::set<Id> visited{owner};
    const auto begin=std::find(proof.begin(),proof.end(),owner);
    const auto goal=std::find(proof.begin(),proof.end(),future);
    if(begin==proof.end()||goal==proof.end())return {false,"owner_outside_proved_suffix"};
    if(begin<goal) {
      auto lane=handler.getLaneletsFromId(owner);
      while(native.back()!=future&&native.size()<128) {
        lanelet::ConstLanelet next;
        if(!handler.getNextLaneletWithinRoute(lane,&next)||!visited.insert(next.id()).second)break;
        native.push_back(next.id());lane=next;
      }
    }
    const auto result=check(proof,future,owner,current_hits,target_hits,native);
    if(!result.allowed)return result;
  }
  return {true,"native_handoff_preserves_proved_suffix"};
}
}
#endif
