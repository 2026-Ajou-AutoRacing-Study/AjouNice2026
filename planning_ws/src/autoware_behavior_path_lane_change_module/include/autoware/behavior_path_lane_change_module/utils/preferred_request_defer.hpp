#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <cstdint>
#include <set>
#include <map>
#include <string>
namespace autoware::behavior_path_planner::lane_change::preferred_request_defer {
struct RoutePrefix {
 std::vector<int64_t> lanes,following;
 bool valid=false;
};
// Follow only a unique route-approved successor. Never choose a branch at a
// fork, assume an absent continuation, or claim a mission checkpoint passed.
template<class Allowed,class Following>
RoutePrefix unique_route_prefix(int64_t start,Allowed allowed,Following following) {
 RoutePrefix out;std::set<int64_t> seen;
 for(size_t i=0;i<40;++i) {
   if(!allowed(start)||!seen.insert(start).second)return {};
   out.lanes.push_back(start);
   auto next=following(start);
   std::sort(next.begin(),next.end());
   if(std::adjacent_find(next.begin(),next.end())!=next.end())return {};
   for(auto id:next)if(!allowed(id))return {};
   if(next.size()>1){out.following=std::move(next);out.valid=true;return out;}
   if(next.empty())return {}; // Unknown/end is not an optional continuation proof.
   start=next.front();
 }
 return {};
}
inline bool same_route_alternatives(const RoutePrefix & a,const RoutePrefix & b) {
 return a.valid&&b.valid&&!a.lanes.empty()&&!b.lanes.empty()&&
   a.following.size()>1&&a.following==b.following;
}
struct OrderedRouteSegment {int64_t preferred;std::vector<int64_t> primitives;};
struct PreferredRoutePrefix {
 RoutePrefix prefix;
 size_t first_segment=0,last_segment=0;
 std::vector<std::pair<size_t,int64_t>> resolved_forks;
 bool terminal=false;
};
// A global preferred-ID set is insufficient: only the actual immediately next
// route segment may resolve a fork. The callback verifies same-direction edges.
template<class Following,class Compatible>
PreferredRoutePrefix preferred_route_prefix(int64_t start,
 const std::vector<OrderedRouteSegment>& route,Following following,Compatible compatible) {
 PreferredRoutePrefix out;std::map<int64_t,size_t> index;
 for(size_t k=0;k<route.size();++k) {
   if(route[k].primitives.empty()||std::count(route[k].primitives.begin(),
      route[k].primitives.end(),route[k].preferred)!=1)return {};
   for(auto id:route[k].primitives)if(!index.emplace(id,k).second)return {};
 }
 if(!index.count(start))return {};
 out.first_segment=index.at(start);std::set<int64_t> seen;
 for(size_t n=0;n<60;++n) {
   if(!index.count(start)||!seen.insert(start).second)return {};
   const size_t k=index.at(start);out.last_segment=k;out.prefix.lanes.push_back(start);
   auto next=following(start);std::sort(next.begin(),next.end());
   if(std::adjacent_find(next.begin(),next.end())!=next.end())return {};
   for(auto id:next)if(!index.count(id)||index.at(id)!=k+1)return {};
   if(next.empty()) {out.terminal=true;out.prefix.valid=true;return out;}
   if(k+1>=route.size())return {};
   int64_t chosen=next.front();
   if(next.size()>1) {
     chosen=route[k+1].preferred;
     if(std::find(next.begin(),next.end(),chosen)==next.end()) {
       out.prefix.following=std::move(next);out.prefix.valid=true;return out;
     }
     out.resolved_forks.emplace_back(k+1,chosen);
   }
   if(!compatible(start,chosen))return {};
   start=chosen;
 }
 return {};
}
inline bool same_preferred_continuation(const PreferredRoutePrefix&a,const PreferredRoutePrefix&b) {
 if(!a.prefix.valid||!b.prefix.valid||a.prefix.lanes.empty()||b.prefix.lanes.empty()||
    a.first_segment!=b.first_segment||a.last_segment!=b.last_segment||
    a.resolved_forks!=b.resolved_forks||a.terminal!=b.terminal)return false;
 if(a.terminal)return a.prefix.lanes.back()==b.prefix.lanes.back();
 return a.prefix.following.size()>1&&a.prefix.following==b.prefix.following;
}
struct ComposedRoutePrefix {
 RoutePrefix current,target;
 bool valid=false,extended=false;
};
inline ComposedRoutePrefix compose_route_prefixes(const RoutePrefix & current,
 const RoutePrefix & target,const PreferredRoutePrefix & extended_current,
 const PreferredRoutePrefix & extended_target) {
 // Extension failure must not revoke the existing legal first-fork proof.
 // Conversely, an extension cannot create permission without that base proof.
 if(!same_route_alternatives(current,target))return {};
 ComposedRoutePrefix out{current,target,true,false};
 const auto preserves=[](const auto & base,const auto & extra) {
   return extra.size()>=base.size()&&std::equal(base.begin(),base.end(),extra.begin());
 };
 if(same_preferred_continuation(extended_current,extended_target)&&
    preserves(current.lanes,extended_current.prefix.lanes)&&
    preserves(target.lanes,extended_target.prefix.lanes)) {
   out.current=extended_current.prefix;out.target=extended_target.prefix;out.extended=true;
 }
 return out;
}
// Reconsider the request each native cycle. H is observed prediction coverage,
// not a claimed time to finish a future lane change. No extrapolation past it.
inline double prediction_horizon(double age,const std::vector<std::vector<double>> & paths) {
 const double invalid=std::numeric_limits<double>::quiet_NaN();
 if(!std::isfinite(age)||age<0||age>.5||paths.empty())return invalid;
 double horizon=3.;
 for(const auto & path:paths) {
   if(path.size()<2||path.front()>age)return invalid;
   double previous=-1.;
   for(double time:path) {
     if(!std::isfinite(time)||time<0||time<=previous)return invalid;
     previous=time;
   }
   horizon=std::min(horizon,path.back()-age);
 }
 return horizon>0?horizon:invalid;
}
inline double acceleration_upper_bound(double observed,double requested,double calibrated) {
 if(!std::isfinite(observed)||!std::isfinite(requested)||!std::isfinite(calibrated)||
    requested<0||calibrated<0||calibrated>requested)
   return std::numeric_limits<double>::quiet_NaN();
 // The installed map is positive and nonincreasing. Its current-speed value
 // bounds the future velocity barrier, even after a temporary slowdown. Keep
 // stronger measured acceleration in both travel and jerk-limited STOP reserve.
 return std::max({0.,observed,calibrated});
}
// Utility lookahead only: perpendicular distance to the finite centerline,
// independent of longitudinal vertex spacing. Never extrapolate past an end.
inline double lateral_shift(
 const std::vector<std::pair<double,double>> & path,double x,double y) {
 const double invalid=std::numeric_limits<double>::quiet_NaN();
 if(path.size()<2||!std::isfinite(x+y))return invalid;
 for(const auto & p:path)if(!std::isfinite(p.first+p.second))return invalid;
 double best=std::numeric_limits<double>::infinity(),fraction=0;
 for(size_t i=1;i<path.size();++i) {
   const double dx=path[i].first-path[i-1].first,dy=path[i].second-path[i-1].second;
   const double length2=dx*dx+dy*dy;
   if(length2<=1e-12)continue;
   const double f=((x-path[i-1].first)*dx+(y-path[i-1].second)*dy)/length2;
   const double bounded=std::clamp(f,0.,1.);
   const double distance=std::hypot(x-path[i-1].first-bounded*dx,y-path[i-1].second-bounded*dy);
   if(distance<best){best=distance;fraction=f;}
 }
 if(!std::isfinite(best)||fraction<0||fraction>1)return invalid;
 return best;
}
struct Context {
 bool enabled=false,normal=false,approved=true,fresh=false,topology=false,near_terminal=true;
 bool target_lead=false,current_lead=false;
 double speed=0,target_speed=0,target_gap=0,current_speed=0,current_gap=0;
 double target_rss_gap=0,current_rss_gap=0;
 // Future STOP reserve, not optional multi-LC preference distance or a proof
 // of future LC completion. Native valid-distance/near-terminal gates stay.
 double horizon=0,remaining=0,future_reserve=0,max_acc=0;
};
// Utility needs an actually ordered observation pair, unlike an extrapolated
// motion hypothesis. Never retimestamp or expand this interval during retry.
inline bool request_epoch_valid(double now,double odom,double objects,double acceleration,
 const std::string & odom_frame,const std::string & object_frame,const std::string & route_frame) {
 return odom>0&&objects>0&&std::isfinite(now)&&
   now-odom>=0&&now-odom<=.5&&odom-objects>=0&&odom-objects<=.5&&
   now-acceleration>=0&&now-acceleration<=.5&&
   odom_frame==object_frame&&odom_frame==route_frame;
}
// Suppresses only an initial preference request, never approves lane-follow or LC.
inline bool evaluate(const Context & c) {
 if(!c.enabled||!c.normal||c.approved||!c.fresh||!c.topology||c.near_terminal||!c.target_lead)return false;
 for(double x:{c.speed,c.target_speed,c.target_gap,c.horizon,c.remaining,c.future_reserve,c.max_acc,c.target_rss_gap,c.current_rss_gap})
   if(!std::isfinite(x))return false;
 if(c.speed<=0||c.target_speed<0||c.target_gap<=0||c.horizon<=0||c.future_reserve<0||c.max_acc<0||c.target_rss_gap<0||c.current_rss_gap<0)return false;
 if(c.remaining<=c.future_reserve+c.speed*c.horizon+.5*c.max_acc*c.horizon*c.horizon)return false;
 // Compare both leads against the same possible accelerating travel. This is
 // optional-request utility, NOT a future RSS/LC safety approval or a promise
 // that the current lane can actually achieve the acceleration upper bound.
 const double accelerating_travel=.5*c.max_acc*c.horizon*c.horizon;
 const auto closure=[&](double lead) {
   return std::max(0.,(c.speed-lead)*c.horizon+accelerating_travel);
 };
 const double target_closure=closure(c.target_speed);
 if(!std::isfinite(accelerating_travel)||!std::isfinite(target_closure)||
    target_closure<=0||c.target_gap>=target_closure+c.target_rss_gap)return false;
 if(c.current_lead) {
   if(!std::isfinite(c.current_speed+c.current_gap)||c.current_speed<0||c.current_gap<=0)return false;
   // A slower but far-away current leader can be preferable to a close target
   // leader. Distance plus the existing RSS requirement, not speed ordering,
   // decides whether postponing this optional request has a closing hazard.
   if(c.current_gap<c.target_gap)return false;
   const double current_closure=closure(c.current_speed);
   if(!std::isfinite(current_closure)||c.current_gap<=current_closure+c.current_rss_gap)return false;
 }
 return true;
}
}
