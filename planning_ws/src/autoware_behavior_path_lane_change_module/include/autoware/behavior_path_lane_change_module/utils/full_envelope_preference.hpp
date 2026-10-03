#pragma once
#include "autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp"
#include <limits>
#include <optional>

namespace autoware::behavior_path_planner::lane_change::full_envelope {
inline double scoring_budget_ms(double native_ms) {
  return std::isfinite(native_ms)&&native_ms>0 ? std::min(35.0,.35*native_ms) : 0.0;
}
// Cooperative budget: setup and individual builder calls are not preemptible.

// Ranking evidence only, never a safety approval or a published speed profile.
struct Score { bool feasible=false; double travel_time=std::numeric_limits<double>::infinity(); };
inline double travel_time(const std::vector<double>&s,const std::vector<double>&v,double distance) {
  if(s.size()<2||s.size()!=v.size()||s.front()!=0||!std::isfinite(distance)||distance<=0||s.back()<distance)
    throw std::logic_error("score distance not covered");
  double time=0;
  for(size_t i=0;i+1<s.size()&&s[i]<distance;++i) {
    if(!std::isfinite(s[i]+s[i+1]+v[i]+v[i+1])||s[i+1]<=s[i]||v[i]<0||v[i+1]<0)
      throw std::logic_error("invalid score profile");
    if(v[i]<=1e-7)throw std::logic_error("score cannot pass existing stop");
    const double ds=std::min(distance,s[i+1])-s[i];
    const double end_v=std::sqrt(std::max(0.,v[i]*v[i]+(v[i+1]*v[i+1]-v[i]*v[i])*ds/(s[i+1]-s[i])));
    if(end_v+v[i]<=1e-7)throw std::logic_error("stopped score profile");
    time+=2*ds/(v[i]+end_v);
  }
  return time;
}
template<class Limit,class Budget>
Score evaluate(const std::vector<double>&arc,const std::vector<double>&caps,double v,double a,
 double horizon,double amin,double jmin,double jmax,double distance,Limit limit,Budget budget,
 const std::vector<double>&targets) {
  Score best;
  // All immutable input caps, zeros, calibrated acceleration and jerk limits
  // are checked by the same builder as the output stage. This is a bounded
  // heuristic, not a globally optimal full-maneuver planner.
  const double maximum=limit(v);
  if(!std::isfinite(maximum)||maximum<0||!std::isfinite(amin)||amin>=0||targets.size()>15)return best;
  for(double target:targets)if(!std::isfinite(target)||target<amin||target>maximum)return best;
  for(double target:targets) {
    if(!budget())break;
    try {
      const auto p=bounded_speed_grid::make(arc,caps,v,a,target,horizon,amin,jmin,jmax,limit);
      const double value=travel_time(arc,p.speeds,distance);
      if(std::isfinite(value)&&value<best.travel_time)best={true,value};
    } catch(const std::logic_error&) { /* Preserve the already-safe fallback. */ }
  }
  return best;
}
// Existing callers retain the exact original three-target ranking family.
template<class Limit,class Budget>
Score evaluate(const std::vector<double>&arc,const std::vector<double>&caps,double v,double a,
 double horizon,double amin,double jmin,double jmax,double distance,Limit limit,Budget budget) {
 const double maximum=limit(v);
 return evaluate(arc,caps,v,a,horizon,amin,jmin,jmax,distance,limit,budget,
   std::vector<double>{maximum,maximum*.5,0.0});
}
class Selection {
 std::optional<size_t> index_;double time_=std::numeric_limits<double>::infinity();
public:
 bool consider(size_t index,bool safe,const Score&score) {
   if(!safe||!score.feasible||!std::isfinite(score.travel_time)||score.travel_time<0)return false;
   if(!index_||score.travel_time+1e-3<time_){index_=index;time_=score.travel_time;return true;}return false;
 }
 bool has_value()const{return index_.has_value();}
 double time()const{return time_;}
 template<class T>bool append_selected(std::vector<T>&paths)const {
   if(!index_||*index_>=paths.size())return false;
   if(*index_+1!=paths.size()){const auto copy=paths[*index_];paths.push_back(copy);}return true;
 }
};
}
