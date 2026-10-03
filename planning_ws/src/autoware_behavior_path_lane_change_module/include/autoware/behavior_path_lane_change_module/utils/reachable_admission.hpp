#pragma once
#include "autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp"
#include "autoware/behavior_path_lane_change_module/utils/rss_positive_grid.hpp"
#include <array>
#include <string>

namespace autoware::behavior_path_planner::lane_change::reachable_admission {
struct Result { bool accepted=false; unsigned attempted=0; double target=0; std::string reason="no_reachable_witness"; };
template<class Candidate,class Finalize,class Admit>
bool finalize_then_admit(Candidate&candidate,Finalize finalize,Admit admit) {
 finalize(candidate);return admit(candidate);
}
// Enabled: every check consumes the same finalized candidate. Disabled: keep
// the legacy native-before-finalization order. Exceptions propagate to the
// generator's existing reject handler; admission never rescues native failure.
template<class Candidate,class Finalize,class Native,class Admit>
bool finalize_and_check(bool final_checks,Candidate&candidate,Finalize finalize,
 Native native,Admit admit) {
 if(final_checks) {
   finalize(candidate);
   return native(candidate)&&admit(candidate);
 }
 if(!native(candidate))return false;
 return finalize_then_admit(candidate,finalize,admit);
}
// Additional admission proof only. Never changes the native candidate and never
// rescues a failed native check. Both finite target families are non-exhaustive.
template<class Limit,class Check,class Budget>
Result evaluate(bool native_safe,const std::vector<double>&arc,const std::vector<double>&caps,
 double v,double a,double horizon,double amin,double jmin,double jmax,
 Limit limit,Check check,Budget budget,bool positive_grid=false) {
 Result result;
 if(!native_safe){result.reason="native_unsafe";return result;}
 const double maximum=limit(v);
 const auto targets=positive_grid ? rss_positive_grid::augment(
   bounded_speed_grid::acceleration_targets(maximum,amin),maximum,true) :
   std::vector<double>{maximum,maximum*.5,0.};
 for(double target:targets) {
   if(!budget()){result.reason="admission_budget_exhausted";return result;}
   ++result.attempted;
   try {
     const auto proposal=bounded_speed_grid::make(arc,caps,v,a,target,horizon,amin,jmin,jmax,limit);
     if(!budget()){result.reason="admission_budget_exhausted";return result;}
     if(check(proposal,result.reason)) {
       if(!budget()){result.reason="admission_budget_exhausted";return result;}
       result.accepted=true;result.target=target;result.reason="reachable_witness";return result;
     }
   }catch(const std::logic_error&e){result.reason=e.what();}
 }
 return result;
}
}
