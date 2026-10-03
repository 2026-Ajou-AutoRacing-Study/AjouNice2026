#pragma once
#include <cmath>
#include <cstddef>

namespace autoware::behavior_path_planner
{
struct CoherentEgoPollDiagnostic {
  const char * reason{"missing_input"};
  size_t attempts{0};
  double acquired_at{0.0};
};
// Keep the newest observed object packet pinned: never hide new actors by
// substituting an older coherent object packet. Real ego messages only.
// Finite attempts, no sleep, no promise that transport has delivered a sample.
template<class OdomSubscriber,class AccelSubscriber,class OdomPtr,class AccelPtr,
         class ObjectsPtr,class Now,class Budget>
bool coherent_ego_poll(OdomSubscriber & os,AccelSubscriber & as,OdomPtr & odom,
  AccelPtr & accel,const ObjectsPtr & objects,Now now,Budget budget,
  const size_t max_attempts=4, CoherentEgoPollDiagnostic * diagnostic=nullptr)
{
  if(diagnostic)*diagnostic=CoherentEgoPollDiagnostic{};
  if(!odom||!accel||!objects)return false;
  auto skew=[&](){return (odom->header.stamp-objects->header.stamp).toSec();};
  // Re-poll only the specific future-object failure; other input contracts are
  // not repaired by changing their timestamps or widening acceptance limits.
  for(size_t i=0;skew()<-.010 && i<max_attempts && budget();++i) {
    if(diagnostic)++diagnostic->attempts;
    if(auto m=os.take_data())
      if(m->header.stamp>odom->header.stamp)odom=m;
    if(auto m=as.take_data())
      if(m->header.stamp>accel->header.stamp)accel=m;
  }
  const auto acquired=now();
  const auto fresh=[&](const auto & stamp) {
    const double age=(acquired-stamp).toSec();
    return std::isfinite(age)&&age>=-.010&&age<=.5;
  };
  const double age=skew();
  const char * reason = !std::isfinite(age) ? "nonfinite_skew" :
    age<-.010 ? "future_objects" : age>.5 ? "stale_objects_relative_to_ego" :
    odom->header.frame_id!=objects->header.frame_id ? "frame_mismatch" :
    !fresh(odom->header.stamp) ? "odom_wall_age" :
    !fresh(accel->header.stamp) ? "accel_wall_age" :
    !fresh(objects->header.stamp) ? "objects_wall_age" : nullptr;
  if(diagnostic) {
    diagnostic->reason=reason ? reason : "ok";
    diagnostic->acquired_at=acquired.toSec();
  }
  return reason==nullptr;
}
}  // namespace autoware::behavior_path_planner
