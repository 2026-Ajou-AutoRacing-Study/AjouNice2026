#pragma once

#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change::c2_reference_window
{
using Path = autoware_internal_planning_msgs::PathWithLaneId;
struct Result { Path path; bool applied=false; std::string reason; };
inline bool enabled_for_candidate(bool requested,bool normal,bool strict,bool continuous,
                                  bool partial,bool physical_endpoint) {
  return requested&&normal&&strict&&continuous&&partial&&physical_endpoint;
}

// The blending weight is C2 at its boundaries. This is NOT a certificate that
// a sampled path is globally C2, drivable, or collision-free. All native
// construction/footprint/velocity/RSS checks must still run afterwards.
inline Result apply(const Path & raw, const Path & native, double interval,
                    double length, bool enabled)
{
  const auto retain=[&](const char *why){return Result{native,false,why};};
  if(!enabled)return retain("disabled"); // Exact default-OFF behavior.
  try {
    if(raw.points.size()<3 || native.points.size()<3 || !std::isfinite(interval) ||
       interval<=0 || !std::isfinite(length) || length<=20.)return retain("invalid_or_short_input");
    std::vector<autoware_planning_msgs::PathPoint> points;
    for(const auto &p:raw.points) {
      const auto&q=p.point.pose.position;
      if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||
         !std::isfinite(p.point.longitudinal_velocity_mps))return retain("nonfinite_input");
      points.push_back(p.point);
    }
    const auto arc=autoware::motion_utils::calcSignedArcLengthPartialSum(points,0,points.size());
    for(size_t i=1;i<arc.size();++i)
      if(!std::isfinite(arc[i]) || arc[i]-arc[i-1]<=1e-6)return retain("nonmonotone_input");
    if(length>arc.back()+1e-8)return retain("incomplete_reference");
    // Match resamplePathWithSpline(keep_input_points=true,{0,length}) including
    // mandatory lane vertices and its exact stop replacement convention.
    std::vector<double> samples;
    const auto near=[&](double value){std::vector<size_t> idx;for(size_t i=0;i<samples.size();++i)
      if(std::abs(samples[i]-value)<.2)idx.push_back(i);return idx;};
    for(size_t i=0;i<raw.points.size();++i)
      if(!raw.points[i].lane_ids.empty()&&near(arc[i]).empty())samples.push_back(arc[i]);
    for(double s=0;s<length;s+=interval)if(near(s).empty())samples.push_back(s);
    if(near(length).empty())samples.push_back(length);
    if(const auto stop=autoware::motion_utils::calcDistanceToForwardStopPoint(points)) {
      const auto duplicates=near(*stop);
      if(duplicates.empty())samples.push_back(*stop);
      else {samples[duplicates.front()]=*stop;for(auto it=duplicates.rbegin();it!=duplicates.rend();++it)
        if(*it!=duplicates.front())samples.erase(samples.begin()+*it);}
    }
    std::sort(samples.begin(),samples.end());
    if(samples.size()!=native.points.size())return retain("sample_count_mismatch");
    const auto parity=autoware::motion_utils::resamplePath(raw,samples);
    if(parity.points!=native.points)return retain("native_sample_parity_mismatch");
    std::vector<double> x,y;
    for(const auto &p:raw.points){x.push_back(p.point.pose.position.x);y.push_back(p.point.pose.position.y);}
    const auto cx=autoware::interpolation::spline(arc,x,samples);
    const auto cy=autoware::interpolation::spline(arc,y,samples);
    // The mandatory requested station can be deduplicated against an input
    // vertex within 0.2 m. Protect that actual native sample, not length itself.
    const auto end_it=std::min_element(samples.begin(),samples.end(),
      [&](double a,double b){return std::abs(a-length)<std::abs(b-length);});
    const size_t end_index=std::distance(samples.begin(),end_it);
    const double end_s=*end_it;
    if(end_s<=10.)return retain("insufficient_protected_interval");
    const auto smooth=[](double u){u=std::clamp(u,0.,1.);return u*u*u*(10.+u*(-15.+6.*u));};
    std::vector<double> dx(samples.size(),0.),dy(samples.size(),0.);
    double maximum=0.;
    std::vector<bool> interior(samples.size(),false);
    for(size_t i=0;i<samples.size();++i) {
      if(i>=end_index)continue;
      const double weight=smooth((samples[i]-10.)/20.)*smooth((end_s-samples[i])/3.);
      if(weight<=0)continue;
      // A stop inside the deformation window has location semantics; do not
      // guess how to relocate it. Native path remains the atomic fallback.
      if(native.points[i].point.longitudinal_velocity_mps<=0)return retain("stop_or_reverse_in_window");
      dx[i]=weight*(cx[i]-native.points[i].point.pose.position.x);
      dy[i]=weight*(cy[i]-native.points[i].point.pose.position.y);
      if(!std::isfinite(dx[i])||!std::isfinite(dy[i]))
        return retain("reference_deformation_bound");
      maximum=std::max(maximum,std::hypot(dx[i],dy[i]));interior[i]=true;
    }
    if(maximum<=1e-10)return retain("no_change");
    // One common scale preserves the blending weight's boundary derivatives.
    // This experiment-specific bound is NOT a safety or drivability certificate.
    const double alpha=maximum>.25?.25/maximum:1.;
    auto result=native;
    for(size_t i=0;i<samples.size();++i)if(interior[i]) {
      result.points[i].point.pose.position.x+=alpha*dx[i];
      result.points[i].point.pose.position.y+=alpha*dy[i];
    }
    autoware::motion_utils::insertOrientation(result.points,true);
    // Preserve the full pose, not only XY, in prepare-side and suffix domains.
    for(size_t i=0;i<result.points.size();++i) {
      if(!interior[i])result.points[i]=native.points[i];
      const auto&q=result.points[i].point.pose.orientation;
      if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w))
        return retain("invalid_orientation");
    }
    return Result{std::move(result),true,"applied"};
  }catch(const std::exception&){return retain("interpolation_exception");}
}
} // namespace autoware::behavior_path_planner::utils::lane_change::c2_reference_window
