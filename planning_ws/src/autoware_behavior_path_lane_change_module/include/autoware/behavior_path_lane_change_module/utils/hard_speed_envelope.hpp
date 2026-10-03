#pragma once
#include "autoware/behavior_path_lane_change_module/utils/lateral_speed_cap.hpp"
#include "autoware/behavior_path_lane_change_module/utils/current_exit_certificate.hpp"
#include "autoware/behavior_path_lane_change_module/utils/prepared_speed_reference.hpp"
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
using HardEnvelopePath=autoware_internal_planning_msgs::PathWithLaneId;

// Inserting a collinear ego anchor does not change the physical polyline.
// Transport its existing curvature limits rather than recomputing a different
// three-point estimator with an artificially shortened incident segment.
inline std::vector<double> immutable_lateral_speed_caps(
  const HardEnvelopePath & source,const HardEnvelopePath & anchored,
  const size_t anchor,const double lateral_limit)
{
  if(source.points.size()<3||anchor>=anchored.points.size()||
      !std::isfinite(lateral_limit)||lateral_limit<=0)
    throw std::logic_error("invalid immutable curvature source");
  const bool inserted=anchored.points.size()==source.points.size()+1;
  if(!inserted&&anchored.points.size()!=source.points.size())
    throw std::logic_error("immutable curvature size mismatch");
  if(inserted&&(anchor==0||anchor>=source.points.size()))
    throw std::logic_error("immutable curvature endpoint insertion");
  // Native insertTargetPoint also updates the preceding point's quaternion.
  // Reproduce that exact operation, not a blanket orientation exemption.
  auto expected=source;
  if(inserted) {
    const auto index=autoware::motion_utils::insertTargetPoint(
      anchor-1,anchored.points[anchor].point.pose.position,expected.points);
    if(!index||*index!=anchor||expected.points.size()!=anchored.points.size())
      throw std::logic_error("immutable native insertion mismatch");
  }
  for(size_t i=0;i<anchored.points.size();++i) {
    if(inserted&&i==anchor) {
      auto actual=anchored.points[i];
      // The explicit rising-velocity interpolation may change only this field.
      actual.point.longitudinal_velocity_mps=expected.points[i].point.longitudinal_velocity_mps;
      if(!(actual==expected.points[i]))
        throw std::logic_error("immutable inserted metadata changed");
      continue;
    }
    if(!(anchored.points[i]==expected.points[i]))
      throw std::logic_error("immutable curvature source changed");
  }
  if(inserted) {
    const auto & a=source.points[anchor-1].point.pose.position;
    const auto & b=source.points[anchor].point.pose.position;
    const auto & p=anchored.points[anchor].point.pose.position;
    const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
    if(!std::isfinite(d2)||d2<1e-12)throw std::logic_error("invalid immutable anchor segment");
    const double u=((p.x-a.x)*dx+(p.y-a.y)*dy)/d2;
    if(!std::isfinite(u+p.x+p.y+p.z)||u<0||u>1||
        std::hypot(p.x-a.x-u*dx,p.y-a.y-u*dy)>1e-7||
        std::abs(p.z-a.z-u*(b.z-a.z))>1e-6)
      throw std::logic_error("noncollinear immutable curvature anchor");
  }
  std::vector<double> original,transported;
  for(size_t i=0;i<source.points.size();++i)
    original.push_back(lateral_speed_cap(source.points,i,lateral_limit));
  for(size_t i=0;i<anchored.points.size();++i) {
    if(inserted&&i==anchor)transported.push_back(std::min(original[anchor-1],original[anchor]));
    else transported.push_back(original[i-(inserted&&i>anchor?1:0)]);
  }
  return transported;
}

inline double hard_road_speed_at_internal_vertex(
  const HardEnvelopePath & ref,const geometry_msgs::Point & p)
{
  // Fallback ONLY for a unique INTERNAL closest vertex in the outward normal
  // cone of both incident segments. No extrapolation at a reference endpoint.
  size_t k=0;double d2=std::numeric_limits<double>::infinity();bool tie=false;
  for(size_t i=0;i<ref.points.size();++i){const auto&q=ref.points[i].point.pose.position;
    if(!std::isfinite(q.x+q.y+q.z))throw std::logic_error("invalid reference coordinate");
    const double d=(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y);
    if(d<d2){d2=d;k=i;tie=false;}else if(d==d2)tie=true;
  }
  if(tie||k==0||k+1>=ref.points.size()||d2>25.)
    throw std::logic_error("speed reference does not cover candidate");
  const auto fraction=[&](size_t i){const auto&a=ref.points[i].point.pose.position;
    const auto&b=ref.points[i+1].point.pose.position;double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
    if(!std::isfinite(l2)||l2<1e-8)throw std::logic_error("degenerate speed reference");
    return ((p.x-a.x)*dx+(p.y-a.y)*dy)/l2;};
  if(fraction(k-1)<1.||fraction(k)>0.)
    throw std::logic_error("speed reference does not cover candidate");
  // Verify that another segment's interior is not actually the nearer feature.
  // This is a certificate of this vertex, not a switch to another curve branch.
  for(size_t i=0;i+1<ref.points.size();++i){const auto&a=ref.points[i].point.pose.position;
    const auto&b=ref.points[i+1].point.pose.position;const double t=std::clamp(fraction(i),0.,1.);
    const double x=p.x-a.x-t*(b.x-a.x),y=p.y-a.y-t*(b.y-a.y);
    if(x*x+y*y<d2-1e-12)throw std::logic_error("uncertified internal closest vertex");
  }
  double cap=std::numeric_limits<double>::infinity();
  for(size_t i=k-1;i<=k+1;++i){const double v=ref.points[i].point.longitudinal_velocity_mps;
    if(!std::isfinite(v)||v<0)throw std::logic_error("invalid reference speed");cap=std::min(cap,v);}
  return cap;
}

inline double prepared_hard_road_speed_limit(
  const HardEnvelopePath & reference,const geometry_msgs::Point & point,
  const PreparedSpeedReference * prepared)
{
  if(reference.points.size()<2)throw std::logic_error("missing speed reference");
  if(!std::isfinite(point.x)||!std::isfinite(point.y))
    throw std::logic_error("invalid speed reference query");
  const size_t index=prepared&&prepared->matches(reference)?prepared->nearest_segment(point):
    autoware::motion_utils::findNearestSegmentIndex(reference.points,point);
  const auto speed_at=[&](size_t i) {
    const double speed=reference.points[i].point.longitudinal_velocity_mps;
    if(!std::isfinite(speed)||speed<0)throw std::logic_error("invalid reference speed");
    return speed;
  };
  double nearest=std::numeric_limits<double>::infinity();
  double result=std::numeric_limits<double>::infinity();
  // Nearest-vertex tie handling may choose the preceding segment at a bend.
  // Check only its direct neighbours, not unrelated global curve segments.
  const size_t first=index?index-1:0,last=std::min(index+1,reference.points.size()-2);
  for(size_t j=first;j<=last;++j) {
    const auto & a=reference.points[j].point.pose.position;
    const auto & b=reference.points[j+1].point.pose.position;
    const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
    if(!std::isfinite(length2)||length2<1e-8)throw std::logic_error("degenerate speed reference");
    const double fraction=((point.x-a.x)*dx+(point.y-a.y)*dy)/length2;
    const double lateral=std::hypot(point.x-a.x-fraction*dx,point.y-a.y-fraction*dy);
    if(!std::isfinite(fraction+lateral))throw std::logic_error("invalid speed reference projection");
    if(fraction<-.01||fraction>1.01||lateral>5.0)continue;
    const double bounded=std::clamp(fraction,0.0,1.0);
    const double distance=std::hypot(point.x-a.x-bounded*dx,point.y-a.y-bounded*dy);
    double cap=std::min(speed_at(j),speed_at(j+1));
    // At the shared vertex retain both incident segments' lower speed. This
    // numerical vertex comparison does not expand the coverage thresholds.
    if(std::abs(fraction)*std::sqrt(length2)<=1e-8 && j>0)
      cap=std::min(cap,speed_at(j-1));
    if(std::abs(1.0-fraction)*std::sqrt(length2)<=1e-8 && j+2<reference.points.size())
      cap=std::min(cap,speed_at(j+2));
    if(distance<nearest-1e-12){nearest=distance;result=cap;}
    else if(std::abs(distance-nearest)<=1e-12)result=std::min(result,cap);
  }
  if(!std::isfinite(result))return hard_road_speed_at_internal_vertex(reference,point);
  return result;
}

// Preserve the original public query and its legacy/degenerate behavior.
inline double hard_road_speed_limit(
  const HardEnvelopePath & reference,const geometry_msgs::Point & point)
{
  return prepared_hard_road_speed_limit(reference,point,nullptr);
}

// Shared B2/partial contract. Positive maneuver speeds are a previous plan,
// not road limits. PREPARE and physical LC may replace those positive plan
// values with the phase-appropriate fresh road envelope; all original zero
// points survive. Beyond LC end retain the original canonical tail ceiling.
inline std::vector<double> make_hard_speed_envelope_certified(
  const HardEnvelopePath & path,const HardEnvelopePath & current_reference,
  const HardEnvelopePath & target_reference,const HardEnvelopePath & upstream,
  const size_t begin,const size_t lc_end,const geometry_msgs::Point & ego,
  const double max_velocity,const double lateral_limit,const size_t lc_start,
  const bool phase_lateral_domain,const HardEnvelopePath * immutable_geometry,
  const size_t inserted_anchor,const geometry_msgs::Point * passed_lc_end,
  const CurrentExitCertificate * exit_certificate)
{
  const bool post_end_tail=begin>lc_end&&passed_lc_end;
  if(path.points.size()<3||(post_end_tail&&begin>=path.points.size()-1)||(begin>lc_end&&!post_end_tail)||lc_start>lc_end||lc_end>=path.points.size()||
     current_reference.points.size()<2||target_reference.points.size()<2||upstream.points.size()<2||
     !std::isfinite(max_velocity+lateral_limit)||max_velocity<=0||lateral_limit<=0)
    throw std::logic_error("invalid hard speed envelope inputs");
  if(post_end_tail) {
    const auto & end=path.points[lc_end].point.pose.position;
    const double passed=autoware::motion_utils::calcSignedArcLength(path.points,end,ego);
    if(!std::isfinite(passed+passed_lc_end->x+passed_lc_end->y)||passed<=0||
        std::hypot(end.x-passed_lc_end->x,end.y-passed_lc_end->y)>1e-7)
      throw std::logic_error("unverified passed physical LC end");
  }
  const auto immutable_caps=immutable_geometry?
    immutable_lateral_speed_caps(*immutable_geometry,path,inserted_anchor,lateral_limit):
    std::vector<double>{};
  const double remaining_lc=autoware::motion_utils::calcSignedArcLength(
    path.points,ego,path.points[lc_end].point.pose.position);
  const double tail_extent=post_end_tail?autoware::motion_utils::calcSignedArcLength(
    path.points,ego,path.points.back().point.pose.position):0.0;
  // Do not move or miss a NEW upstream stop that falls between candidate
  // vertices. It needs geometric insertion, so leave it to the old fallback.
  for(const auto & point:upstream.points) {
    const double speed=point.point.longitudinal_velocity_mps;
    if(!std::isfinite(speed)||speed<0)throw std::logic_error("invalid upstream speed");
    const double distance=autoware::motion_utils::calcSignedArcLength(
      upstream.points,ego,point.point.pose.position);
    if(speed==0.0&&distance>=0.0) {
      if(post_end_tail&&distance<=tail_extent) {
        const auto i=autoware::motion_utils::findNearestIndex(path.points,point.point.pose.position);
        const auto & matched=path.points[i].point;
        if(std::hypot(matched.pose.position.x-point.point.pose.position.x,
            matched.pose.position.y-point.point.pose.position.y)>1e-7||
            matched.longitudinal_velocity_mps!=0)
          throw std::logic_error("unrepresented new post-LC tail stop");
      } else if(!post_end_tail&&distance<=remaining_lc+10.0)
        throw std::logic_error("upstream stop present");
    }
  }
  // The immutable input references outlive these invocation-local views.
  // Only the native nearest-vertex projection copy is eliminated; no safety
  // result, speed cap, zero or cross-candidate state is cached.
  const PreparedSpeedReference prepared_current(current_reference),
    prepared_target(target_reference),prepared_upstream(upstream);
  std::vector<double> caps;
  caps.reserve(path.points.size()-begin);
  for(size_t i=begin;i<path.points.size();++i) {
    const auto & point=path.points[i].point;
    const double original=point.longitudinal_velocity_mps;
    if(!std::isfinite(original)||original<0)throw std::logic_error("invalid original speed");
    double cap=max_velocity;
    const auto reference_cap=[&](const HardEnvelopePath & reference,const char * name) {
      try {return prepared_hard_road_speed_limit(reference,point.pose.position,
        &reference==&target_reference?&prepared_target:
        &reference==&current_reference?&prepared_current:&prepared_upstream);}
      catch(const std::logic_error & error) {
        if(std::string(error.what()).find("speed reference does not cover candidate")!=0)throw;
        if(exit_certificate&&std::string(name)!="target"&&i>=lc_start&&i<=lc_end&&
           exit_certificate->permits(i,point.pose)) {
          // A coverage exception must not hide malformed data. Existing zero
          // scans and per-path original zeros remain independently binding.
          for(const auto & p:reference.points) {
            const auto & pos=p.point.pose.position;const double speed=p.point.longitudinal_velocity_mps;
            if(!std::isfinite(pos.x+pos.y+pos.z)||!std::isfinite(speed)||speed<0)
              throw std::logic_error("invalid certified speed reference");
            if(speed==0)throw std::logic_error("zero speed in uncovered certified reference");
          }
          return prepared_hard_road_speed_limit(target_reference,point.pose.position,&prepared_target);
        }
        std::ostringstream detail;detail.precision(17);
        detail<<error.what()<<" reference="<<name<<" i="<<i
          <<" point_xyz="<<point.pose.position.x<<","<<point.pose.position.y<<","<<point.pose.position.z
          <<" phase="<<(i<lc_start?"prepare":i<=lc_end?"lane_change":"tail")
          <<" lc_start="<<lc_start<<" lc_end="<<lc_end<<" begin="<<begin;
        throw std::logic_error(detail.str());
      }
    };
    // The target lane may only begin at the physical shift start. PREPARE
    // remains in the current lane; never extrapolate a future target segment
    // backwards to fabricate an ego road limit. Default start=0 is legacy.
    if(i>=lc_start)cap=std::min(cap,reference_cap(target_reference,"target"));
    if(i<=lc_end) {
      cap=std::min({cap,reference_cap(current_reference,"current"),
        reference_cap(upstream,"upstream")});
    } else {
      cap=std::min(cap,original);
      try {cap=std::min(cap,reference_cap(upstream,"upstream"));}
      catch(const std::logic_error &) {
        // Existing B2 contract: old-lane upstream may end before target tail.
        // Target coverage and untouched canonical tail ceiling remain binding.
      }
    }
    // LC raw-three-point curvature belongs to the shifted maneuver only.
    // Unchanged PREPARE/tail retain road/upstream/canonical speed contracts and
    // the normal downstream smoother's curvature model. Legacy callers keep
    // their previous all-points curvature behavior unless explicitly enabled.
    if(!phase_lateral_domain||(i>=lc_start&&i<=lc_end))
      cap=std::min(cap,immutable_geometry?immutable_caps[i]:lateral_speed_cap(path.points,i,lateral_limit));
    if(original==0.0)cap=0.0;
    caps.push_back(cap);
  }
  return caps;
}
// Preserve the original public inline signature and its exact default behavior.
inline std::vector<double> make_hard_speed_envelope(
  const HardEnvelopePath & path,const HardEnvelopePath & current_reference,
  const HardEnvelopePath & target_reference,const HardEnvelopePath & upstream,
  const size_t begin,const size_t lc_end,const geometry_msgs::Point & ego,
  const double max_velocity,const double lateral_limit,const size_t lc_start=0,
  const bool phase_lateral_domain=false,const HardEnvelopePath * immutable_geometry=nullptr,
  const size_t inserted_anchor=0,const geometry_msgs::Point * passed_lc_end=nullptr)
{
  return make_hard_speed_envelope_certified(path,current_reference,target_reference,upstream,
    begin,lc_end,ego,max_velocity,lateral_limit,lc_start,phase_lateral_domain,
    immutable_geometry,inserted_anchor,passed_lc_end,nullptr);
}
}
