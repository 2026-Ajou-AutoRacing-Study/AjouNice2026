#pragma once

#include "autoware/behavior_path_lane_change_module/structs/path.hpp"
#include <cmath>
#include <optional>
#include <stdexcept>

namespace autoware::behavior_path_planner::utils::lane_change::physical_endpoint {
using Path=autoware_internal_planning_msgs::PathWithLaneId;
struct Certificate { geometry_msgs::Pose pose; size_t reference_index,shifted_index,combined_index; };
inline bool same_position(const geometry_msgs::Point&a,const geometry_msgs::Point&b) {
  return a.x==b.x&&a.y==b.y&&a.z==b.z;
}
inline bool finite(const geometry_msgs::Point&p) {
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);
}
inline size_t unique_index(const Path&path,const geometry_msgs::Point&point) {
  size_t result=path.points.size(),count=0;
  for(size_t i=0;i<path.points.size();++i) {
    if(!finite(path.points[i].point.pose.position))throw std::logic_error("nonfinite endpoint path");
    if(same_position(path.points[i].point.pose.position,point)){result=i;++count;}
  }
  if(count!=1)throw std::logic_error("physical endpoint not uniquely preserved");
  return result;
}
// Single normal PathShifter maneuver, offset_back=false and base offset zero.
// shift_length is still indexed by the original reference after generate()
// removes overlapping/invalid first points. Never index shifted.path with it.
inline Certificate certify(const Path&reference,const ShiftedPath&shifted,
  const Path&combined,size_t start,size_t end) {
  if(start>=reference.points.size()||end>=reference.points.size()||
     end<=start+1||end+1>=reference.points.size()||shifted.shift_length.size()!=reference.points.size())
    throw std::logic_error("invalid physical endpoint reference domain");
  for(size_t i=end;i<shifted.shift_length.size();++i)
    if(!std::isfinite(shifted.shift_length[i])||shifted.shift_length[i]!=0.)
      throw std::logic_error("physical endpoint is not completed zero shift");
  const auto&p=reference.points[end].point.pose.position;
  if(!finite(p))throw std::logic_error("nonfinite physical endpoint");
  size_t next=end+1;
  while(next<reference.points.size()&&same_position(reference.points[next].point.pose.position,p))++next;
  if(next==reference.points.size()||!finite(reference.points[next].point.pose.position))
    throw std::logic_error("physical endpoint has no valid following line");
  const auto&following=reference.points[next].point.pose.position;
  if(std::hypot(following.x-p.x,following.y-p.y)<=0.)
    throw std::logic_error("physical endpoint following line is not planar");
  const size_t si=unique_index(shifted.path,p),ci=unique_index(combined,p);
  if(si+1>=shifted.path.points.size()||ci+1>=combined.points.size()||
     !same_position(shifted.path.points[si+1].point.pose.position,following)||
     !same_position(combined.points[ci+1].point.pose.position,following))
    throw std::logic_error("physical endpoint following segment changed");
  const auto &pose=combined.points[ci].point.pose;
  const auto&q=pose.orientation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(norm)||std::abs(norm-1.)>1e-6)
    throw std::logic_error("invalid physical endpoint orientation");
  return {pose,end,si,ci};
}
// Commit only after the complete certificate succeeds. Do not alter sampled
// phase metrics, original reference indices, geometry or published velocities.
inline Certificate apply(const Path&reference,LaneChangePath&candidate,
  const std::optional<size_t> phase_end_index=std::nullopt) {
  if(candidate.type==autoware::behavior_path_planner::lane_change::PathType::FrenetPlanner)
    throw std::logic_error("physical shifter endpoint does not apply to Frenet");
  const auto &line=candidate.info.shift_line;
  const auto result=certify(reference,candidate.shifted_path,candidate.path,line.start_idx,line.end_idx);
  if(phase_end_index&&*phase_end_index!=result.shifted_index)
    throw std::logic_error("physical endpoint differs from speed phase endpoint");
  candidate.info.lane_changing_end=result.pose;
  candidate.info.shift_line.end=result.pose;
  return result;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::physical_endpoint
