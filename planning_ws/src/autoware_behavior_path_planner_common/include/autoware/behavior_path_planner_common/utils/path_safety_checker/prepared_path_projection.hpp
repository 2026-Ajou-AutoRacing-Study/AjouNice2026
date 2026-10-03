#pragma once
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <Eigen/Core>
#include <cmath>
namespace autoware::behavior_path_planner::utils::path_safety_checker {
// Call-local immutable geometry view. The referenced path must outlive this
// object and remain unchanged. No speed/model/RSS result is cached.
// Finite non-overlap certification only removes identity copy operations;
// all malformed/overlap references retain native fallback behavior.
class PreparedPathProjection {
 public:
 using Path=autoware_internal_planning_msgs::PathWithLaneId;
 explicit PreparedPathProjection(const Path & path):path_(path) {
   direct_=path.points.size()>=2;
   for(size_t i=0;i<path.points.size();++i) {
     const auto&p=path.points[i].point.pose.position;
     if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))direct_=false;
     if(i){const auto&q=path.points[i-1].point.pose.position;
       if(std::abs(p.x-q.x)<1e-8&&std::abs(p.y-q.y)<1e-8)direct_=false;}
   }
 }
 private:
 double offset(size_t i,const geometry_msgs::Point&p)const {
   if(!direct_)return autoware::motion_utils::calcLongitudinalOffsetToSegment(path_.points,i,p);
   const auto&a=path_.points.at(i).point.pose.position;const auto&b=path_.points.at(i+1).point.pose.position;
   const Eigen::Vector3d segment{b.x-a.x,b.y-a.y,0},target{p.x-a.x,p.y-a.y,0};
   return segment.dot(target)/segment.norm();
 }
 public:
 bool matches(const Path & path)const{return &path==&path_;}
 size_t nearest(const geometry_msgs::Point&p)const {
   if(!direct_)return autoware::motion_utils::findNearestSegmentIndex(path_.points,p);
   const auto i=autoware::motion_utils::findNearestIndex(path_.points,p);
   if(i==0)return 0;if(i==path_.points.size()-1)return i-1;
   return offset(i,p)<=0?i-1:i;
 }
 std::optional<geometry_msgs::Pose> offset_pose(const geometry_msgs::Point&p,double distance)const {
   if(!direct_)return autoware::motion_utils::calcLongitudinalOffsetPose(path_.points,p,distance);
   const auto i=nearest(p);const auto local=offset(i,p);
   return autoware::motion_utils::calcLongitudinalOffsetPose(path_.points,i,distance+local);
 }
 private:const Path&path_;bool direct_=false;
};
}
