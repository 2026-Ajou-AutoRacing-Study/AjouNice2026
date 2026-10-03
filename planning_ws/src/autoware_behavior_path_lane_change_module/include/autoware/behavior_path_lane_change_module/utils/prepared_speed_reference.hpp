#pragma once
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <Eigen/Core>
#include <cmath>
namespace autoware::behavior_path_planner::utils::lane_change {
// Invocation-local view of an immutable reference. No cached decisions,
// global state, point removal, index remapping or nearest-segment substitution.
class PreparedSpeedReference {
public:
  using Path=autoware_internal_planning_msgs::PathWithLaneId;
  explicit PreparedSpeedReference(const Path & reference):reference_(reference) {
    direct_=reference.points.size()>=2;
    for(size_t i=0;i<reference.points.size();++i) {
      const auto & p=reference.points[i].point.pose.position;
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))direct_=false;
      if(i) {
        const auto & previous=reference.points[i-1].point.pose.position;
        // EXACT removeOverlapPoints epsilon; equality is NOT an overlap.
        if(std::abs(previous.x-p.x)<1e-8&&std::abs(previous.y-p.y)<1e-8)direct_=false;
      }
    }
  }
  bool direct()const{return direct_;}
  bool matches(const Path & path)const{return &path==&reference_;}
  size_t nearest_segment(const geometry_msgs::Point & point)const {
    if(!direct_)return autoware::motion_utils::findNearestSegmentIndex(reference_.points,point);
    const auto nearest=autoware::motion_utils::findNearestIndex(reference_.points,point);
    if(nearest==0)return 0;
    if(nearest==reference_.points.size()-1)return nearest-1;
    const auto & a=reference_.points[nearest].point.pose.position;
    const auto & b=reference_.points[nearest+1].point.pose.position;
    // Same arithmetic/order as native calcLongitudinalOffsetToSegment. Its
    // removeOverlapPoints is the identity on a certified direct reference.
    const Eigen::Vector3d segment{b.x-a.x,b.y-a.y,0};
    const Eigen::Vector3d target{point.x-a.x,point.y-a.y,0};
    const double offset=segment.dot(target)/segment.norm();
    return offset<=0?nearest-1:nearest;
  }
private:
  const Path & reference_;
  bool direct_=false;
};
}
