#pragma once
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <boost/geometry.hpp>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
// Local value certificate. It authorizes only a missing old-lane speed-reference
// lookup, never collision safety, a stop removal, or a different path pose.
class CurrentExitCertificate
{
public:
  template<class MultiPolygon,class Vehicle>
  CurrentExitCertificate(
    const autoware_internal_planning_msgs::PathWithLaneId & path,
    const MultiPolygon & current,const MultiPolygon & target,const Vehicle & vehicle,
    const size_t begin=0,const size_t end=std::numeric_limits<size_t>::max())
  {
    if(path.points.empty()||begin>=path.points.size()||begin>end)return;
    const size_t last=std::min(end,path.points.size()-1);
    begin_=begin;
    const double xmin=vehicle.min_longitudinal_offset_m,xmax=vehicle.max_longitudinal_offset_m;
    const double ymin=vehicle.min_lateral_offset_m,ymax=vehicle.max_lateral_offset_m;
    if(!std::isfinite(xmin+xmax+ymin+ymax)||xmin>=xmax||ymin>=ymax||
       current.empty()||target.empty()||!boost::geometry::is_valid(current)||
       !boost::geometry::is_valid(target))return;
    for(size_t index=begin;index<=last;++index) {
      const auto & point=path.points[index];
      const auto & pose=point.point.pose;const auto & q=pose.orientation;
      poses_.push_back(pose);allowed_.push_back(false);
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(!std::isfinite(pose.position.x+pose.position.y+pose.position.z+norm)||
         std::abs(norm-1.0)>1e-6)continue;
      const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      const double c=std::cos(yaw),s=std::sin(yaw);
      typename MultiPolygon::value_type footprint;
      for(const auto & xy:std::vector<std::pair<double,double>>{
          {xmin,ymin},{xmax,ymin},{xmax,ymax},{xmin,ymax}})
        footprint.outer().emplace_back(pose.position.x+c*xy.first-s*xy.second,
          pose.position.y+s*xy.first+c*xy.second);
      boost::geometry::correct(footprint);
      if(!boost::geometry::within(footprint,target)||!boost::geometry::disjoint(footprint,current))continue;
      bool touches=false;
      for(const auto & polygon:target) {
        for(size_t i=1;i<polygon.outer().size();++i) {
          using Point=typename boost::geometry::point_type<decltype(footprint)>::type;
          boost::geometry::model::segment<Point> edge(polygon.outer()[i-1],polygon.outer()[i]);
          if(boost::geometry::intersects(footprint,edge))touches=true;
        }
        for(const auto & ring:polygon.inners())for(size_t i=1;i<ring.size();++i) {
          using Point=typename boost::geometry::point_type<decltype(footprint)>::type;
          boost::geometry::model::segment<Point> edge(ring[i-1],ring[i]);
          if(boost::geometry::intersects(footprint,edge))touches=true;
        }
      }
      allowed_.back()=!touches;
    }
  }
  bool permits(const size_t index,const geometry_msgs::Pose & pose) const
  {return index>=begin_&&index-begin_<poses_.size()&&allowed_[index-begin_]&&poses_[index-begin_]==pose;}
private:
  size_t begin_{0};
  std::vector<geometry_msgs::Pose> poses_;
  std::vector<bool> allowed_;
};
}
