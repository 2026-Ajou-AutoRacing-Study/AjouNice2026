#pragma once
#include <boost/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner::lane_change::preferred_path_hazard
{
// Utility evidence only: this permits searching for a NORMAL candidate; it
// never establishes that candidate's collision safety or grants approval.
inline bool eligible(
  bool enabled,bool normal,bool approved,bool fresh,bool same_epoch,
  double prediction_time,double horizon,double current_distance,double current_gap,
  double ego_speed,double object_speed,double max_acc,double vehicle_length)
{
  if(!enabled||!normal||approved||!fresh||!same_epoch)return false;
  for(double v:{prediction_time,horizon,current_distance,current_gap,ego_speed,
                object_speed,max_acc,vehicle_length})if(!std::isfinite(v))return false;
  if(prediction_time<0||prediction_time>horizon||horizon<=0||current_distance<0||
     current_gap<=0||ego_speed<=0||object_speed<0||object_speed>=ego_speed||
     max_acc<0||vehicle_length<=0)return false;
  // Exactly the existing utility's observation range, not a longer horizon.
  return current_distance<=ego_speed*horizon+.5*max_acc*horizon*horizon+vehicle_length;
}

template<class Path,class Pose,class Polygon,class Vehicle>
bool intersects_input_path(const Path & path,const Pose & object_pose,
  const Polygon & object_polygon,const Vehicle & vehicle,double yaw_limit)
{
  if(path.points.size()<2||!std::isfinite(yaw_limit)||yaw_limit<=0||
     !boost::geometry::is_valid(object_polygon))return false;
  const auto valid=[](const auto & pose){
    const auto&p=pose.position;const auto&q=pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return std::isfinite(p.x+p.y+p.z+norm)&&std::abs(norm-1.)<=1e-3;
  };
  const auto yaw=[](const auto&q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
  const auto angle=[](double a){return std::atan2(std::sin(a),std::cos(a));};
  if(!valid(object_pose))return false;
  const double x=object_pose.position.x,y=object_pose.position.y,oyaw=yaw(object_pose.orientation);
  double best=std::numeric_limits<double>::infinity(),cx=0,cy=0,heading=0;
  bool extrapolated=false;
  for(size_t i=1;i<path.points.size();++i) {
    const auto&a=path.points[i-1].point.pose;const auto&b=path.points[i].point.pose;
    if(!valid(a)||!valid(b))return false;
    const double dx=b.position.x-a.position.x,dy=b.position.y-a.position.y,l2=dx*dx+dy*dy;
    if(!std::isfinite(l2))return false;
    if(l2<=1e-12)continue;
    const double f=((x-a.position.x)*dx+(y-a.position.y)*dy)/l2,t=std::clamp(f,0.,1.);
    const double px=a.position.x+t*dx,py=a.position.y+t*dy,d=std::hypot(x-px,y-py);
    if(d<best){best=d;cx=px;cy=py;heading=yaw(a.orientation)+t*angle(yaw(b.orientation)-yaw(a.orientation));
      extrapolated=(i==1&&f<0)||(i+1==path.points.size()&&f>1);}
  }
  if(!std::isfinite(best)||extrapolated||std::abs(angle(oyaw-heading))>yaw_limit)return false;
  const double xmin=vehicle.min_longitudinal_offset_m,xmax=vehicle.max_longitudinal_offset_m;
  const double ymin=vehicle.min_lateral_offset_m,ymax=vehicle.max_lateral_offset_m;
  if(!std::isfinite(xmin+xmax+ymin+ymax)||xmin>=xmax||ymin>=ymax)return false;
  Polygon ego;
  const double c=std::cos(heading),s=std::sin(heading);
  for(const auto & xy:{std::pair<double,double>{xmin,ymin},{xmax,ymin},{xmax,ymax},{xmin,ymax}})
    ego.outer().emplace_back(cx+c*xy.first-s*xy.second,cy+s*xy.first+c*xy.second);
  boost::geometry::correct(ego);
  return boost::geometry::is_valid(ego)&&boost::geometry::intersects(ego,object_polygon);
}
}  // namespace autoware::behavior_path_planner::lane_change::preferred_path_hazard
