#include "autoware/behavior_path_lane_change_module/utils/world_candidate.hpp"
#include <autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp>
#include <autoware/behavior_path_lane_change_module/utils/maneuver_safety.hpp>
#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace bpp=autoware::behavior_path_planner;
namespace lc=bpp::utils::lane_change;
namespace autoware::behavior_path_planner::utils::lane_change::world_candidate::detail {
bpp::PathWithLaneId construct_geometry(
 const bpp::PathWithLaneId &prep,const bpp::PathWithLaneId &ref,
 const geometry_msgs::Pose &end,size_t &start_index,size_t &end_index) {
 if(prep.points.size()<3||ref.points.size()<3)throw std::logic_error("insufficient endpoint context");
 const auto valid_pose=[](const geometry_msgs::Pose&p){const auto&q=p.orientation;
   const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
   return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&
     std::isfinite(norm)&&std::abs(norm-1.)<=1e-6;};
 if(!valid_pose(end))throw std::logic_error("invalid world endpoint pose");
 for(const auto&p:prep.points)if(!valid_pose(p.point.pose)||
   !std::isfinite(p.point.longitudinal_velocity_mps)||p.point.longitudinal_velocity_mps<=0)
   throw std::logic_error("invalid or stopped world prepare");
 for(const auto&p:ref.points)if(!valid_pose(p.point.pose)||
   !std::isfinite(p.point.longitudinal_velocity_mps)||p.point.longitudinal_velocity_mps<0)
   throw std::logic_error("invalid world reference");
 const auto xy=[](const auto&p){return Eigen::Vector2d(p.x,p.y);};
 const auto curvature=[&](const auto&a,const auto&b,const auto&c){const Eigen::Vector2d x=xy(b)-xy(a),y=xy(c)-xy(b),z=xy(c)-xy(a);
  const double den=x.norm()*y.norm()*z.norm();if(!std::isfinite(den)||den<1e-12)throw std::logic_error("degenerate endpoint curvature");return 2*(x.x()*y.y()-x.y()*y.x())/den;};
 double best=1e99,mu=0;size_t j=0;
 for(size_t k=0;k+1<ref.points.size();++k){const Eigen::Vector2d a=xy(ref.points[k].point.pose.position),b=xy(ref.points[k+1].point.pose.position),d=b-a;
  if(d.squaredNorm()<1e-12)continue;const double f=std::clamp((xy(end.position)-a).dot(d)/d.squaredNorm(),0.,1.);
  const double distance=(xy(end.position)-a-f*d).norm();if(distance<best){best=distance;mu=f;j=k;}}
 if(best>1e-3||j+2>=ref.points.size())throw std::logic_error("canonical endpoint coverage");
 // At an exact reference vertex use the first strictly following segment,
 // avoiding a duplicate endpoint in the curvature stencil and output suffix.
 if((xy(ref.points[j+1].point.pose.position)-xy(end.position)).norm()<1e-8){++j;mu=0.;}
 if(j+2>=ref.points.size())throw std::logic_error("canonical endpoint lacks following tangent");
 const auto &p0=prep.points.back().point.pose.position;
 const auto &pm=prep.points[prep.points.size()-2].point.pose.position;
 const double yaw0=std::atan2(p0.y-pm.y,p0.x-pm.x),yaw1=tf2::getYaw(end.orientation);
 const double k0=curvature(prep.points[prep.points.size()-3].point.pose.position,pm,p0);
 const double k1=curvature(end.position,ref.points[j+1].point.pose.position,ref.points[j+2].point.pose.position);
 const double length=(xy(end.position)-xy(p0)).norm();
 if(!std::isfinite(length)||length<1e-6||!std::isfinite(yaw0+yaw1+k0+k1)||
    length/.5>100000.)throw std::logic_error("invalid or unsupported world curve length");
 Eigen::Matrix<double,6,2> c;const Eigen::Vector2d t0(std::cos(yaw0),std::sin(yaw0)),t1(std::cos(yaw1),std::sin(yaw1));
 c.row(0)=xy(p0).transpose();c.row(1)=(length*t0).transpose();c.row(2)=(.5*length*length*k0*Eigen::Vector2d(-t0.y(),t0.x())).transpose();
 Eigen::Matrix3d m;m<<1,1,1,3,4,5,6,12,20;Eigen::Matrix<double,3,2> rhs;
 rhs.row(0)=xy(end.position).transpose()-c.row(0)-c.row(1)-c.row(2);
 rhs.row(1)=(length*t1).transpose()-c.row(1)-2*c.row(2);
 rhs.row(2)=(length*length*k1*Eigen::Vector2d(-t1.y(),t1.x())).transpose()-2*c.row(2);
 c.bottomRows<3>()=m.fullPivLu().solve(rhs);
 if(!c.allFinite()||!rhs.allFinite())throw std::logic_error("nonfinite world coefficients");
 const auto arc=lc::safety_arc_lengths(ref);const double terminal_s=arc[j]+mu*(arc[j+1]-arc[j]);
 for(size_t k=1;k<arc.size();++k)if(!std::isfinite(arc[k])||arc[k]<=arc[k-1])
   throw std::logic_error("degenerate world reference station");
 for(size_t k=0;k<arc.size();++k)if(arc[k]<=terminal_s&&ref.points[k].point.longitudinal_velocity_mps==0)
   throw std::logic_error("world body stop unsupported");
 auto out=prep;start_index=out.points.size()-1;
 tf2::Quaternion q;q.setRPY(0,0,yaw0);out.points.back().point.pose.orientation=tf2::toMsg(q);
 const size_t n=std::max<size_t>(2,std::ceil(length/.5));
 std::vector<Eigen::Vector2d> points,derivatives;std::vector<double> body_s{0.};
 for(size_t i=0;i<=n;++i){const double u=double(i)/n;Eigen::Vector2d p=Eigen::Vector2d::Zero(),d=p;
  for(size_t k=0;k<6;++k){p+=c.row(k).transpose()*std::pow(u,k);if(k)d+=double(k)*c.row(k).transpose()*std::pow(u,k-1);}
  if(!p.allFinite()||!d.allFinite()||d.norm()<1e-6)throw std::logic_error("nonregular world curve");
  if(i)body_s.push_back(body_s.back()+(p-points.back()).norm());points.push_back(p);derivatives.push_back(d);}
 for(size_t i=1;i<=n;++i){const double s=body_s[i]/body_s.back()*terminal_s;
  auto upper=std::upper_bound(arc.begin(),arc.end(),s);const size_t k=std::min(ref.points.size()-2,size_t(std::max<ptrdiff_t>(0,upper-arc.begin()-1)));
  auto point=ref.points[k];point.point.pose.position.x=points[i].x();point.point.pose.position.y=points[i].y();
  const double f=(s-arc[k])/(arc[k+1]-arc[k]);point.point.pose.position.z=ref.points[k].point.pose.position.z+f*(ref.points[k+1].point.pose.position.z-ref.points[k].point.pose.position.z);
  q.setRPY(0,0,std::atan2(derivatives[i].y(),derivatives[i].x()));point.point.pose.orientation=tf2::toMsg(q);
  if(i==n)point.point.pose=end;out.points.push_back(point);}
 end_index=out.points.size()-1;out.points.insert(out.points.end(),ref.points.begin()+j+1,ref.points.end());return out;
}

}
namespace autoware::behavior_path_planner::utils::lane_change::world_candidate {
bool enabled(bool requested,bool normal,bool strict,bool continuous,bool partial,
             bool reachable,bool physical_endpoint) {
 return requested&&normal&&strict&&continuous&&partial&&reachable&&physical_endpoint;
}
Result make(const Input &in,const Services &svc) {
 const auto started=ros::WallTime::now();
 const auto budget=[&](){if(in.deadline.isZero()||ros::WallTime::now()>=in.deadline)
   throw std::logic_error("world candidate budget exhausted");};
 budget();
 if(!in.common||!in.common->is_data_available()||!in.common->current_acceleration||
    in.common->lc_type!=bpp::lane_change::ModuleType::NORMAL||
    !svc.native_reference||!svc.anchor||!svc.hard_caps||
    in.prepare.points.size()<3||!std::isfinite(in.horizon)||in.horizon<2.5||in.horizon>3.01)
   throw std::logic_error("invalid world context");
 const auto &common=*in.common;
 const auto finite_phase=[](const LaneChangePhaseMetrics&p){return
   std::isfinite(p.duration)&&std::isfinite(p.length)&&std::isfinite(p.velocity)&&
   std::isfinite(p.sampled_lon_accel)&&std::isfinite(p.actual_lon_accel)&&std::isfinite(p.lat_accel)&&
   p.duration>=0&&p.length>=0&&p.velocity>=0;};
 if(common.lanes_ptr->current.empty()||common.lanes_ptr->target.empty()||
    !finite_phase(in.prepare_metric)||!finite_phase(in.changing_metric)||in.changing_metric.length<=0||
    !std::isfinite(in.shift_length)||!std::isfinite(common.transient_data.next_dist_buffer.min)||
    !std::isfinite(common.transient_data.dist_to_terminal_end)||
    common.transient_data.next_dist_buffer.min<0||common.transient_data.dist_to_terminal_end<=0)
   throw std::logic_error("invalid world native phase or route extent");
 const auto reference=svc.native_reference(in);
 if(reference.points.size()<3)throw std::logic_error("empty native reference");
 const auto s0=autoware::experimental::lanelet2_utils::get_arc_coordinates(
     common.lanes_ptr->target,in.prepare.points.back().point.pose).length;
 const auto end=common.route_handler_ptr->get_pose_from_2d_arc_length(
     common.lanes_ptr->target,s0+in.changing_metric.length);
 if(!std::isfinite(in.prepare_metric.length+in.changing_metric.length)||
    in.prepare_metric.length+in.changing_metric.length+common.transient_data.next_dist_buffer.min>
      common.transient_data.dist_to_terminal_end)
   throw std::logic_error("native requested length infeasible");
 size_t start_index=0,end_index=0;
 auto geometry=detail::construct_geometry(in.prepare,reference,end,start_index,end_index);
 const auto ref_arc=lc::safety_arc_lengths(reference);
 struct Projection {size_t owner;double station,offset;};
 const auto project=[&](const geometry_msgs::Point&q){
   double best=std::numeric_limits<double>::infinity();Projection out{};
   for(size_t j=1;j<reference.points.size();++j){
     const auto&a=reference.points[j-1].point.pose.position;const auto&b=reference.points[j].point.pose.position;
     const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;if(l2<1e-12)continue;
     const double u=std::clamp(((q.x-a.x)*dx+(q.y-a.y)*dy)/l2,0.,1.);
     const double ex=q.x-a.x-u*dx,ey=q.y-a.y-u*dy;
     if(ex*ex+ey*ey<best){best=ex*ex+ey*ey;out={u<=.5?j-1:j,ref_arc[j-1]+u*std::sqrt(l2),
       (dx*(q.y-a.y)-dy*(q.x-a.x))/std::sqrt(l2)};}
   }
   if(!std::isfinite(best))throw std::logic_error("invalid world reference projection");return out;
 };
 const double end_station=project(end.position).station;
 // New body may not skip a stop. Initial implementation rejects unsupported
 // internal stops; suffix is exact and the caller additionally checks upstream.
 for(size_t j=0;j<reference.points.size();++j){
   const double v=reference.points[j].point.longitudinal_velocity_mps;
   if(!std::isfinite(v)||v<0)throw std::logic_error("invalid native reference speed");
   if(ref_arc[j]<=end_station&&v==0)throw std::logic_error("world body stop unsupported");
 }
 for(const auto&p:in.prepare.points)if(!std::isfinite(p.point.longitudinal_velocity_mps)||
    p.point.longitudinal_velocity_mps<=0)throw std::logic_error("world prepare stop unsupported");
 std::vector<int64_t> previous_ids,previous_sorted;double previous_s=-1.;
 for(size_t j=start_index+1;j<end_index;++j){
   const auto projection=project(geometry.points[j].point.pose.position);
   if(projection.station+1e-9<previous_s)throw std::logic_error("nonmonotone lane annotation");
   previous_s=projection.station;
   geometry.points[j].lane_ids=lc::replace_with_sorted_ids(reference.points[projection.owner].lane_ids,
     in.sorted_lane_ids,previous_ids,previous_sorted);
 }
 budget();
 auto anchored=svc.anchor(geometry);auto &path=anchored.first;const size_t begin=anchored.second;
 start_index=autoware::motion_utils::findNearestIndex(path.points,geometry.points[start_index].point.pose.position);
 end_index=autoware::motion_utils::findNearestIndex(path.points,end.position);
 if(begin>start_index||start_index>=end_index)throw std::logic_error("invalid world phases");
 const auto caps=svc.hard_caps(path,begin,start_index,end_index,geometry);
 const auto arc=lc::safety_arc_lengths(path);std::vector<double> stations;
 for(size_t i=begin;i<arc.size();++i)stations.push_back(arc[i]-arc[begin]);
 const double v0=common.self_odometry_ptr->twist.twist.linear.x;
 const double a0=common.current_acceleration->accel.accel.linear.x;
 const auto limit=[&](double v){return common.acceleration_limits.limit(v,
   std::min(common.bpp_param_ptr->max_acc,common.lc_param_ptr->trajectory.max_longitudinal_acc));};
 budget();
 // A finite existing coast subset; no new speed floor, no constant-a claim.
 const auto speed=bounded_speed_grid::make(stations,caps,v0,a0,0.,in.horizon,
   in.minimum_acceleration,in.minimum_jerk,in.maximum_jerk,limit);
 budget();
 for(size_t i=0;i<speed.speeds.size();++i)path.points[begin+i].point.longitudinal_velocity_mps=speed.speeds[i];
 LaneChangePath candidate;candidate.type=bpp::lane_change::PathType::SpatialSpeedProfile;
 candidate.path=path;
 ShiftLine shift;shift.start=path.points[start_index].point.pose;shift.end=path.points[end_index].point.pose;
 candidate.info=LaneChangeInfo(in.prepare_metric,in.changing_metric,shift.start,shift.end,shift);
 const auto elapsed=[&](size_t from,size_t to){double t=0;for(size_t i=from+1;i<=to;++i){
   const double sum=path.points[i-1].point.longitudinal_velocity_mps+path.points[i].point.longitudinal_velocity_mps;
   if(sum<=0)throw std::logic_error("stop before world endpoint");t+=2*(arc[i]-arc[i-1])/sum;}return t;};
 candidate.info.length.prepare=arc[start_index]-arc[begin];
 candidate.info.length.lane_changing=arc[end_index]-arc[start_index];
 candidate.info.duration.prepare=elapsed(begin,start_index);
 candidate.info.duration.lane_changing=elapsed(start_index,end_index);
 candidate.info.velocity.prepare=path.points[start_index].point.longitudinal_velocity_mps;
 candidate.info.velocity.lane_changing=candidate.info.velocity.prepare;
 candidate.info.terminal_lane_changing_velocity=path.points[end_index].point.longitudinal_velocity_mps;
 candidate.info.longitudinal_acceleration.prepare=candidate.info.duration.prepare>0?
   (candidate.info.velocity.prepare-v0)/candidate.info.duration.prepare:0.;
 candidate.info.longitudinal_acceleration.lane_changing=
   (candidate.info.terminal_lane_changing_velocity-candidate.info.velocity.prepare)/candidate.info.duration.lane_changing;
 candidate.shifted_path.path=path;candidate.shifted_path.path.points.erase(candidate.shifted_path.path.points.begin(),
   candidate.shifted_path.path.points.begin()+start_index);
 for(const auto&p:candidate.shifted_path.path.points)
   candidate.shifted_path.shift_length.push_back(project(p.point.pose.position).offset);
 candidate.info.shift_line.start_idx=0;candidate.info.shift_line.end_idx=end_index-start_index;
 candidate.info.shift_line.start_shift_length=0.;
 candidate.info.shift_line.end_shift_length=candidate.shifted_path.shift_length.front()-
   candidate.shifted_path.shift_length.at(end_index-start_index);
 double peak=0;
 for(size_t i=start_index;i<=end_index;++i){if(i==0||i+1>=path.points.size())continue;
   const auto&a=path.points[i-1].point.pose.position;const auto&b=path.points[i].point.pose.position;const auto&c=path.points[i+1].point.pose.position;
   const double x=b.x-a.x,y=b.y-a.y,u=c.x-b.x,w=c.y-b.y;
   const double den=std::hypot(x,y)*std::hypot(u,w)*std::hypot(c.x-a.x,c.y-a.y);
   if(den<1e-12)throw std::logic_error("degenerate world body");
   const double v=path.points[i].point.longitudinal_velocity_mps;peak=std::max(peak,std::abs(2*(x*w-y*u)/den)*v*v);
 }
 candidate.info.lateral_acceleration=peak;
 if(candidate.info.length.sum()+common.transient_data.next_dist_buffer.min>common.transient_data.dist_to_terminal_end)
   throw std::logic_error("actual world length infeasible");
 const double seam=std::abs(autoware_utils::normalize_radian(
   tf2::getYaw(in.prepare.points[in.prepare.points.size()-2].point.pose.orientation)-
   tf2::getYaw(candidate.shifted_path.path.points.at(1).point.pose.orientation)));
 if(seam>autoware_utils::deg2rad(5.))throw std::logic_error("original world seam gate");
 if(lc::is_intersecting_no_lane_change_lines(in.common,candidate.info.length,candidate.shifted_path.path.points))
   throw std::logic_error("world forbidden line");
 budget();
 return {std::move(candidate),(ros::WallTime::now()-started).toSec()*1000.,false};
}
} // namespace
