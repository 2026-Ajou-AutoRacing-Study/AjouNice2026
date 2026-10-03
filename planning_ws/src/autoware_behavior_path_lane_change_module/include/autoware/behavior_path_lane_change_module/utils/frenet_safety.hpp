#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__FRENET_SAFETY_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__FRENET_SAFETY_HPP_
#include "autoware/behavior_path_lane_change_module/utils/maneuver_safety.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp"

namespace autoware::behavior_path_planner::utils::lane_change
{
inline SafetyPrediction restore_frenet_transition(
  SafetyPrediction native, const autoware::behavior_path_planner::lane_change::TrajectoryGroup & group)
{
  const auto & trajectory=group.lane_changing;
  if(trajectory.times.empty() || trajectory.poses.size()!=trajectory.times.size() ||
     trajectory.longitudinal_velocities.size()!=trajectory.times.size() ||
     std::abs(trajectory.times.front())>1e-9)
    throw std::logic_error("invalid stored frenet transition");
  const double time=group.prepare_metric.duration;
  const auto it=std::lower_bound(native.begin(),native.end(),time,
    [](const auto & point,double t){return point.time<t;});
  if(it==native.end() || std::abs(it->time-time)>1e-9)
    native.insert(it,{time,trajectory.poses.front(),trajectory.longitudinal_velocities.front()});
  else if(std::hypot(it->pose.position.x-trajectory.poses.front().position.x,
                   it->pose.position.y-trajectory.poses.front().position.y)>1e-3 ||
          std::abs(it->velocity-trajectory.longitudinal_velocities.front())>1e-3)
    throw std::logic_error("inconsistent native frenet transition");
  return native;
}

struct SafetyProjection {double arc; double distance;};
inline SafetyProjection project_safety_pose(
  const SafetyPath & path,const std::vector<double> & arc,const geometry_msgs::Pose & pose,
  const double max_distance)
{
  const auto yaw=[](const auto&q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
  const auto & q=pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(pose.position.x+pose.position.y+norm) || std::abs(norm-1.0)>1e-3)
    throw std::logic_error("invalid frenet projection pose");
  double best=std::numeric_limits<double>::infinity(),station=0;
  std::vector<std::pair<double,double>> alternatives;
  for(size_t i=1;i<path.points.size();++i){
    const auto & a=path.points[i-1].point.pose.position;
    const auto & b=path.points[i].point.pose.position;
    const double heading=std::atan2(b.y-a.y,b.x-a.x);
    if(std::abs(std::remainder(yaw(pose.orientation)-heading,2*M_PI))>M_PI_2)continue;
    const double ds=arc[i]-arc[i-1];
    const double ratio=std::clamp(((pose.position.x-a.x)*(b.x-a.x)+(pose.position.y-a.y)*(b.y-a.y))/(ds*ds),0.0,1.0);
    const double distance=std::hypot(pose.position.x-a.x-ratio*(b.x-a.x),pose.position.y-a.y-ratio*(b.y-a.y));
    const double at=arc[i-1]+ratio*ds;
    alternatives.emplace_back(distance,at);
    if(distance<best){best=distance;station=at;}
  }
  if(!std::isfinite(best)||best>max_distance)throw std::logic_error("frenet geometry or heading mismatch");
  for(const auto & candidate:alternatives)
    if(candidate.first<best+1e-3 && std::abs(candidate.second-station)>2.0)
      throw std::logic_error("ambiguous frenet path projection");
  return {station,best};
}

// RSS phase time is local to this prediction's current time zero, never the
// original candidate creation time. Use the same piecewise-linear pose/time
// interpolant as the collision checker; do not alter native predictions.
inline double remaining_prepare_time(
  const SafetyPath & path,const size_t prepare_end,const SafetyPrediction & hypothesis,
  const bool immediate)
{
  const auto arc=safety_arc_lengths(path);
  if(prepare_end>=arc.size() || hypothesis.size()<2 ||
     std::abs(hypothesis.front().time)>1e-9)
    throw std::logic_error("invalid remaining prepare input");
  std::vector<double> projected;
  for(size_t i=0;i<hypothesis.size();++i){
    if(!std::isfinite(hypothesis[i].time) ||
       (i && hypothesis[i].time<=hypothesis[i-1].time))
      throw std::logic_error("invalid remaining prepare time");
    const double station=project_safety_pose(path,arc,hypothesis[i].pose,.1).arc;
    if(i && station<projected.back()-1e-4)
      throw std::logic_error("nonmonotone remaining prepare prediction");
    projected.push_back(station);
  }
  if(immediate || projected.front()>=arc[prepare_end]-1e-6)return 0;
  for(size_t i=1;i<projected.size();++i){
    if(projected[i]<arc[prepare_end])continue;
    const double u=(arc[prepare_end]-projected[i-1])/(projected[i]-projected[i-1]);
    return hypothesis[i-1].time+u*(hypothesis[i].time-hypothesis[i-1].time);
  }
  // The final sample also stays prepare when no crossing was observed.
  return std::nextafter(hypothesis.back().time,std::numeric_limits<double>::infinity());
}

// Retain the native Frenet time/pose/velocity interpolant. Only the time origin
// is rebased to measured ego progress. The normal-following tail is a separate
// reachable speed model, never native road-cap-as-speed interpolation.
inline SafetyPrediction frenet_observation_hypothesis(
  const SafetyPath & path,const SafetyPrediction & native,const geometry_msgs::Pose & ego,
  const double actual_speed,const double horizon,const double resolution,
  const double tail_acceleration,const double deceleration,
  const autoware::motion_utils::AccelerationLimits & calibration={})
{
  // Up to 10 ms extra ego coverage allows rebasing to a newer object stamp.
  if(native.size()<2 || !std::isfinite(horizon) || horizon<2.5 || horizon>3.010000001 ||
     !std::isfinite(resolution)||resolution<.05||resolution>1.0)
    throw std::logic_error("invalid frenet observation input");
  const auto arc=safety_arc_lengths(path);
  // Time rebasing belongs to the stored native geometry, not to today's
  // cropped output prefix. Old native points need not remain in that prefix.
  SafetyPath native_geometry;
  std::vector<double> native_arc;
  for(size_t i=0;i<native.size();++i){
    const auto & point=native[i];
    if(!std::isfinite(point.time+point.velocity)||point.velocity<0 ||
       (i && point.time<=native[i-1].time))throw std::logic_error("invalid native frenet time");
    const auto & q=point.pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(!std::isfinite(point.pose.position.x+point.pose.position.y+norm)||std::abs(norm-1)>1e-3)
      throw std::logic_error("invalid native frenet pose");
    double ds=0;
    if(i){
      const auto & previous=native[i-1].pose;
      const double dx=point.pose.position.x-previous.position.x;
      const double dy=point.pose.position.y-previous.position.y;
      ds=std::hypot(dx,dy);
      const auto & p=previous.orientation;
      const double yaw=std::atan2(2*(p.w*p.z+p.x*p.y),1-2*(p.y*p.y+p.z*p.z));
      if(ds>1e-6 && dx*std::cos(yaw)+dy*std::sin(yaw)<-1e-4)
        throw std::logic_error("nonmonotone native frenet geometry");
    }
    native_arc.push_back((i?native_arc.back():0)+ds);
    if(!i || ds>1e-6){
      autoware_internal_planning_msgs::PathPointWithLaneId p;
      p.point.pose=point.pose;p.point.longitudinal_velocity_mps=point.velocity;
      native_geometry.points.push_back(p);
    }
  }
  const auto ego_projection=project_safety_pose(path,arc,ego,1.0);
  const auto native_geometry_arc=safety_arc_lengths(native_geometry);
  const auto native_end_projection=project_safety_pose(path,arc,native.back().pose,.1);
  const bool completed=ego_projection.arc>native_end_projection.arc+1e-4;
  const double ego_native_arc=completed?native_arc.back():
    project_safety_pose(native_geometry,native_geometry_arc,ego,1.0).arc;
  double origin=native.back().time;
  if(!completed && ego_native_arc<native_arc.back()-1e-6){
    for(size_t i=1;i<native_arc.size();++i)
      if(std::abs(native_arc[i]-native_arc[i-1])<1e-6 &&
         native_arc[i]>=ego_native_arc-1e-4 &&
         (native[i-1].velocity<=1e-6 || native[i].velocity<=1e-6))
        throw std::logic_error("ambiguous native frenet stop time");
    const auto hi=static_cast<size_t>(std::upper_bound(native_arc.begin(),native_arc.end(),ego_native_arc)-native_arc.begin());
    if(hi==0||hi>=native.size())throw std::logic_error("frenet time projection failed");
    const double u=(ego_native_arc-native_arc[hi-1])/(native_arc[hi]-native_arc[hi-1]);
    origin=native[hi-1].time+u*(native[hi].time-native[hi-1].time);
  }
  SafetyPrediction future_native;
  std::vector<double> future_arc;
  if(!completed){
    const auto start=path_safety_checker::calc_interpolated_pose_with_velocity(native,origin);
    if(!start)throw std::logic_error("native frenet initial interpolation failed");
    future_native.push_back(*start);
    for(const auto & point:native)if(point.time>origin+1e-9)future_native.push_back(point);
    for(const auto & point:future_native){
      const double station=project_safety_pose(path,arc,point.pose,.1).arc;
      if(!future_arc.empty() && station<future_arc.back()-1e-4)
        throw std::logic_error("nonmonotone future frenet geometry");
      future_arc.push_back(station);
    }
  }
  const double native_remaining=completed?0.0:native.back().time-origin;
  std::vector<double> tail_s,tail_v;
  SafetyPath tail_path;
  if(horizon>native_remaining+1e-9){
    const auto & start_pose=completed?ego:native.back().pose;
    const size_t segment=autoware::motion_utils::findNearestSegmentIndex(path.points,start_pose.position);
    const auto anchored=anchor_maneuver_at_ego(path,segment,start_pose.position);
    const auto model=maneuver_prediction_profile(anchored.first,anchored.second,anchored.second,
      anchored.second,true,completed?actual_speed:native.back().velocity,0,0,
      tail_acceleration,deceleration,calibration);
    tail_path=model;
    const auto s=safety_arc_lengths(model);
    for(size_t i=anchored.second;i<model.points.size();++i){
      tail_s.push_back(s[i]);tail_v.push_back(model.points[i].point.longitudinal_velocity_mps);
      if(tail_v.back()<=1e-6)break;
    }
    if(tail_v.back()>1e-6 &&
       (tail_s.size()<2 || speed_profile_duration(tail_s,tail_v)<horizon-native_remaining))
      throw std::logic_error("insufficient frenet tail coverage");
  }
  const auto cap_at=[&](double s){
    const auto hi=std::upper_bound(arc.begin(),arc.end(),s);
    if(hi==arc.end())return double(path.points.back().point.longitudinal_velocity_mps);
    const size_t i=std::max<size_t>(1,hi-arc.begin());
    const double u=(s-arc[i-1])/(arc[i]-arc[i-1]);
    return (1-u)*path.points[i-1].point.longitudinal_velocity_mps+u*path.points[i].point.longitudinal_velocity_mps;
  };
  double stop_arc=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<path.points.size();++i)
    if(arc[i]>=ego_projection.arc-1e-4 && path.points[i].point.longitudinal_velocity_mps<=1e-6){stop_arc=arc[i];break;}
  SafetyPrediction output;
  std::vector<double> sample_times;
  const size_t count=static_cast<size_t>(std::ceil(horizon/resolution));
  for(size_t n=0;n<=count;++n)sample_times.push_back(std::min(horizon,n*resolution));
  if(!completed) {
    for(const auto & point:native)
      if(point.time>origin && point.time-origin<horizon)sample_times.push_back(point.time-origin);
    // Include every final-path cap knot within native coverage; a narrow
    // inserted stop/cap cannot hide between the regular prediction samples.
    for(const double station:arc) {
      if(station<ego_projection.arc || station>future_arc.back())continue;
      const size_t hi=std::upper_bound(future_arc.begin(),future_arc.end(),station)-future_arc.begin();
      if(hi==0 || hi>=future_native.size())continue;
      const double u=(station-future_arc[hi-1])/(future_arc[hi]-future_arc[hi-1]);
      const double time=future_native[hi-1].time+u*(future_native[hi].time-future_native[hi-1].time)-origin;
      if(time>0 && time<horizon)sample_times.push_back(time);
    }
  }
  std::sort(sample_times.begin(),sample_times.end());
  sample_times.erase(std::unique(sample_times.begin(),sample_times.end(),
    [](double a,double b){return std::abs(a-b)<1e-9;}),sample_times.end());
  for(const double time:sample_times){
    path_safety_checker::PoseWithVelocityStamped point;
    if(!completed && time<=native_remaining+1e-9){
      const auto native_point=path_safety_checker::calc_interpolated_pose_with_velocity(native,origin+time);
      if(!native_point)throw std::logic_error("native frenet interpolation failed");
      point=*native_point;
    }else{
      const auto state=tail_s.size()>1 ? sample_speed_profile(tail_s,tail_v,time-native_remaining) :
        std::make_pair(tail_s.front(),0.0);
      point.pose=safety_pose_at_arc(tail_path,state.first);
      point.velocity=state.second;
    }
    point.time=time;
    const auto station=project_safety_pose(path,arc,point.pose,.1).arc;
    if(point.velocity>cap_at(station)+.1 || station>stop_arc+1e-4)
      throw std::logic_error("native frenet conflicts with final cap or stop");
    output.push_back(point);
  }
  return output;
}
}
#endif
