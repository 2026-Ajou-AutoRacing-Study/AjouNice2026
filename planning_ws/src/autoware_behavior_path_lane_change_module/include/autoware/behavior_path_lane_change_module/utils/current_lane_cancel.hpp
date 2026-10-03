#ifndef AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_UTILS_CURRENT_LANE_CANCEL_HPP_
#define AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_UTILS_CURRENT_LANE_CANCEL_HPP_
#include "autoware/behavior_path_lane_change_module/utils/frenet_safety.hpp"
namespace autoware::behavior_path_planner::utils::lane_change
{
// Advance ego to the newer object stamp; never back-extrapolate objects or
// silently clamp a future observation to an older ego time.
inline SafetyPrediction rebase_observation_prediction(
  const SafetyPrediction & input,double offset,double horizon)
{
  if(!std::isfinite(offset)||offset<0||offset>.010000001||!std::isfinite(horizon)||horizon<=0)
    throw std::logic_error("invalid_observation_time_rebase");
  SafetyPrediction result;
  const auto append=[&](double time){
    const auto point=path_safety_checker::calc_interpolated_pose_with_velocity(input,time+offset);
    if(!point)throw std::logic_error("observation_time_rebase_missing_coverage");
    result.push_back(*point);result.back().time=time;
  };
  append(0);
  for(const auto & point:input)
    if(point.time>offset&&point.time<offset+horizon){result.push_back(point);result.back().time-=offset;}
  append(horizon);
  return result;
}
inline std::optional<double> strict_physical_overlap(
  const SafetyPrediction & ego,const path_safety_checker::ExtendedPredictedObject & object,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle)
{
  for(const auto & prediction:object.predicted_paths)
    for(const auto & point:prediction.path){
      const auto at=path_safety_checker::calc_interpolated_pose_with_velocity(ego,point.time);
      if(!at)throw std::logic_error("physical_overlap_missing_ego_coverage");
      const auto footprint=path_safety_checker::create_extended_ego_polygon(at->pose,vehicle,0,0,false);
      if(boost::geometry::intersects(footprint,point.polygon))return point.time;
    }
  return std::nullopt;
}
// Current-lane following is not an LC native trajectory. Its upstream speeds
// are ceilings; form a separately reachable profile without inventing LC times.
inline double current_lane_cancel_envelope(
  const SafetyPath & upstream,const SafetyPath & approved,const geometry_msgs::Pose & ego,
  const geometry_msgs::Pose & prepare_start,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle)
{
  const auto incoming_arc=safety_arc_lengths(upstream),outgoing_arc=safety_arc_lengths(approved);
  const auto current=project_safety_pose(approved,outgoing_arc,ego,std::numeric_limits<double>::infinity());
  const auto endpoint=project_safety_pose(approved,outgoing_arc,prepare_start,.1);
  if(endpoint.arc<=current.arc)throw std::logic_error("current_lane_cancel_no_shared_prepare_prefix");
  const auto yaw=[](const auto & q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
  double last_station=-std::numeric_limits<double>::infinity();
  const auto check=[&](double station){
    const auto from=safety_pose_at_arc(approved,outgoing_arc,station);
    const auto projection=project_safety_pose(upstream,incoming_arc,from,std::numeric_limits<double>::infinity());
    if(projection.arc<last_station-1e-4)throw std::logic_error("current_lane_cancel_nonmonotone_shared_prefix");
    last_station=projection.arc;
    if(projection.distance>.1)
      throw std::logic_error("current_lane_cancel_path_discontinuity_m="+std::to_string(projection.distance));
    const auto to=safety_pose_at_arc(upstream,incoming_arc,projection.arc);
    const double angle=std::abs(std::remainder(yaw(from.orientation)-yaw(to.orientation),2*M_PI));
    if(angle>.1)throw std::logic_error("current_lane_cancel_path_heading_discontinuity_rad="+std::to_string(angle));
  };
  for(double s=current.arc;s<endpoint.arc;s+=.5)check(s);
  check(endpoint.arc);
  const auto projection=project_safety_pose(upstream,incoming_arc,ego,std::numeric_limits<double>::infinity());
  const auto nominal=safety_pose_at_arc(upstream,incoming_arc,projection.arc);
  const double angle=std::abs(std::remainder(yaw(nominal.orientation)-yaw(ego.orientation),2*M_PI));
  const double radius=std::hypot(std::max(std::abs(vehicle.min_longitudinal_offset_m),
    std::abs(vehicle.max_longitudinal_offset_m)),std::max(std::abs(vehicle.min_lateral_offset_m),
    std::abs(vehicle.max_lateral_offset_m)));
  // Enclose the CURRENT measured translation and rotation error about base_link.
  // Carrying this envelope through H is an explicit receding prediction
  // assumption, not a calibrated bound on future controller tracking error.
  const double margin=projection.distance+2*radius*std::sin(angle/2);
  if(!std::isfinite(margin))throw std::logic_error("invalid_current_lane_cancel_envelope");
  return margin;
}

inline autoware::vehicle_info_utils::VehicleInfo cancel_envelope_vehicle(
  const autoware::vehicle_info_utils::VehicleInfo & v,const double margin)
{
  if(!std::isfinite(margin)||margin<0)throw std::logic_error("invalid_cancel_footprint_margin");
  return autoware::vehicle_info_utils::createVehicleInfo(v.wheel_radius_m,v.wheel_width_m,
    v.wheel_base_m,v.wheel_tread_m,v.front_overhang_m+margin,v.rear_overhang_m+margin,
    v.left_overhang_m+margin,v.right_overhang_m+margin,v.vehicle_height_m,v.max_steer_angle_rad);
}

inline SafetyPath current_lane_following_prediction(
  const SafetyPath & upstream,const geometry_msgs::Pose & ego,const double velocity,
  const double acceleration,const double deceleration,
  const autoware::motion_utils::AccelerationLimits & calibration)
{
  const auto arc=safety_arc_lengths(upstream);
  project_safety_pose(upstream,arc,ego,std::numeric_limits<double>::infinity());
  const size_t segment=autoware::motion_utils::findNearestSegmentIndex(upstream.points,ego.position);
  const auto anchored=anchor_maneuver_at_ego(upstream,segment,ego.position);
  return maneuver_prediction_profile(anchored.first,anchored.second,anchored.second,
    anchored.second,true,velocity,0,0,acceleration,deceleration,calibration);
}
}
#endif
