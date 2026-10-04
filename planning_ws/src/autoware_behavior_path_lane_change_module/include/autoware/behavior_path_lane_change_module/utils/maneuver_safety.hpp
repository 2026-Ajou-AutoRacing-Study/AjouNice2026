#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MANEUVER_SAFETY_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MANEUVER_SAFETY_HPP_

#include "autoware/behavior_path_lane_change_module/utils/speed_profile.hpp"
#include "autoware/behavior_path_lane_change_module/structs/path.hpp"
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/rising_velocity.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

namespace autoware::behavior_path_planner::utils::lane_change
{
using SafetyPath = autoware_internal_planning_msgs::PathWithLaneId;
using SafetyPrediction = std::vector<path_safety_checker::PoseWithVelocityStamped>;

// Reuse the already validated fresh envelope, not the old maneuver's sampled
// nonzero velocities, for an independently checked braking proposal. Geometry,
// prefix and every original stop remain unchanged; new envelope stops also win.
inline SafetyPath maneuver_braking_hard_caps(
  const SafetyPath & original, const size_t begin, const std::vector<double> & caps)
{
  if(begin>=original.points.size()||caps.size()!=original.points.size()-begin)
    throw std::logic_error("invalid braking hard envelope size");
  for(size_t i=0;i<caps.size();++i) {
    const double old=original.points[begin+i].point.longitudinal_velocity_mps;
    if(!std::isfinite(caps[i])||caps[i]<0||!std::isfinite(old)||old<0)
      throw std::logic_error("invalid braking hard envelope speed");
  }
  auto result=original;
  for(size_t i=0;i<caps.size();++i)
    result.points[begin+i].point.longitudinal_velocity_mps=
      original.points[begin+i].point.longitudinal_velocity_mps==0.0?0.0:caps[i];
  return result;
}

inline LaneChangePath abort_prediction_metadata(const LaneChangePath & published)
{
  auto prediction=published;
  // calcAbortPath creates a fresh PathShifter, calls setVelocity(), and never
  // sets longitudinal acceleration (default 0). Old LC acceleration metadata
  // must not leak into an abort whose reference speeds are merely ceilings.
  prediction.type=autoware::behavior_path_planner::lane_change::PathType::ConstantJerk;
  prediction.info.longitudinal_acceleration.prepare=0.0;
  prediction.info.longitudinal_acceleration.lane_changing=0.0;
  prediction.info.duration.prepare=0.0;
  return prediction;
}

inline std::string safety_json_string(const std::string & value)
{
  static constexpr char hex[]="0123456789abcdef";
  std::string out="\"";
  for(const unsigned char c:value) {
    if(c=='"' || c=='\\'){out+='\\';out+=c;}
    else if(c<0x20){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}
    else out+=c;
  }
  return out+'"';
}

inline path_safety_checker::ExtendedPredictedObject observed_maneuver_object(
  const autoware_perception_msgs::PredictedObject & raw, const double resolution)
{
  if (!std::isfinite(resolution) || resolution<0.05 || resolution>1.0)
    throw std::logic_error("invalid object observation resolution");
  path_safety_checker::ExtendedPredictedObject object(raw);
  const double velocity=std::hypot(object.initial_twist.linear.x,object.initial_twist.linear.y);
  for (const auto & prediction : raw.kinematics.predicted_paths) {
    const double step=prediction.time_step.sec+1e-9*prediction.time_step.nanosec;
    if (prediction.path.size()<2 || !std::isfinite(step) || step<=0)
      throw std::logic_error("invalid raw object observation");
    const double end=step*(prediction.path.size()-1);
    path_safety_checker::PredictedPathWithPolygon out;
    out.confidence=prediction.confidence;
    for (size_t n=0;n*resolution<end;++n) {
      const double time=n*resolution;
      const auto pose=autoware::object_recognition_utils::calcInterpolatedPose(prediction,time);
      if (!pose) throw std::logic_error("raw object interpolation failed");
      out.path.emplace_back(time,*pose,velocity,autoware_utils::to_polygon2d(*pose,raw.shape));
    }
    // Preserve the actually observed endpoint explicitly. t<end+epsilon loses
    // exactly 3.0 s at double precision and otherwise shortens a 3 s contract.
    const auto & final_pose=prediction.path.back();
    out.path.emplace_back(end,final_pose,velocity,autoware_utils::to_polygon2d(final_pose,raw.shape));
    object.predicted_paths.push_back(std::move(out));
  }
  return object;
}

inline std::vector<double> safety_arc_lengths(const SafetyPath & path)
{
  if (path.points.size() < 2) throw std::logic_error("safety path too short");
  std::vector<double> s(path.points.size(), 0.0);
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & p = path.points[i].point;
    const auto & q = p.pose.orientation;
    const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) ||
        !std::isfinite(norm) || std::abs(norm-1.0)>1e-3 ||
        !std::isfinite(p.longitudinal_velocity_mps) || p.longitudinal_velocity_mps < 0.0)
      throw std::logic_error("nonfinite safety path");
    if (i) {
      const auto & a = path.points[i-1].point.pose.position;
      const double ds = std::hypot(p.pose.position.x-a.x, p.pose.position.y-a.y);
      if (ds < 1e-6) throw std::logic_error("duplicate safety path point");
      s[i] = s[i-1]+ds;
    }
  }
  return s;
}

// All safety profiles/projectors use planar arc length. The general motion
// utility calcInterpolatedPose uses 3-D length and must not consume these
// stations: even a small grade can move an exact vertex behind its segment.
inline geometry_msgs::Pose safety_pose_at_arc(
  const SafetyPath & path, const std::vector<double> & arc, const double station)
{
  if (arc.size()!=path.points.size() || arc.size()<2)
    throw std::logic_error("invalid safety pose arc cache");
  if (!std::isfinite(station)) throw std::logic_error("invalid safety pose station");
  if (station < 0.0) return path.points.front().point.pose;
  if (station >= arc.back()) return path.points.back().point.pose;
  const auto upper = std::upper_bound(arc.begin(), arc.end(), station);
  const size_t end = static_cast<size_t>(upper - arc.begin());
  const size_t begin = end - 1;
  const double ratio = (station - arc[begin]) / (arc[end] - arc[begin]);
  return autoware_utils_geometry::calc_interpolated_pose(
    path.points[begin].point.pose, path.points[end].point.pose, ratio, true);
}

inline geometry_msgs::Pose safety_pose_at_arc(const SafetyPath & path, const double station)
{
  return safety_pose_at_arc(path,safety_arc_lengths(path),station);
}

inline std::pair<SafetyPath, size_t> anchor_maneuver_at_ego(
  const SafetyPath & path, size_t segment, const geometry_msgs::Point & ego,
  const bool interpolate_rising_velocity = false)
{
  const auto arc = safety_arc_lengths(path);
  if (segment+1 >= path.points.size()) throw std::logic_error("invalid ego segment");
  const auto & a = path.points[segment].point.pose.position;
  const auto & b = path.points[segment+1].point.pose.position;
  const double ds = arc[segment+1]-arc[segment];
  const double fraction = std::clamp(((ego.x-a.x)*(b.x-a.x)+(ego.y-a.y)*(b.y-a.y))/(ds*ds),0.0,1.0);
  const auto pose = safety_pose_at_arc(path,arc,arc[segment]+fraction*ds);
  auto result = path;
  const auto index = autoware::motion_utils::insertTargetPoint(segment,pose.position,result.points);
  if (!index) throw std::logic_error("failed ego projection anchor");
  // Only a newly inserted point inherits the preceding sample's ceiling.
  // Existing vertices (including repeat anchoring) retain their exact values.
  if (interpolate_rising_velocity && result.points.size() > path.points.size()) {
    result.points[*index].point.longitudinal_velocity_mps =
      autoware::motion_utils::rising_velocity_at_projection(
        a,b,pose.position,path.points[segment].point.longitudinal_velocity_mps,
        path.points[segment+1].point.longitudinal_velocity_mps);
  }
  return {result,*index};
}

// Spatial v(s) is a commanded profile, not a road-speed ceiling. Its t=0
// sample must represent this observation's actual speed. Compare in the ROS
// float32 field's representation, not with a physical speed tolerance. Never
// silently rewrite the profile: that would validate a different publication.
inline void require_actual_spatial_start(
  const SafetyPath & path, const size_t begin, const double actual_speed)
{
  if(begin>=path.points.size() || !std::isfinite(actual_speed) || actual_speed<0 ||
     !std::isfinite(static_cast<float>(actual_speed)) ||
     path.points[begin].point.longitudinal_velocity_mps!=static_cast<float>(actual_speed))
    throw std::logic_error("spatial initial speed differs from actual state");
}

// Prediction-only representation. Published normal-following tail velocities
// are ceilings, not instantaneous commands. Spatial LC points remain exact;
// legacy phase points use their selected acceleration rather than road caps.
inline SafetyPath maneuver_prediction_profile(
  const SafetyPath & original, const size_t begin, const size_t lc_end,
  const size_t prepare_end, const bool spatial, const double actual_speed,
  const double prepare_acceleration, const double changing_acceleration,
  const double tail_acceleration, const double deceleration,
  const autoware::motion_utils::AccelerationLimits & calibration = {})
{
  const auto arc=safety_arc_lengths(original);
  if (begin>=original.points.size()-1 || lc_end>=original.points.size() ||
      prepare_end>=original.points.size() || !std::isfinite(actual_speed) || actual_speed<0 ||
      !std::isfinite(prepare_acceleration+changing_acceleration+tail_acceleration+deceleration) ||
      tail_acceleration<=0 || deceleration<=0)
    throw std::logic_error("invalid maneuver prediction model");
  auto modeled=original;
  const size_t end=std::max(begin,lc_end);
  if(spatial && end>begin)require_actual_spatial_start(original,begin,actual_speed);
  const auto reachable=[&](double v,double distance,double acc) {
    if (acc < -deceleration-1e-6) throw std::logic_error("phase acceleration below limit");
    return acc<0 ? std::sqrt(std::max(0.0,v*v+2*acc*distance)) :
      calibration.reachableVelocity(v,distance,acc);
  };
  if (!spatial || end==begin) {
    if (actual_speed>original.points[begin].point.longitudinal_velocity_mps+0.1)
      throw std::logic_error("legacy initial speed above published cap");
    modeled.points[begin].point.longitudinal_velocity_mps=actual_speed;
    for (size_t i=begin+1;i<=end;++i) {
      const double previous=modeled.points[i-1].point.longitudinal_velocity_mps;
      const double acc=i<=prepare_end ? prepare_acceleration : changing_acceleration;
      const double desired=reachable(previous,arc[i]-arc[i-1],acc);
      modeled.points[i].point.longitudinal_velocity_mps=std::min<double>(
        original.points[i].point.longitudinal_velocity_mps,desired);
      const double next=modeled.points[i].point.longitudinal_velocity_mps;
      if ((next*next-previous*previous)/(2*(arc[i]-arc[i-1])) < -deceleration-1e-3)
        throw std::logic_error("legacy published cap not reachable");
    }
  }
  std::vector<double> s,limits;
  for (size_t i=end;i<original.points.size();++i) {
    s.push_back(arc[i]-arc[end]);
    limits.push_back(original.points[i].point.longitudinal_velocity_mps);
  }
  if (s.size()>1) {
    const auto tail=make_speed_profile(s,limits,
      modeled.points[end].point.longitudinal_velocity_mps,tail_acceleration,deceleration,calibration);
    for(size_t i=1;i<tail.size();++i)
      modeled.points[end+i].point.longitudinal_velocity_mps=tail[i];
  }
  return modeled;
}

// Preserve XY/lane IDs and all upstream stops. This is a reachable braking
// proposal, not proof of collision freedom; the caller must check both motion
// hypotheses and the final geometry before using it.
inline SafetyPath make_maneuver_braking_proposal(
  const SafetyPath & original, const size_t begin, const double speed,
  const double acceleration, const double min_acceleration, const double min_jerk,
  const double max_jerk)
{
  const auto arc = safety_arc_lengths(original);
  if (begin >= original.points.size()-1 || !std::isfinite(speed) || speed < 0.0 ||
      !std::isfinite(acceleration) || !std::isfinite(min_acceleration) || min_acceleration >= 0.0 ||
      !std::isfinite(min_jerk) || min_jerk >= 0.0 ||
      !std::isfinite(max_jerk) || max_jerk <= 0.0)
    throw std::logic_error("invalid braking state");
  std::vector<double> s{0.0}, v{speed};
  double x = 0.0, velocity = speed, a = acceleration;
  constexpr double dt = 0.01;
  for (size_t n = 0; velocity > 1e-6 && n < 3000; ++n) {
    // A command comfort bound is not a validity bound on measured motion.
    // Keep the observed stronger braking at t=0 and release toward the nominal
    // command limit at bounded positive jerk, never jump/clamp acceleration.
    // This commanded profile does not replace the independent measured-motion
    // safety hypothesis or guarantee that the controller realizes max_jerk.
    const double next_a = a < min_acceleration
      ? std::min(min_acceleration, a+max_jerk*dt)
      : std::max(min_acceleration, a+min_jerk*dt);
    const double next_v = std::max(0.0, velocity+0.5*(a+next_a)*dt);
    x += 0.5*(velocity+next_v)*dt;
    s.push_back(x); v.push_back(next_v); velocity=next_v; a=next_a;
  }
  if (velocity > 1e-6 || arc.back()-arc[begin] < x)
    throw std::logic_error("insufficient braking path");
  auto output = original;
  for (size_t i = begin; i < output.points.size(); ++i) {
    const double distance = arc[i]-arc[begin];
    double planned = 0.0;
    if (distance < s.back()) {
      const auto hi = static_cast<size_t>(std::upper_bound(s.begin(),s.end(),distance)-s.begin());
      const size_t lo=hi-1;
      const double u=(distance-s[lo])/(s[hi]-s[lo]);
      planned=std::sqrt(std::max(0.0,v[lo]*v[lo]+u*(v[hi]*v[hi]-v[lo]*v[lo])));
    }
    // Use the same numerical cap tolerance as the bounded candidate family.
    // A braking fallback must not manufacture feasibility by clipping a
    // genuinely unreachable speed to a lower road cap.
    if (planned > original.points[i].point.longitudinal_velocity_mps+1e-5)
      throw std::logic_error("upstream cap requires stronger braking");
    output.points[i].point.longitudinal_velocity_mps = std::min<double>(
      original.points[i].point.longitudinal_velocity_mps, planned);
  }
  return output;
}

// Output v(s) is the planned hypothesis. Measured v/a is a distinct hypothesis;
// it is never replaced with the desired acceleration. Stops retain stationary
// samples through the horizon, rather than making later RSS checks disappear.
inline std::vector<SafetyPrediction> maneuver_motion_hypotheses(
  const SafetyPath & path, const size_t begin, const size_t end,
  const double actual_speed, const double actual_acceleration,
  const double maximum_speed, const double minimum_horizon, const double resolution,
  const std::optional<double> observation_horizon = std::nullopt)
{
  const auto arc = safety_arc_lengths(path);
  if (begin > end || (!observation_horizon && begin==end) || end >= path.points.size() || !std::isfinite(actual_speed) || actual_speed<0 ||
      !std::isfinite(actual_acceleration) || !std::isfinite(maximum_speed) || maximum_speed<=0 ||
      !std::isfinite(minimum_horizon) || minimum_horizon<0 ||
      !std::isfinite(resolution) || resolution<0.05 || resolution>1.0)
    throw std::logic_error("invalid motion hypothesis state");
  std::vector<double> s, v;
  // A bounded observation window may cross the physical LC endpoint. Continue
  // along the final published tail; only a genuine zero may hold a pose still.
  const size_t profile_end = observation_horizon ? path.points.size()-1 : end;
  for (size_t i=begin;i<=profile_end;++i) {
    s.push_back(arc[i]); v.push_back(path.points[i].point.longitudinal_velocity_mps);
    if (v.back()<=1e-6) break;
  }
  const double planned_duration=s.size()>1 ? speed_profile_duration(s,v) : 0.0;
  const bool stopped = v.back() <= 1e-6;
  const double horizon=observation_horizon.value_or(
    std::max(stopped ? 1.0 : planned_duration,minimum_horizon));
  if (observation_horizon && (!std::isfinite(horizon) || horizon<minimum_horizon || horizon<=0))
    throw std::logic_error("insufficient observation horizon");
  if (!stopped && horizon > planned_duration+1e-6)
    throw std::logic_error("moving hypothesis exceeds LC coverage");
  if (!std::isfinite(horizon) || horizon>20.0) throw std::logic_error("unbounded motion horizon");
  std::vector<SafetyPrediction> out(2);
  const size_t samples=static_cast<size_t>(std::ceil(horizon/resolution));
  for (size_t n=0;n<=samples;++n) {
    const double time=std::min(horizon,n*resolution);
    const auto planned=s.size()>1 ? sample_speed_profile(s,v,time) : std::make_pair(s.front(),0.0);
    double moving=time;
    const double cap=std::max(actual_speed,maximum_speed);
    if (actual_acceleration< -1e-6) moving=std::min(time,-actual_speed/actual_acceleration);
    if (actual_acceleration>1e-6) moving=std::min(time,(cap-actual_speed)/actual_acceleration);
    const double actual_v=std::clamp(actual_speed+actual_acceleration*moving,0.0,cap);
    const double actual_s=arc[begin]+actual_speed*moving+0.5*actual_acceleration*moving*moving+
      actual_v*(time-moving);
    if (actual_s>arc.back()+1e-3) throw std::logic_error("actual motion exceeds path coverage");
    out[0].emplace_back(time,safety_pose_at_arc(path,arc,planned.first),planned.second);
    out[1].emplace_back(time,safety_pose_at_arc(path,arc,actual_s),actual_v);
  }
  return out;
}

// No candidate state changes until the final-output validator accepts it.
template<class Validate>
bool accept_checked_maneuver(SafetyPath & output, const SafetyPath & proposal, Validate validate)
{
  if (!validate(proposal)) return false;
  output=proposal;
  return true;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif
