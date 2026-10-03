#ifndef AUTOWARE__PATH_OPTIMIZER__C2_BRIDGE_HPP_
#define AUTOWARE__PATH_OPTIMIZER__C2_BRIDGE_HPP_

#include "autoware/path_optimizer/type_alias.hpp"
#include <autoware/motion_utils/trajectory/rising_velocity.hpp>
#include <autoware/motion_utils/trajectory/acceleration_limits.hpp>
#include <Eigen/Dense>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::path_optimizer::detail
{
// Candidate construction and all candidate-specific checks are transactional.
// Returning the baseline is NOT a safety approval: callers must still run the
// ordinary final road/obstacle/stop checks on the selected output.
template<class BuildAndValidate>
std::vector<TrajectoryPoint> adoptValidatedBridgeOrBaseline(
  const std::vector<TrajectoryPoint> & baseline, BuildAndValidate && build,
  bool & adopted, std::string & rejection)
{
  adopted = false;
  rejection.clear();
  try {
    auto candidate = build();
    if (candidate.empty()) throw std::logic_error("C2 bridge empty candidate");
    adopted = true;
    return candidate;
  } catch (const std::logic_error & error) {
    rejection = error.what();
    return baseline;
  }
}

struct C2BridgeResult
{
  std::vector<TrajectoryPoint> points;
  size_t begin;
  size_t end;
};

// A midpoint disk enlarged by half the maximum center travel encloses every
// disk on the interpolated segment. Refine this certificate, never the actual
// vehicle radius. Callback owns a shared per-message work/time budget.
template<class Outside>
bool certifySweptCircleSegment(
  const geometry_msgs::Pose & a, const geometry_msgs::Pose & b,
  const std::vector<double> & radii, const std::vector<double> & offsets,
  Outside && outside, const size_t depth=0)
{
  if (radii.empty() || radii.size()!=offsets.size()) return false;
  const double yaw=tf2::getYaw(a.orientation);
  const double dyaw=std::atan2(std::sin(tf2::getYaw(b.orientation)-yaw),
    std::cos(tf2::getYaw(b.orientation)-yaw));
  double max_offset=0.0;
  for(double v:offsets) {if(!std::isfinite(v))return false;max_offset=std::max(max_offset,std::abs(v));}
  const double sweep=std::hypot(b.position.x-a.position.x,b.position.y-a.position.y)+
    max_offset*std::abs(dyaw);
  if(!std::isfinite(sweep) || sweep>10.0)return false;
  auto mid=a;mid.position.x=(a.position.x+b.position.x)/2;
  mid.position.y=(a.position.y+b.position.y)/2;
  mid.orientation.x=0;mid.orientation.y=0;
  mid.orientation.z=std::sin((yaw+dyaw/2)/2);mid.orientation.w=std::cos((yaw+dyaw/2)/2);
  auto inflated=radii;
  for(auto &r:inflated) {if(!std::isfinite(r)||r<=0)return false;r+=sweep/2;}
  if(!outside(mid,inflated,offsets))return true;
  // A real disk collision cannot be rescued by subdivision.
  if(outside(mid,radii,offsets) || depth>=8 || sweep/2<=.001)return false;
  return certifySweptCircleSegment(a,mid,radii,offsets,outside,depth+1) &&
    certifySweptCircleSegment(mid,b,radii,offsets,outside,depth+1);
}

// A local quintic matches endpoint position, heading and signed curvature.
// It never reinterpolates the optimized prefix or unchanged input suffix.
// Road/obstacle validation is deliberately mandatory at the caller, which owns
// the synchronized PlannerData. Throwing must result in a checked fallback/stop.
inline C2BridgeResult makeC2Bridge(
  const std::vector<TrajectoryPoint> & prefix,
  const std::vector<TrajectoryPoint> & input, const size_t join,
  const double max_curvature, const bool transfer_rising_velocity=false,
  const autoware::motion_utils::AccelerationLimits * acceleration_limits=nullptr)
{
  if (prefix.size() < 3 || join < 1 || join + 1 >= input.size() ||
      !std::isfinite(max_curvature) || max_curvature <= 0.0) {
    throw std::logic_error("C2 bridge invalid endpoint input");
  }
  const auto xy = [](const auto & point) {
    return Eigen::Vector2d(point.pose.position.x, point.pose.position.y);
  };
  const auto signed_curvature = [&](const auto & a, const auto & b, const auto & c) {
    const Eigen::Vector2d ab = xy(b)-xy(a), bc = xy(c)-xy(b), ac = xy(c)-xy(a);
    const double denominator = ab.norm()*bc.norm()*ac.norm();
    if (!std::isfinite(denominator) || denominator < 1e-8) {
      throw std::logic_error("C2 bridge degenerate endpoint curvature");
    }
    return 2.0*(ab.x()*ac.y()-ab.y()*ac.x())/denominator;
  };
  const auto & first = prefix.back();
  const auto & last = input[join];
  const auto valid_endpoint = [](const auto & p) {
    const auto & q = p.pose.orientation;
    const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return std::isfinite(norm) && std::abs(norm-1.0) < 1e-3 &&
      std::isfinite(p.longitudinal_velocity_mps) && p.longitudinal_velocity_mps >= 0.0;
  };
  if (!valid_endpoint(first) || !valid_endpoint(last)) {
    throw std::logic_error("C2 bridge invalid endpoint orientation/velocity");
  }
  const Eigen::Vector2d a = xy(first), b = xy(last);
  const double length = (b-a).norm();
  if (!a.allFinite() || !b.allFinite() || !std::isfinite(length) || length < 1.0 ||
      length > 30.0) throw std::logic_error("C2 bridge invalid length");
  const double yaw0 = tf2::getYaw(first.pose.orientation);
  const double yaw1 = tf2::getYaw(last.pose.orientation);
  const double k0 = signed_curvature(prefix[prefix.size()-3], prefix[prefix.size()-2], first);
  const double k1 = signed_curvature(input[join-1], last, input[join+1]);
  if (!std::isfinite(yaw0) || !std::isfinite(yaw1) || std::abs(k0) > max_curvature ||
      std::abs(k1) > max_curvature) throw std::logic_error("C2 bridge invalid endpoint state");
  const auto tangent_check = [&](const Eigen::Vector2d & segment, const double yaw) {
    if (segment.norm() < 1e-6) return false;
    const double delta = yaw-std::atan2(segment.y(),segment.x());
    return std::abs(std::atan2(std::sin(delta),std::cos(delta))) <=
      max_curvature*segment.norm()+0.02;
  };
  if (!tangent_check(a-xy(prefix[prefix.size()-2]), yaw0) ||
      !tangent_check(xy(input[join+1])-b, yaw1)) {
    throw std::logic_error("C2 bridge endpoint tangent mismatch");
  }
  const Eigen::Vector2d t0{std::cos(yaw0),std::sin(yaw0)}, t1{std::cos(yaw1),std::sin(yaw1)};
  Eigen::Matrix<double,6,2> c;
  c.row(0)=a.transpose(); c.row(1)=(length*t0).transpose();
  c.row(2)=(0.5*length*length*k0*Eigen::Vector2d{-t0.y(),t0.x()}).transpose();
  Eigen::Matrix3d matrix;
  matrix << 1,1,1, 3,4,5, 6,12,20;
  Eigen::Matrix<double,3,2> rhs;
  rhs.row(0)=b.transpose()-c.row(0)-c.row(1)-c.row(2);
  rhs.row(1)=(length*t1).transpose()-c.row(1)-2*c.row(2);
  rhs.row(2)=(length*length*k1*Eigen::Vector2d{-t1.y(),t1.x()}).transpose()-2*c.row(2);
  c.bottomRows<3>()=matrix.fullPivLu().solve(rhs);
  if (!c.allFinite()) throw std::logic_error("C2 bridge nonfinite polynomial");

  C2BridgeResult result{prefix,prefix.size()-1,0};
  // This is velocity-envelope transport, not a new acceleration command.
  // Integrate only the input's positive v^2/2 gradient on the actual bridge
  // arc, starting from the unchanged optimized endpoint. A shorter bridge
  // therefore cannot amplify the source acceleration. Falling/flat limits
  // never raise the previous velocity, and zero-containing joins retain the
  // legacy stop insertion contract unchanged.
  bool transport=transfer_rising_velocity && first.longitudinal_velocity_mps>0;
  if(transport && (!acceleration_limits || !acceleration_limits->enabled()))
    throw std::logic_error("C2 bridge missing acceleration calibration");
  if(transport) {
    for(size_t j=0;j<=join;++j) {
      const double v=input[j].longitudinal_velocity_mps;
      if(!std::isfinite(v)||v<0)throw std::logic_error("C2 bridge invalid transport speed");
      if(v==0)transport=false;
    }
  }
  double previous_energy=double(first.longitudinal_velocity_mps)*first.longitudinal_velocity_mps;
  const size_t samples = static_cast<size_t>(std::ceil(length/0.05));
  float legacy_end_velocity=first.longitudinal_velocity_mps;
  Eigen::Vector2d previous = a;
  for (size_t i = 1; i <= samples; ++i) {
    const double u = static_cast<double>(i)/samples;
    Eigen::Vector2d point = Eigen::Vector2d::Zero(), derivative = point, second = point;
    for (int j = 0; j < 6; ++j) {
      point += std::pow(u,j)*c.row(j).transpose();
      if (j >= 1) derivative += j*std::pow(u,j-1)*c.row(j).transpose();
      if (j >= 2) second += j*(j-1)*std::pow(u,j-2)*c.row(j).transpose();
    }
    const double speed = derivative.norm();
    const double curvature = (derivative.x()*second.y()-derivative.y()*second.x())/
      std::pow(speed,3);
    if (!point.allFinite() || !std::isfinite(speed) || speed < 1e-6 ||
        !std::isfinite(curvature) || std::abs(curvature) > max_curvature ||
        (point-previous).norm() > 0.2 || (point-previous).norm() < 1e-8 ||
        derivative.dot(b-a) <= 0.0) {
      throw std::logic_error("C2 bridge invalid sampled geometry");
    }
    auto value = first;
    value.pose.position.x=point.x();value.pose.position.y=point.y();
    value.pose.position.z=(1-u)*first.pose.position.z+u*last.pose.position.z;
    const double yaw=std::atan2(derivative.y(),derivative.x());
    value.pose.orientation.x=0.0;value.pose.orientation.y=0.0;
    value.pose.orientation.z=std::sin(yaw/2);value.pose.orientation.w=std::cos(yaw/2);
    if (!std::isfinite(value.pose.position.z)) throw std::logic_error("C2 bridge invalid z");
    // Copy a nonnegative, bounded speed only. Exact input stop transitions are
    // reinserted by the caller using the original stop pose, then propagated.
    value.longitudinal_velocity_mps=first.longitudinal_velocity_mps;
    size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j <= join; ++j) {
      const double squared = (xy(input[j])-point).squaredNorm();
      if (squared < nearest_distance) {nearest_distance=squared;nearest=j;}
    }
    const double cap = input[nearest].longitudinal_velocity_mps;
    if (!std::isfinite(cap) || cap < 0.0) throw std::logic_error("C2 bridge invalid input speed");
    // Zero transitions are inserted at the original stop pose, not shifted to
    // the nearest sample. Nonzero local limits are never raised by this bridge.
    if (cap > 0.0) value.longitudinal_velocity_mps=std::min<double>(cap,value.longitudinal_velocity_mps);
    if(i==samples)legacy_end_velocity=std::min(value.longitudinal_velocity_mps,last.longitudinal_velocity_mps);
    if(transport) {
      // Same nearest-vertex/incident-segment rule as native path projection,
      // bounded to the immutable prefix of the input used by this bridge.
      size_t segment=nearest;
      if(segment==join)--segment;
      else if(segment>0 && (point-xy(input[segment])).dot(
          xy(input[segment+1])-xy(input[segment]))<=0)--segment;
      const auto & left=input[segment];const auto & right=input[segment+1];
      const double source_length=(xy(right)-xy(left)).norm();
      if(!std::isfinite(source_length)||source_length<1e-8)
        throw std::logic_error("C2 bridge degenerate velocity projection");
      double mapped=autoware::motion_utils::rising_velocity_at_projection(
        left.pose.position,right.pose.position,value.pose.position,
        left.longitudinal_velocity_mps,right.longitudinal_velocity_mps);
      if(right.longitudinal_velocity_mps<=left.longitudinal_velocity_mps)
        mapped=std::min(mapped,cap); // Keep the original falling-limit convention.
      const double gradient=std::max(0.0,
        (double(right.longitudinal_velocity_mps)*right.longitudinal_velocity_mps-
         double(left.longitudinal_velocity_mps)*left.longitudinal_velocity_mps)/source_length);
      const double reachable=acceleration_limits->reachableVelocity(
        std::sqrt(previous_energy),(point-previous).norm(),gradient*.5);
      const double energy=std::min(mapped*mapped,reachable*reachable);
      if(!std::isfinite(energy)||energy<0)throw std::logic_error("C2 bridge invalid transported energy");
      const double bounded_velocity=std::sqrt(energy);
      value.longitudinal_velocity_mps=bounded_velocity;
      if(value.longitudinal_velocity_mps>bounded_velocity)
        value.longitudinal_velocity_mps=std::nextafter(value.longitudinal_velocity_mps,0.0f);
      previous_energy=double(value.longitudinal_velocity_mps)*value.longitudinal_velocity_mps;
    }
    result.points.push_back(value);previous=point;
  }
  result.end=result.points.size()-1;
  result.points.back().pose=last.pose;  // Exact rejoin pose; derivative already matches its yaw.
  result.points.back().longitudinal_velocity_mps=std::min(
    result.points.back().longitudinal_velocity_mps,last.longitudinal_velocity_mps);
  // The suffix is immutable. Do not worsen its existing positive seam jump
  // when a preceding falling limit prevented the transported profile catching
  // back up. This comparison does not certify the legacy seam physically.
  if(transport && input[join+1].longitudinal_velocity_mps>
      result.points.back().longitudinal_velocity_mps &&
      result.points.back().longitudinal_velocity_mps<legacy_end_velocity)
    throw std::logic_error("C2 bridge worsened suffix velocity seam");
  result.points.insert(result.points.end(),input.begin()+join+1,input.end());
  return result;
}
}  // namespace autoware::path_optimizer::detail
#endif
