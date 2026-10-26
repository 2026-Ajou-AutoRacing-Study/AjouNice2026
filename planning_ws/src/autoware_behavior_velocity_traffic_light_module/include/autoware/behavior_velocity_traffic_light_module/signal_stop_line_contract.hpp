// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_STOP_LINE_CONTRACT_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_STOP_LINE_CONTRACT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner::signal_compliance
{
struct MapPoint { double x{}; double y{}; };
struct StopLineContract
{
  std::int64_t regulatory_id{};
  std::vector<MapPoint> expected_stop_line;
  std::vector<MapPoint> crossing_line;
  double additional_stop_margin_m{};
  double max_anchor_error_m{0.25};
  // Retain a recognized ID even when its configuration is invalid. Its
  // ordinary stopping behavior remains available, but timed GO is not granted.
  bool valid{true};
};
inline MapPoint subtract(MapPoint a, MapPoint b) { return {a.x-b.x,a.y-b.y}; }
inline double dot(MapPoint a, MapPoint b) { return a.x*b.x+a.y*b.y; }
inline double cross(MapPoint a, MapPoint b) { return a.x*b.y-a.y*b.x; }
inline double norm(MapPoint a) { return std::hypot(a.x,a.y); }
inline bool finite(MapPoint p) { return std::isfinite(p.x)&&std::isfinite(p.y); }
inline MapPoint midpoint(MapPoint a, MapPoint b) { return {(a.x+b.x)/2.,(a.y+b.y)/2.}; }

inline bool validStopLineContract(const StopLineContract & c)
{
  if (!c.valid || c.regulatory_id<=0 || c.expected_stop_line.size()!=2 ||
      (c.crossing_line.size()!=2 && c.crossing_line.size()!=3) ||
      !std::isfinite(c.additional_stop_margin_m) || c.additional_stop_margin_m<0. ||
      c.additional_stop_margin_m>5. || !std::isfinite(c.max_anchor_error_m) ||
      c.max_anchor_error_m<=0. || c.max_anchor_error_m>2.) return false;
  for (auto p:c.expected_stop_line) if (!finite(p)) return false;
  for (auto p:c.crossing_line) if (!finite(p)) return false;
  if (norm(subtract(c.expected_stop_line[1],c.expected_stop_line[0]))<.1) return false;
  const auto upstream=midpoint(c.expected_stop_line[0],c.expected_stop_line[1]);
  for (std::size_t i=1;i<c.crossing_line.size();++i) {
    const auto segment=subtract(c.crossing_line[i],c.crossing_line[i-1]);
    const double length=norm(segment);
    // The anchored virtual line identifies the upstream side independently of
    // ego yaw. Do not infer direction afresh after a turn or relocation.
    if (length<.1 || std::abs(cross(segment,subtract(upstream,c.crossing_line[i-1])))/length<.05)
      return false;
  }
  return true;
}

inline bool stopLineAnchorMatches(const StopLineContract & c, const std::vector<MapPoint> & actual)
{
  if (!validStopLineContract(c) || actual.size()<2 || !finite(actual.front()) ||
      !finite(actual.back())) return false;
  const auto & a=c.expected_stop_line;
  const double direct=std::max(norm(subtract(a[0],actual.front())),norm(subtract(a[1],actual.back())));
  const double reversed=std::max(norm(subtract(a[0],actual.back())),norm(subtract(a[1],actual.front())));
  return std::min(direct,reversed)<=c.max_anchor_error_m;
}

// Signed normal distance to a FINITE physical-paint proxy segment: positive
// upstream. An off-segment point is not evidence of crossing this signal line.
// This remains available when a short planner horizon omits the crossing line.
inline std::optional<double> finiteCrossingDistance(const StopLineContract & c, MapPoint front)
{
  if (!validStopLineContract(c) || !finite(front)) return std::nullopt;
  const auto upstream=midpoint(c.expected_stop_line[0],c.expected_stop_line[1]);
  std::optional<double> result;
  for (std::size_t i=1;i<c.crossing_line.size();++i) {
    const auto a=c.crossing_line[i-1], segment=subtract(c.crossing_line[i],a);
    const double length=norm(segment), t=dot(subtract(front,a),segment)/(length*length);
    if (t < -1e-9 || t > 1.+1e-9) continue;
    const double upstream_side=cross(segment,subtract(upstream,a));
    const double value=cross(segment,subtract(front,a))/length*(upstream_side>0.?1.:-1.);
    if (!result || std::abs(value)<std::abs(*result)) result=value;
  }
  return result;
}

// Route arc distance, not the nearer virtual stop target, is used for yellow
// ETA. If the finite physical line is outside the path horizon, return no ETA:
// callers must not silently substitute the virtual line or extrapolate it.
inline std::optional<double> pathCrossingFrontDistance(
  const StopLineContract & c, const std::vector<MapPoint> & path, MapPoint rear,
  double front_axle_offset_m)
{
  if (!validStopLineContract(c) || path.size()<2 || !finite(rear) ||
      !std::isfinite(front_axle_offset_m) || front_axle_offset_m<0.) return std::nullopt;
  double ego_s=0., nearest=std::numeric_limits<double>::infinity(), total=0.;
  std::vector<double> intersections;
  for (std::size_t i=1;i<path.size();++i) {
    const auto a=path[i-1], v=subtract(path[i],a);
    if (!finite(a)||!finite(path[i])) return std::nullopt;
    const double length=norm(v);
    if (length<1e-9) continue;
    const double t=std::clamp(dot(subtract(rear,a),v)/(length*length),0.,1.);
    const MapPoint q{a.x+t*v.x,a.y+t*v.y};
    const double distance=norm(subtract(rear,q));
    if (distance<nearest) {nearest=distance;ego_s=total+t*length;}
    for (std::size_t j=1;j<c.crossing_line.size();++j) {
      const auto b=c.crossing_line[j-1], w=subtract(c.crossing_line[j],b);
      const double denominator=cross(v,w);
      if (std::abs(denominator)<1e-9) continue;
      const auto delta=subtract(b,a);
      const double u=cross(delta,w)/denominator, z=cross(delta,v)/denominator;
      if (u>=-1e-9&&u<=1.+1e-9&&z>=-1e-9&&z<=1.+1e-9)
        intersections.push_back(total+std::clamp(u,0.,1.)*length);
    }
    total+=length;
  }
  if (intersections.empty() || !std::isfinite(nearest)) return std::nullopt;
  const auto closest=*std::min_element(intersections.begin(),intersections.end(),
    [ego_s](double a,double b){return std::abs(a-ego_s)<std::abs(b-ego_s);});
  return closest-ego_s-front_axle_offset_m;
}
}  // namespace autoware::behavior_velocity_planner::signal_compliance
#endif
