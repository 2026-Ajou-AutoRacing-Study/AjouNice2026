// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_JUDGING_LINE_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_JUDGING_LINE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner::signal_compliance
{
struct JudgingPoint { double x{}; double y{}; };
struct SignalJudgingLine
{
  std::int64_t regulatory_id{};
  std::vector<JudgingPoint> points;
  // Direction of the permitted APPROACH, not current ego yaw or a shifted
  // Lanelet stopping target. Only used to orient the signed crossing distance.
  JudgingPoint approach_direction;
  bool valid{true};
};
inline JudgingPoint judgingSubtract(JudgingPoint a, JudgingPoint b)
{ return {a.x-b.x,a.y-b.y}; }
inline double judgingDot(JudgingPoint a, JudgingPoint b)
{ return a.x*b.x+a.y*b.y; }
inline double judgingCross(JudgingPoint a, JudgingPoint b)
{ return a.x*b.y-a.y*b.x; }
inline double judgingNorm(JudgingPoint p) { return std::hypot(p.x,p.y); }
inline bool judgingFinite(JudgingPoint p)
{ return std::isfinite(p.x)&&std::isfinite(p.y); }

inline bool validJudgingLine(const SignalJudgingLine & line)
{
  if(!line.valid || line.regulatory_id<=0 || line.points.size()<2 || line.points.size()>16 ||
     !judgingFinite(line.approach_direction)) return false;
  const auto direction_length=judgingNorm(line.approach_direction);
  if(direction_length<.99 || direction_length>1.01) return false;
  for(std::size_t i=1;i<line.points.size();++i) {
    if(!judgingFinite(line.points[i-1]) || !judgingFinite(line.points[i])) return false;
    const auto segment=judgingSubtract(line.points[i],line.points[i-1]);
    const double length=judgingNorm(segment);
    // A lane divider (parallel to approach) is not an eligible crossing line.
    if(length<.1 || std::abs(judgingCross(segment,line.approach_direction))/length<.5)
      return false;
  }
  return true;
}

inline const SignalJudgingLine * findSignalJudgingLine(
  const std::vector<SignalJudgingLine> & lines,std::int64_t regulatory_id)
{
  const SignalJudgingLine * found=nullptr;
  for(const auto & line:lines) if(line.regulatory_id==regulatory_id) {
    if(found || !validJudgingLine(line)) return nullptr;
    found=&line;
  }
  return found;
}

// Normal distance to a FINITE original-MGeo segment: positive before crossing.
// An off-segment point is not proof of passing this regulatory signal. This
// remains usable when an upstream stop has shortened the planner path horizon.
inline std::optional<double> finiteJudgingLineDistance(
  const SignalJudgingLine & line,JudgingPoint front)
{
  if(!validJudgingLine(line)||!judgingFinite(front)) return std::nullopt;
  std::optional<double> result;
  for(std::size_t i=1;i<line.points.size();++i) {
    const auto a=line.points[i-1], segment=judgingSubtract(line.points[i],a);
    const double length=judgingNorm(segment);
    const double t=judgingDot(judgingSubtract(front,a),segment)/(length*length);
    if(t < -1e-9 || t > 1.+1e-9) continue;
    const double side=judgingCross(segment,line.approach_direction);
    const double distance=-judgingCross(segment,judgingSubtract(front,a))/length*(side>0.?1.:-1.);
    if(!result || std::abs(distance)<std::abs(*result)) result=distance;
  }
  return result;
}

// Route arc from the ACTUAL front-axle point to the finite MGeo line. Caller
// transforms the front point once; do not subtract the axle offset again.
// No intersection means no yellow ETA, never a shorter Lanelet-line substitute.
inline std::optional<double> pathJudgingLineDistance(
  const SignalJudgingLine & line,const std::vector<JudgingPoint> & path,JudgingPoint front)
{
  if(!validJudgingLine(line)||path.size()<2||!judgingFinite(front)) return std::nullopt;
  double front_s=0.,nearest=std::numeric_limits<double>::infinity(),total=0.;
  std::vector<double> intersections;
  for(std::size_t i=1;i<path.size();++i) {
    const auto a=path[i-1],v=judgingSubtract(path[i],a);
    if(!judgingFinite(a)||!judgingFinite(path[i])) return std::nullopt;
    const double length=judgingNorm(v);
    if(length<1e-9) continue;
    const double t=std::clamp(judgingDot(judgingSubtract(front,a),v)/(length*length),0.,1.);
    const JudgingPoint projection{a.x+t*v.x,a.y+t*v.y};
    const double distance=judgingNorm(judgingSubtract(front,projection));
    if(distance<nearest) { nearest=distance;front_s=total+t*length; }
    for(std::size_t j=1;j<line.points.size();++j) {
      const auto b=line.points[j-1],w=judgingSubtract(line.points[j],b);
      const double denominator=judgingCross(v,w);
      if(std::abs(denominator)<1e-9) continue;
      const auto delta=judgingSubtract(b,a);
      const double u=judgingCross(delta,w)/denominator,z=judgingCross(delta,v)/denominator;
      if(u>=-1e-9&&u<=1.+1e-9&&z>=-1e-9&&z<=1.+1e-9)
        intersections.push_back(total+std::clamp(u,0.,1.)*length);
    }
    total+=length;
  }
  // The front point must still lie on the supplied route corridor. A bad path
  // projection is not a reason to authorize timing against another lane.
  if(intersections.empty()||!std::isfinite(nearest)||nearest>3.) return std::nullopt;
  const double crossing=*std::min_element(intersections.begin(),intersections.end(),
    [front_s](double a,double b){return std::abs(a-front_s)<std::abs(b-front_s);});
  return crossing-front_s;
}
}  // namespace autoware::behavior_velocity_planner::signal_compliance
#endif
