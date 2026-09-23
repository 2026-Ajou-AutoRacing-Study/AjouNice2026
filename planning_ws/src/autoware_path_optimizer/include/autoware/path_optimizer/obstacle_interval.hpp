#ifndef AUTOWARE_PATH_OPTIMIZER_OBSTACLE_INTERVAL_HPP_
#define AUTOWARE_PATH_OPTIMIZER_OBSTACLE_INTERVAL_HPP_
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace autoware::path_optimizer
{
// Intersection of polygon (+ disk radius) with the circle-center lateral axis.
// Evaluate the segment extrema analytically. A bounding rectangle would apply the full
// lateral radius even at the obstacle's longitudinal corners and reject feasible turns.
inline std::optional<std::pair<double,double>> obstacleLateralInterval(
  const geometry_msgs::Polygon & polygon, const geometry_msgs::Pose & origin, double radius)
{
  if (polygon.points.size() < 3 || !(radius > 0.0)) return std::nullopt;
  const double yaw = tf2::getYaw(origin.orientation), c = std::cos(yaw), s = std::sin(yaw);
  const auto local = [&](const geometry_msgs::Point32 & p) {
    const double dx=p.x-origin.position.x, dy=p.y-origin.position.y;
    return std::make_pair(c*dx+s*dy, -s*dx+c*dy);
  };
  double lower=std::numeric_limits<double>::infinity(), upper=-lower;
  const auto accumulate = [&](double x,double y) {
    if (std::abs(x) > radius + 1e-9) return;
    const double height=std::sqrt(std::max(0.0,radius*radius-x*x));
    lower=std::min(lower,y-height); upper=std::max(upper,y+height);
  };
  for (size_t i=0; i<polygon.points.size(); ++i) {
    const auto a=local(polygon.points[i]), b=local(polygon.points[(i+1)%polygon.points.size()]);
    if (std::abs(a.first-b.first)<1e-9) {
      accumulate(a.first,a.second); accumulate(b.first,b.second); continue;
    }
    const double lo=std::max(-radius,std::min(a.first,b.first));
    const double hi=std::min(radius,std::max(a.first,b.first));
    if (lo>hi) continue;
    const double slope=(b.second-a.second)/(b.first-a.first);
    const auto evaluate = [&](double x) { accumulate(x,a.second+slope*(x-a.first)); };
    evaluate(lo); evaluate(hi);
    const double extremum=radius*slope/std::hypot(1.0,slope);
    evaluate(std::clamp(extremum,lo,hi)); evaluate(std::clamp(-extremum,lo,hi));
  }
  if (lower>upper) return std::nullopt;
  return std::make_pair(lower,upper);
}
}
#endif
