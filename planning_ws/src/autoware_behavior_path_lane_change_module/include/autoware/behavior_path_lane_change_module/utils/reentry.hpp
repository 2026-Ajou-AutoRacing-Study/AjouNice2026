#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__REENTRY_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__REENTRY_HPP_

#include <array>
#include <cmath>
#include <optional>

namespace autoware::behavior_path_planner::utils::lane_change
{
// Boundary conditions for a single trajectory from the measured moving vehicle.
// No intermediate original-lane centre or zero lateral-velocity assumption.
struct ReentryQuintic
{
  std::array<double, 6> c{};
  static std::optional<ReentryQuintic> to_zero(double p, double v, double a, double t)
  {
    if (!std::isfinite(p) || !std::isfinite(v) || !std::isfinite(a) ||
        !std::isfinite(t) || t <= 0.0) return std::nullopt;
    ReentryQuintic q;
    q.c = {p, v, a / 2.0,
      -10.0*p/(t*t*t)-6.0*v/(t*t)-1.5*a/t,
      15.0*p/std::pow(t,4)+8.0*v/std::pow(t,3)+1.5*a/(t*t),
      -6.0*p/std::pow(t,5)-3.0*v/std::pow(t,4)-0.5*a/std::pow(t,3)};
    return q;
  }
  double at(double t, unsigned derivative = 0) const
  {
    double result = 0.0;
    for (unsigned i = derivative; i < c.size(); ++i) {
      double factor = c[i];
      for (unsigned j = 0; j < derivative; ++j) factor *= i-j;
      result += factor * std::pow(t, i-derivative);
    }
    return result;
  }
};

struct ReentryMotion
{
  double s_velocity{}, s_acceleration{}, d_velocity{}, d_acceleration{};
  double slip{}, slip_rate{};
};

inline std::optional<ReentryMotion> reentry_motion(
  double yaw_delta, double vx, double vy, double ax, double ay, double yaw_rate,
  double offset, double curvature, double curvature_derivative)
{
  for (double x : {yaw_delta,vx,vy,ax,ay,yaw_rate,offset,curvature,curvature_derivative}) {
    if (!std::isfinite(x)) return std::nullopt;
  }
  const double scale = 1.0-curvature*offset;
  if (scale <= 0.0 || vx < 0.0) return std::nullopt;
  const double cs=std::cos(yaw_delta), sn=std::sin(yaw_delta);
  ReentryMotion m;
  m.s_velocity=(vx*cs-vy*sn)/scale;
  m.d_velocity=vx*sn+vy*cs;
  // Accel is measured physical acceleration expressed in the vehicle frame,
  // matching localization_manager's MORAI acceleration input contract.
  const double world_ax=ax, world_ay=ay;
  const double tangent_acc=world_ax*cs-world_ay*sn;
  const double normal_acc=world_ax*sn+world_ay*cs;
  m.s_acceleration=(tangent_acc+curvature_derivative*offset*m.s_velocity*m.s_velocity+
    2.0*curvature*m.s_velocity*m.d_velocity)/scale;
  m.d_acceleration=normal_acc-curvature*scale*m.s_velocity*m.s_velocity;
  const double speed2=vx*vx+vy*vy;
  m.slip=std::atan2(vy,vx);
  m.slip_rate=speed2 > 1e-8 ? (vx*ay-vy*ax)/speed2-yaw_rate : 0.0;
  if (m.s_velocity < 0.0) return std::nullopt;
  return m;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif
