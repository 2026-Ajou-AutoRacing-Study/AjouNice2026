#include <algorithm>
#include <cmath>
#include "purepursuit_pi/controller.hpp"

namespace purepursuit_pi {
double computePurePursuit(const EgoState& ego, const RefPoint& target, const VehicleModel& vehicle, double lookahead) {
  const double dx = target.x - ego.x;
  const double dy = target.y - ego.y;
  const double alpha = wrapAngle(std::atan2(dy, dx) - ego.yaw);
  const double ld = std::max(lookahead, 0.1);
  return std::atan2(2.0 * vehicle.wheelbase_m * std::sin(alpha), ld);
}
}  // namespace purepursuit_pi
