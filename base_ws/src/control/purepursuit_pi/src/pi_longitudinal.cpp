#include <algorithm>
#include "purepursuit_pi/controller.hpp"

namespace purepursuit_pi {
double computePI(double error, double target_acceleration, double dt, const LongitudinalConfig& config, double* integral) {
  if (std::abs(error) < config.deadband) error = 0.0;
  *integral = std::max(-config.integral_limit, std::min(config.integral_limit, *integral + error * dt));
  double output = config.kp * error + config.ki * (*integral);
  if (config.use_ff) output += target_acceleration;
  // Conditional integration keeps the stored integrator from driving a saturated actuator further out.
  const double upper = config.accel_limit / std::max(config.accel_gain, 1e-6);
  const double lower = -config.brake_limit / std::max(config.brake_gain, 1e-6);
  if ((output >= upper && error > 0.0) || (output <= lower && error < 0.0)) {
    *integral = std::max(-config.integral_limit, std::min(config.integral_limit, *integral - error * dt));
    output = config.kp * error + config.ki * (*integral) + (config.use_ff ? target_acceleration : 0.0);
  }
  return output;
}
}  // namespace purepursuit_pi
