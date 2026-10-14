#pragma once

// Research-only recovery of a position offset admitted by the P+R gate.
// This file is never included by the installed tracker. Removing XY/dynamic
// cross-covariance prevents one recovered localization offset from becoming
// a fictitious instantaneous velocity observation. Principal covariance
// blocks remain, so a PSD prior remains PSD; future prediction rebuilds the
// physically meaningful position/velocity correlation. No signed velocity,
// heading, acceleration or track identity is overwritten.
#include <Eigen/Dense>
#include <cmath>

namespace vehicle_recovery {

template <class State, class Covariance, class MeasurementCovariance>
bool SeparatePosition(State& state, Covariance& covariance,
                      const Eigen::Vector2d& measurement,
                      const MeasurementCovariance& measurement_covariance,
                      const bool reset_position) {
    if (!state.allFinite() || !covariance.allFinite() ||
        !measurement.allFinite() || !measurement_covariance.allFinite()) {
        return false;
    }
    Eigen::Matrix2d r = measurement_covariance;
    r = (0.5 * (r + r.transpose())).eval();
    if (r(0, 0) <= 0.0 || r(1, 1) <= 0.0 || r.determinant() <= 1.e-12) {
        return false;
    }
    // Native state order: x,y,yaw,vx,vy,yaw_rate,ax,ay. Only x,y are reset.
    covariance.template block<2, 6>(0, 2).setZero();
    covariance.template block<6, 2>(2, 0).setZero();
    if (reset_position) {
        state.template head<2>() = measurement;
        covariance.template block<2, 2>(0, 0) = r;
    }
    return true;
}

}  // namespace vehicle_recovery
