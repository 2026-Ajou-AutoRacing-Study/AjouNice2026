#include "vehicle_covariance_recovery.hpp"
#include <Eigen/Eigenvalues>
#include <cassert>
#include <limits>

using State = Eigen::Matrix<double, 8, 1>;
using Covariance = Eigen::Matrix<double, 8, 8>;

int main() {
    Covariance factor = Covariance::Identity();
    factor(0, 3) = 2.0;
    factor(1, 4) = -1.5;
    factor(2, 5) = .4;
    const Covariance prior = factor * factor.transpose();
    State state;
    state << 10., 20., 1.2, -4., 3., -.2, .7, -1.;
    const State original = state;
    const Eigen::Vector2d measurement(12.5, 20.);
    const Eigen::Matrix2d r = .16 * Eigen::Matrix2d::Identity();

    Covariance separated = prior;
    assert(vehicle_recovery::SeparatePosition(state, separated, measurement, r, false));
    assert((state - original).norm() == 0.0);
    assert((separated.block<2, 2>(0, 0) - prior.block<2, 2>(0, 0)).norm() == 0.0);
    assert((separated.block<6, 6>(2, 2) - prior.block<6, 6>(2, 2)).norm() == 0.0);
    assert((separated.block<2, 6>(0, 2).norm() == 0.0));
    assert((separated.block<6, 2>(2, 0).norm() == 0.0));
    assert(Eigen::SelfAdjointEigenSolver<Covariance>(separated).eigenvalues().minCoeff() > 0.0);

    state = original;
    Covariance reset = prior;
    assert(vehicle_recovery::SeparatePosition(state, reset, measurement, r, true));
    assert((state.head<2>() - measurement).norm() == 0.0);
    assert((state.tail<6>() - original.tail<6>()).norm() == 0.0);
    assert((reset.block<2, 2>(0, 0) - r).norm() == 0.0);
    assert((reset.block<6, 6>(2, 2) - prior.block<6, 6>(2, 2)).norm() == 0.0);
    assert(Eigen::SelfAdjointEigenSolver<Covariance>(reset).eigenvalues().minCoeff() > 0.0);

    // Reset consumes the XY observation exactly once. Only yaw remains for
    // the normal EKF measurement update; R must not spuriously become R/2.
    Eigen::Matrix<double, 3, 8> h = Eigen::Matrix<double, 3, 8>::Zero();
    h(0, 0) = h(1, 1) = h(2, 2) = 1.;
    Eigen::Matrix3d measurement_r = Eigen::Matrix3d::Identity();
    measurement_r.block<2, 2>(0, 0) = r;
    Eigen::Matrix<double, 8, 3> k = reset * h.transpose() *
        (h * reset * h.transpose() + measurement_r).inverse();
    k.block<8, 2>(0, 0).setZero();
    const Covariance after_yaw = (Covariance::Identity() - k * h) * reset;
    assert((after_yaw.block<2, 2>(0, 0) - r).norm() == 0.0);
    assert(Eigen::SelfAdjointEigenSolver<Covariance>(after_yaw).eigenvalues().minCoeff() > 0.0);

    // Correlation is rebuilt by the motion model; this is not a permanent
    // zero-K velocity policy or a future velocity clamp.
    Covariance f = Covariance::Identity();
    f(0, 3) = .1;
    f(1, 4) = .1;
    const Covariance predicted = f * reset * f.transpose();
    assert(predicted(0, 3) > 0.0);
    assert(predicted(1, 4) > 0.0);
    assert(Eigen::SelfAdjointEigenSolver<Covariance>(predicted).eigenvalues().minCoeff() > 0.0);

    for (const bool reset_position : {false, true}) {
        state = original;
        Covariance invalid = prior;
        Eigen::Vector2d bad_measurement = measurement;
        bad_measurement(0) = std::numeric_limits<double>::quiet_NaN();
        assert(!vehicle_recovery::SeparatePosition(state, invalid, bad_measurement, r, reset_position));
        assert((state - original).norm() == 0.0 && (invalid - prior).norm() == 0.0);
        Eigen::Matrix2d bad_r = r;
        bad_r(0, 0) = -1.;
        assert(!vehicle_recovery::SeparatePosition(state, invalid, measurement, bad_r, reset_position));
        assert((state - original).norm() == 0.0 && (invalid - prior).norm() == 0.0);
        bad_r.setZero();
        assert(!vehicle_recovery::SeparatePosition(state, invalid, measurement, bad_r, reset_position));
        assert((state - original).norm() == 0.0 && (invalid - prior).norm() == 0.0);
    }
    return 0;
}
