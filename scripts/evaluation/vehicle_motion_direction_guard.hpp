#ifndef AJOU_EVALUATION_VEHICLE_MOTION_DIRECTION_GUARD_HPP
#define AJOU_EVALUATION_VEHICLE_MOTION_DIRECTION_GUARD_HPP

// Research-only output policy. This does not mutate the EKF, associate an
// observation, force motion along a road, or enter the installed ROS node.
#include <cmath>
#include <deque>
#include <unordered_map>
#include <unordered_set>

namespace ajou_evaluation {

struct DirectionGuardConfig {
    double maximum_track_age_sec{2.0};
    double fit_window_sec{0.45};
    double minimum_fit_span_sec{0.30};
    double maximum_fit_residual_m{0.10};
    double minimum_motion_speed_mps{1.0};
    double maximum_hold_sec{0.40};
    double maximum_axis_error_rad{20.0 * M_PI / 180.0};
    // Distinguish a decision over measurement history from an output copy's
    // publication clock. This alternative is separately qualified, not a
    // silent increase of the original publication-time hold.
    bool measurement_time_decision{false};
    double maximum_output_age_sec{0.50};
};

struct DirectionGuardResult {
    double vx{0.0}, vy{0.0};
    double trusted_stamp{0.0}, fit_residual_m{-1.0};
    bool guarded{false}, fit_valid{false};
};

class VehicleMotionDirectionGuard {
public:
    explicit VehicleMotionDirectionGuard(DirectionGuardConfig config = {}) : config_(config) {}

    template <typename Observation>
    DirectionGuardResult Apply(int id, double stamp, double initialization_stamp,
                              double yaw, double vx, double vy, bool vehicle,
                              const std::deque<Observation>& history) {
        DirectionGuardResult result;
        result.vx = vx;
        result.vy = vy;
        if (history.empty()) { memory_.erase(id); return result; }
        const double observation_stamp = history.back().time_stamp;
        const double decision_stamp = config_.measurement_time_decision ? observation_stamp : stamp;
        const double age = decision_stamp - initialization_stamp;
        const double output_age = stamp - observation_stamp;
        if (!vehicle || !std::isfinite(age) || age < 0.0 ||
            age > config_.maximum_track_age_sec ||
            !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw)) {
            memory_.erase(id);
            return result;
        }
        if (config_.measurement_time_decision && (!std::isfinite(output_age) ||
            output_age < -1.e-6 || output_age > config_.maximum_output_age_sec)) return result;
        auto& memory = memory_[id];
        if (memory.initialization_stamp != initialization_stamp) {
            memory = Memory{};
            memory.initialization_stamp = initialization_stamp;
        }
        // A scheduler/coasting copy cannot add an observation or renew hold.
        if (observation_stamp > memory.last_observation_stamp) {
            memory.last_observation_stamp = observation_stamp;
            double t_mean = 0.0, x_mean = 0.0, y_mean = 0.0;
            int count = 0;
            double first_stamp = observation_stamp;
            for (const auto& p : history) {
                if (observation_stamp - p.time_stamp > config_.fit_window_sec + 1.e-6) continue;
                if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                    !std::isfinite(p.time_stamp)) return result;
                if (!count) first_stamp = p.time_stamp;
                // Relative time avoids epoch-scale cancellation.
                t_mean += p.time_stamp - observation_stamp;
                x_mean += p.x;
                y_mean += p.y;
                ++count;
            }
            memory.fit_valid = false;
            memory.residual_m = -1.0;
            if (count >= 4 && observation_stamp - first_stamp + 1.e-6 >=
                                  config_.minimum_fit_span_sec) {
                t_mean /= count; x_mean /= count; y_mean /= count;
                double tt = 0.0, tx = 0.0, ty = 0.0;
                for (const auto& p : history) {
                    if (observation_stamp - p.time_stamp > config_.fit_window_sec + 1.e-6) continue;
                    const double dt = p.time_stamp - observation_stamp - t_mean;
                    tt += dt * dt;
                    tx += dt * (p.x - x_mean); ty += dt * (p.y - y_mean);
                }
                if (tt > 1.e-9) {
                    const double fit_vx = tx / tt, fit_vy = ty / tt;
                    double residual_squared = 0.0;
                    for (const auto& p : history) {
                        if (observation_stamp - p.time_stamp > config_.fit_window_sec + 1.e-6) continue;
                        const double dt = p.time_stamp - observation_stamp - t_mean;
                        const double dx = p.x - x_mean - fit_vx * dt;
                        const double dy = p.y - y_mean - fit_vy * dt;
                        residual_squared += dx * dx + dy * dy;
                    }
                    memory.residual_m = std::sqrt(residual_squared / count);
                    memory.fit_valid = std::isfinite(memory.residual_m) &&
                        memory.residual_m <= config_.maximum_fit_residual_m;
                    const double speed = std::hypot(fit_vx, fit_vy);
                    if (memory.fit_valid) {
                        // Valid stopping/reverse observations release old
                        // direction rather than insisting on forward motion.
                        memory.has_trusted = speed >= config_.minimum_motion_speed_mps && speed <= 60.0;
                        if (memory.has_trusted) {
                            memory.unit_x = fit_vx / speed;
                            memory.unit_y = fit_vy / speed;
                            memory.trusted_stamp = observation_stamp;
                        }
                    }
                }
            }
        }
        result.fit_valid = memory.fit_valid;
        result.fit_residual_m = memory.residual_m;
        result.trusted_stamp = memory.trusted_stamp;
        const double speed = std::hypot(vx, vy);
        const double hold_age = decision_stamp - memory.trusted_stamp;
        if (memory.fit_valid || !memory.has_trusted || speed < 0.5 ||
            hold_age < 0.0 || hold_age > config_.maximum_hold_sec) return result;
        const double axis_dot = std::abs(std::cos(yaw) * memory.unit_x +
                                         std::sin(yaw) * memory.unit_y);
        const double motion_dot = (vx * memory.unit_x + vy * memory.unit_y) / speed;
        if (axis_dot < std::cos(config_.maximum_axis_error_rad) || motion_dot > -0.5) return result;
        // Preserve magnitude, not an old speed estimate. Do not invent a
        // precise acceleration prediction or change center/association state.
        result.vx = -vx; result.vy = -vy;
        result.guarded = true;
        return result;
    }

    void RetainOnly(const std::unordered_set<int>& active) {
        for (auto i = memory_.begin(); i != memory_.end();) {
            if (!active.count(i->first)) i = memory_.erase(i); else ++i;
        }
    }

private:
    struct Memory {
        double initialization_stamp{-1.0};
        double last_observation_stamp{-1.0}, trusted_stamp{0.0};
        double unit_x{0.0}, unit_y{0.0}, residual_m{-1.0};
        bool has_trusted{false}, fit_valid{false};
    };
    DirectionGuardConfig config_;
    std::unordered_map<int, Memory> memory_;
};

} // namespace ajou_evaluation
#endif
