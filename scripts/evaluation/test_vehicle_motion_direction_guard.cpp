#include "vehicle_motion_direction_guard.hpp"
#include <cassert>
#include <iostream>

struct Point { double time_stamp, x, y; };

int main() {
    using ajou_evaluation::VehicleMotionDirectionGuard;
    auto linear = [](double speed, double offset = 0.0, double start = 100.0) {
        std::deque<Point> points;
        for (int i = 0; i < 5; ++i) points.push_back({start + .1 * i, offset + speed * .1 * i, 0.0});
        return points;
    };
    auto corrupted = [](std::deque<Point> points) {
        points.push_back({100.5, points.back().x - .9, 0.0});
        return points;
    };
    VehicleMotionDirectionGuard guard;
    auto forward = linear(2.0);
    auto good = guard.Apply(1,100.4,100.0,0.0,2.0,0.0,true,forward);
    assert(good.fit_valid && !good.guarded);
    auto bad = corrupted(forward);
    auto corrected = guard.Apply(1,100.5,100.0,0.0,-2.0,0.0,true,bad);
    assert(corrected.guarded && corrected.vx == 2.0);
    // Repeated 30Hz publication does not refresh measurement or trusted time.
    auto coast = guard.Apply(1,100.75,100.0,0.0,-2.0,0.0,true,bad);
    assert(coast.guarded && coast.trusted_stamp == 100.4);
    auto expired = guard.Apply(1,100.81,100.0,0.0,-2.0,0.0,true,bad);
    assert(!expired.guarded);
    // Native ID slot reuse must not inherit another actor's direction.
    auto reused = guard.Apply(1,100.5,100.49,0.0,-2.0,0.0,true,
                             std::deque<Point>{{100.49,0.0,0.0},{100.5,-.02,0.0}});
    assert(!reused.guarded);
    // Born reversing is supported, independent of the box heading sign.
    auto reverse = linear(-2.0);
    auto reversing = guard.Apply(2,100.4,100.0,0.0,-2.0,0.0,true,reverse);
    assert(reversing.fit_valid && !reversing.guarded && reversing.vx == -2.0);
    // A valid observed direction reversal replaces the reference immediately.
    auto clean_reverse = guard.Apply(2,100.9,100.0,0.0,2.0,0.0,true,linear(2.0,0.0,100.5));
    assert(clean_reverse.fit_valid && !clean_reverse.guarded && clean_reverse.trusted_stamp == 100.9);
    auto stopped = guard.Apply(3,100.4,100.0,0.0,0.0,0.0,true,linear(0.0));
    assert(stopped.fit_valid && !stopped.guarded);
    // A changing trajectory axis cannot be forced back along an old axis.
    guard.Apply(4,100.4,100.0,0.0,2.0,0.0,true,forward);
    auto turned = guard.Apply(4,100.5,100.0,M_PI/2.0,-2.0,0.0,true,bad);
    assert(!turned.guarded);
    auto mature = guard.Apply(5,100.5,97.0,0.0,-2.0,0.0,true,bad);
    assert(!mature.guarded);
    auto pedestrian = guard.Apply(6,100.5,100.0,0.0,-2.0,0.0,false,bad);
    assert(!pedestrian.guarded);
    guard.RetainOnly({});
    auto removed = guard.Apply(4,100.5,100.0,0.0,-2.0,0.0,true,bad);
    assert(!removed.guarded);
    ajou_evaluation::DirectionGuardConfig source_config;
    source_config.measurement_time_decision = true;
    VehicleMotionDirectionGuard source_guard(source_config);
    source_guard.Apply(1,100.65,100.0,0.0,2.0,0.0,true,forward);
    // Input age must not consume a measurement-history hold; coasting still
    // cannot renew either the trusted time or the input freshness limit.
    auto delayed = source_guard.Apply(1,100.85,100.0,0.0,-2.0,0.0,true,bad);
    assert(delayed.guarded && delayed.trusted_stamp == 100.4);
    auto stale_copy = source_guard.Apply(1,101.01,100.0,0.0,-2.0,0.0,true,bad);
    assert(!stale_copy.guarded);
    auto future_copy = source_guard.Apply(1,100.49,100.0,0.0,-2.0,0.0,true,bad);
    assert(!future_copy.guarded);
    std::cout << "direction guard boundary checks passed\n";
}
