#!/usr/bin/env python3
"""Current profile normalization and legacy vote/hold contracts.

A/B red fallback: a1b5c4b8. D impossible R+G correction: c40da0ff.
D lamp-combination yellow identification (not prior phase): c650beb1.
These tests do not authorize a departure; the planner owns that decision.
"""

import json
import os
import sys


PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS = os.path.join(PACKAGE, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from signal_phase_policy import RESULT_TO_TOKEN, SignalPhasePolicy
from signal_smoother import SignalSmoother


POLICY = SignalPhasePolicy.from_yaml(
    os.path.join(PACKAGE, "config", "traffic_signal_profiles.yaml")
)


def _tokens(results):
    return {RESULT_TO_TOKEN[result] for result in results}


def test_observation_guard_d2_missing_and_observed_red_yellow_lifetime():
    from signal_phase_policy import TOKEN_TO_RESULT
    p = SignalPhasePolicy.from_yaml(os.path.join(PACKAGE, "config", "traffic_signal_profiles.yaml"))
    smoother = SignalSmoother(window_size=1)
    def step(tokens, time, missing=False):
        lamps = [TOKEN_TO_RESULT[t] for t in tokens]
        # process_normalized consumes profile output, not raw lone yellow.
        normalized, reason = p.apply([] if missing else lamps, 201326)
        result, _, _ = p.process_normalized(normalized, reason,
                                           201326, time, 1.0, smoother)
        return _tokens(result)
    left = ["RED_CIRCLE", "GREEN_LEFT_ARROW"]
    red_yellow = ["RED_CIRCLE", "YELLOW_CIRCLE"]
    assert step(left, 0) == set(left)
    assert step(red_yellow, .1) == set(red_yellow)
    assert step(["RED_CIRCLE"], .5, True) == set(red_yellow)
    assert step(red_yellow, .8) == set(red_yellow)
    assert step(["RED_CIRCLE"], .9) == {"RED_CIRCLE"}
    assert step(["YELLOW_CIRCLE"], 1) == {"RED_CIRCLE"}
    step(left, 2)
    assert step(["RED_CIRCLE"], 3.6, True) == {"RED_CIRCLE"}
    assert step(["YELLOW_CIRCLE"], 3.7) == {"RED_CIRCLE"}
    step(left, 4)
    for t in [4.1, 5, 6, 7, 8]:
        assert step(red_yellow, t) == set(red_yellow)
    # This existing 4 s observation bound is not remaining yellow time.
    assert step(red_yellow, 8.2) == {"RED_CIRCLE"}
    p.reset_transition()
    assert step(red_yellow, 9) == set(red_yellow)
    step(left, 10)
    assert step(["RED_CIRCLE"], 1, True) == {"RED_CIRCLE"}


def test_guard_seven_frame_vote_abstains_on_missing_observations():
    p = SignalPhasePolicy.from_yaml(os.path.join(PACKAGE, "config", "traffic_signal_profiles.yaml"))
    sm = SignalSmoother(7, 4)
    left = [("red", "circle", None), ("green", "arrow", True)]
    yellow = [("red", "circle", None), ("yellow", "circle", None)]
    red = [("red", "circle", None)]
    for i in range(7):
        p.process_normalized(left, "p:valid", 201326, i * .1, 1, sm)
    # Four genuine yellows, interrupted by failures, must not vote as red.
    for i, lamps in enumerate([yellow, red, red, yellow, red, red, yellow, red, yellow]):
        out, _, _ = p.process_normalized(lamps, "p:fail_open" if lamps == red else "p:valid",
                                         201326, .7 + i * .1, 1, sm)
        assert _tokens(out) != {"RED_CIRCLE"}
    assert out == yellow
    # A real red still wins the normal four-of-seven confirmation vote.
    for i in range(4):
        out, _, _ = p.process_normalized(red, "p:valid", 201326, 1.6 + i * .1, 1, sm)
    assert out == red


def test_assignments_are_course_profiles():
    assert POLICY.profile_name(201351) == "simple_3color"
    assert POLICY.profile_name(201365) == "simple_3color"
    assert POLICY.profile_name(201290) == "straight_left_combined"
    assert POLICY.profile_name(201326) == "straight_or_protected_left"
    assert POLICY.profile_name(201397) == "ignored_right_turn"


def test_assigned_group_ids_exist_in_route_matcher_table():
    table_path = os.path.join(PACKAGE, "traffic_light_group_id_table.json")
    with open(table_path, encoding="utf-8") as stream:
        table_ids = {int(item["group_id"]) for item in json.load(stream)["traffic_lights"]}
    assert set(POLICY.assignments).issubset(table_ids)


def test_ab_impossible_red_green_and_red_yellow_prefer_observed_red():
    red = ("red", "circle", None)
    for group in (201351, 201365):
        for other in (("green", "circle", None), ("yellow", "circle", None)):
            normalized, reason = POLICY.apply([red, other], group)
            assert _tokens(normalized) == {"RED_CIRCLE"}
            assert reason == "simple_3color:corrected"


def test_c_partial_green_is_completed_as_straight_left():
    for observed in (
        [("green", "circle", None)],
        [("green", "arrow", True)],
    ):
        normalized, _reason = POLICY.apply(observed, 201290)
        assert _tokens(normalized) == {"GREEN_CIRCLE", "GREEN_LEFT_ARROW"}


def test_c_impossible_red_green_prefers_red_and_missing_is_red():
    for observed in (
        [("red", "circle", None), ("green", "circle", None)],
        [("red", "circle", None), ("green", "arrow", True)],
        [],
    ):
        normalized, _reason = POLICY.apply(observed, 201290)
        assert _tokens(normalized) == {"RED_CIRCLE"}


def test_d2_impossible_red_green_circle_does_not_invent_protected_left():
    observed = [("red", "circle", None), ("green", "circle", None)]
    normalized, reason = POLICY.apply(observed, 201326)
    assert _tokens(normalized) == {"RED_CIRCLE"}
    assert reason == "straight_or_protected_left:corrected"
    # A genuinely observed arrow still has its profile-defined completion.
    normalized, reason = POLICY.apply([("green", "arrow", True)], 201326)
    assert _tokens(normalized) == {"RED_CIRCLE", "GREEN_LEFT_ARROW"}
    assert reason == "straight_or_protected_left:corrected"


def test_mission_profile_missing_fallback_is_red_despite_legacy_fail_open_reason():
    for group in (201351, 201365, 201290, 201326):
        normalized, reason = POLICY.apply([], group)
        assert _tokens(normalized) == {"RED_CIRCLE"}
        assert reason.endswith(":fail_open")
    unsupported = [("green", "arrow", True)]
    for group in (201351, 201365):
        normalized, reason = POLICY.apply(unsupported, group)
        assert _tokens(normalized) == {"RED_CIRCLE"}
        assert reason == "simple_3color:fail_open"


def test_d1_is_passthrough_because_planner_ignores_it():
    observed = [("red", "circle", None)]
    normalized, reason = POLICY.apply(observed, 201397)
    assert normalized == observed
    assert reason == "ignored_right_turn:passthrough"


def test_unsupported_arrow_does_not_override_valid_ab_red():
    observed = [("red", "circle", None), ("green", "arrow", True)]
    normalized, reason = POLICY.apply(observed, 201351)
    assert _tokens(normalized) == {"RED_CIRCLE"}
    assert reason.endswith(":filtered")


def test_d2_two_yellow_normalizations_are_independent_of_previous_phase():
    green = [("green", "circle", None)]
    yellow = [("yellow", "circle", None)]
    protected_left = [("red", "circle", None), ("green", "arrow", True)]

    red_yellow = [("red", "circle", None), ("yellow", "circle", None)]
    for preceding in (green, protected_left, [("red", "circle", None)], []):
        for observed, expected in ((yellow, {"RED_CIRCLE"}),
                                   (red_yellow, {"RED_CIRCLE", "YELLOW_CIRCLE"})):
            POLICY.reset_transition()
            POLICY.apply_transition(preceding, 201326)
            normalized, _ = POLICY.apply(observed, 201326)
            output, reason = POLICY.apply_transition(normalized, 201326)
            assert _tokens(output) == expected
            assert reason == "straight_or_protected_left:transition_valid"


def test_ab_and_c_preserve_yellow_observation_without_claiming_go_permission():
    yellow = [("yellow", "circle", None)]
    for group_id in (201351, 201365, 201290):
        POLICY.reset_transition()
        normalized, _ = POLICY.apply_transition(yellow, group_id)
        assert _tokens(normalized) == {"YELLOW_CIRCLE"}


def test_d2_full_sequence_with_smoothing_preserves_normalized_lamp_combinations():
    green = [("green", "circle", None)]
    yellow = [("yellow", "circle", None)]
    protected_left = [("red", "circle", None), ("green", "arrow", True)]
    red = [("red", "circle", None)]
    red_yellow = red + yellow
    smoother = SignalSmoother(window_size=7, majority_count=4)
    POLICY.reset_transition()

    stable_outputs = []
    for phase in (green, yellow, protected_left, red_yellow, red, green):
        output = None
        for _ in range(7):
            normalized, _ = POLICY.apply(phase, 201326)
            smoothed = smoother.update(normalized)
            output, _ = POLICY.apply_transition(smoothed or [], 201326)
        stable_outputs.append(_tokens(output))

    assert stable_outputs == [
        {"GREEN_CIRCLE"},
        {"RED_CIRCLE"},
        {"RED_CIRCLE", "GREEN_LEFT_ARROW"},
        {"RED_CIRCLE", "YELLOW_CIRCLE"},
        {"RED_CIRCLE"},
        {"GREEN_CIRCLE"},
    ]
