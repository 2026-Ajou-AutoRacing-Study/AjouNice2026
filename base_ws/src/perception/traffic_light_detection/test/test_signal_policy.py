#!/usr/bin/env python3

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


def test_observation_guard_d2_missing_red_and_yellow_lifetime():
    from signal_phase_policy import TOKEN_TO_RESULT
    p = SignalPhasePolicy.from_yaml(os.path.join(PACKAGE, "config", "traffic_signal_profiles.yaml"))
    smoother = SignalSmoother(window_size=1)
    def step(tokens, time, missing=False):
        lamps = [TOKEN_TO_RESULT[t] for t in tokens]
        result, _, _ = p.process_normalized(lamps, "p:fail_open" if missing else "p:valid",
                                           201326, time, 1.0, smoother)
        return _tokens(result)
    left = ["RED_CIRCLE", "GREEN_LEFT_ARROW"]
    # D-2 delivers its yellow as RED+YELLOW; a lone YELLOW is corrected to red upstream.
    yellow = ["RED_CIRCLE", "YELLOW_CIRCLE"]
    assert step(left, 0) == set(left)
    assert step(yellow, .1) == set(yellow)
    # A missing frame abstains and the observed yellow is held within the gap.
    assert step(["RED_CIRCLE"], .5, True) == set(yellow)
    assert step(yellow, .8) == set(yellow)
    assert step(["RED_CIRCLE"], .9) == {"RED_CIRCLE"}
    assert step(yellow, 1) == set(yellow)
    step(left, 2)
    # No valid observation for more than max_observation_gap_sec (1.5 s).
    assert step(["RED_CIRCLE"], 3.6, True) == {"RED_CIRCLE"}
    assert step(yellow, 3.7) == set(yellow)
    step(left, 4)
    for t in [4.1, 5, 6, 7, 8]:
        assert step(yellow, t) == set(yellow)
    # Yellow older than max_yellow_sec (4.0 s) falls back to red.
    assert step(yellow, 8.2) == {"RED_CIRCLE"}
    p.reset_transition()
    assert step(yellow, 9) == set(yellow)
    step(left, 10)
    # A clock that runs backwards resets the guard instead of extending a lease.
    assert step(yellow, 1) == set(yellow)
    assert step(["RED_CIRCLE"], 2.6, True) == {"RED_CIRCLE"}


def test_guard_seven_frame_vote_abstains_on_missing_observations():
    p = SignalPhasePolicy.from_yaml(os.path.join(PACKAGE, "config", "traffic_signal_profiles.yaml"))
    sm = SignalSmoother(7, 4)
    left = [("red", "circle", None), ("green", "arrow", True)]
    yellow = [("yellow", "circle", None)]
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


def test_ab_impossible_red_green_and_red_yellow_prefer_red():
    for observed in (
        [("red", "circle", None), ("green", "circle", None)],
        [("red", "circle", None), ("yellow", "circle", None)],
    ):
        for group_id in (201351, 201365):
            normalized, reason = POLICY.apply(observed, group_id)
            assert _tokens(normalized) == {"RED_CIRCLE"}
            assert reason.endswith(":corrected")


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


def test_d2_impossible_combinations_prefer_the_safe_phase():
    cases = (
        ([("red", "circle", None), ("green", "circle", None)], {"RED_CIRCLE"}),
        ([("green", "circle", None), ("green", "arrow", True)], {"GREEN_CIRCLE"}),
        ([("green", "arrow", True)], {"RED_CIRCLE", "GREEN_LEFT_ARROW"}),
    )
    for observed, expected in cases:
        normalized, _reason = POLICY.apply(observed, 201326)
        assert _tokens(normalized) == expected


def test_profiles_fall_back_to_red_without_a_detection():
    for group_id in (201351, 201365, 201290, 201326):
        normalized, reason = POLICY.apply([], group_id)
        assert _tokens(normalized) == {"RED_CIRCLE"}
        assert reason.endswith(":fail_open")
    unsupported = [("green", "arrow", True)]
    assert _tokens(POLICY.apply(unsupported, 201351)[0]) == {"RED_CIRCLE"}


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


def test_d2_yellow_is_decided_by_lamp_combination_alone():
    straight_yellow = [("yellow", "circle", None)]
    left_yellow = [("red", "circle", None), ("yellow", "circle", None)]
    for previous in ([("green", "circle", None)], [("red", "circle", None), ("green", "arrow", True)]):
        POLICY.reset_transition()
        POLICY.apply_transition(POLICY.apply(previous, 201326)[0], 201326)
        normalized, reason = POLICY.apply(straight_yellow, 201326)
        assert _tokens(normalized) == {"RED_CIRCLE"}
        assert reason.endswith(":corrected")
        normalized, reason = POLICY.apply(left_yellow, 201326)
        assert _tokens(normalized) == {"RED_CIRCLE", "YELLOW_CIRCLE"}
        assert reason.endswith(":valid")
        assert POLICY.apply_transition(normalized, 201326)[0] == normalized


def test_b_and_c_yellow_remain_go_phases_without_conditional_rewrite():
    yellow = [("yellow", "circle", None)]
    for group_id in (201351, 201365, 201290):
        POLICY.reset_transition()
        normalized, _ = POLICY.apply_transition(yellow, group_id)
        assert _tokens(normalized) == {"YELLOW_CIRCLE"}


def test_d2_full_sequence_with_smoothing_has_only_protected_left_yellow_as_go():
    green = [("green", "circle", None)]
    yellow = [("yellow", "circle", None)]
    protected_left = [("red", "circle", None), ("green", "arrow", True)]
    left_yellow = [("red", "circle", None), ("yellow", "circle", None)]
    red = [("red", "circle", None)]
    smoother = SignalSmoother(window_size=7, majority_count=4)
    POLICY.reset_transition()

    stable_outputs = []
    for phase in (green, yellow, red, protected_left, left_yellow, red, green):
        output = None
        for _ in range(7):
            normalized, _ = POLICY.apply(phase, 201326)
            smoothed = smoother.update(normalized)
            output, _ = POLICY.apply_transition(smoothed or [], 201326)
        stable_outputs.append(_tokens(output))

    assert stable_outputs == [
        {"GREEN_CIRCLE"},
        {"RED_CIRCLE"},
        {"RED_CIRCLE"},
        {"RED_CIRCLE", "GREEN_LEFT_ARROW"},
        {"RED_CIRCLE", "YELLOW_CIRCLE"},
        {"RED_CIRCLE"},
        {"GREEN_CIRCLE"},
    ]
