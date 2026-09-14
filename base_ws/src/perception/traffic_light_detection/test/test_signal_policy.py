#!/usr/bin/env python3

import json
import os
import sys


PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS = os.path.join(PACKAGE, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from signal_phase_policy import RESULT_TO_TOKEN, SignalPhasePolicy


POLICY = SignalPhasePolicy.from_yaml(
    os.path.join(PACKAGE, "config", "traffic_signal_profiles.yaml")
)


def _tokens(results):
    return {RESULT_TO_TOKEN[result] for result in results}


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


def test_ab_red_green_prefers_go():
    observed = [("red", "circle", None), ("green", "circle", None)]
    normalized, reason = POLICY.apply(observed, 201351)
    assert _tokens(normalized) == {"GREEN_CIRCLE"}
    assert reason.endswith(":corrected")


def test_c_partial_green_is_completed_as_straight_left():
    for observed in (
        [("green", "circle", None)],
        [("green", "arrow", True)],
    ):
        normalized, _reason = POLICY.apply(observed, 201290)
        assert _tokens(normalized) == {"GREEN_CIRCLE", "GREEN_LEFT_ARROW"}


def test_d2_red_green_circle_becomes_protected_left():
    observed = [("red", "circle", None), ("green", "circle", None)]
    normalized, _reason = POLICY.apply(observed, 201326)
    assert _tokens(normalized) == {"RED_CIRCLE", "GREEN_LEFT_ARROW"}


def test_profiles_fail_open_with_route_compatible_phase():
    assert _tokens(POLICY.apply([], 201351)[0]) == {"GREEN_CIRCLE"}
    assert _tokens(POLICY.apply([], 201290)[0]) == {
        "GREEN_CIRCLE",
        "GREEN_LEFT_ARROW",
    }
    assert _tokens(POLICY.apply([], 201326)[0]) == {
        "RED_CIRCLE",
        "GREEN_LEFT_ARROW",
    }
    unsupported = [("green", "arrow", True)]
    assert _tokens(POLICY.apply(unsupported, 201351)[0]) == {"GREEN_CIRCLE"}


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
