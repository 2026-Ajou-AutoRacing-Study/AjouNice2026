#!/usr/bin/env python3

import os
import sys


SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from signal_conflict_resolver import resolve_conflict


def _has(values, color, shape):
    return any(c == color and s == shape for c, s, _direction in values)


def test_red_green_circle_becomes_protected_left_globally():
    observed = [("red", "circle", None), ("green", "circle", None)]

    for group_id in (201326, 201290, 999999):
        normalized = resolve_conflict(observed, gid=group_id)
        assert _has(normalized, "red", "circle")
        assert _has(normalized, "green", "arrow")
        assert not _has(normalized, "green", "circle")


def test_green_circle_and_left_arrow_remain_straight_left():
    observed = [("green", "circle", None), ("green", "arrow", True)]
    assert resolve_conflict(observed) == observed


def test_orphan_green_arrow_is_completed_as_straight_left():
    normalized = resolve_conflict([("green", "arrow", True)])
    assert _has(normalized, "green", "circle")
    assert _has(normalized, "green", "arrow")
