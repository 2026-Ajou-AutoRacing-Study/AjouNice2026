"""Map-based highway horizon eligibility without ROS dependencies.

This module decides where the requested highway policy applies; trajectory
generation and fallback remain owned by the caller. Legacy append experiments
require every route lanelet to be highway. Native long-horizon prediction can
classify the current start lanelet only, without constraining future branches.

``speed_limit=150`` is this map's geographic highway marker, NOT an estimate or
requirement for the NPC's speed.  No historical lanelet IDs are embedded here.
Confirmation is elapsed *track output classification* stability.  In particular,
30 Hz coasting outputs are not claimed to be fresh detector measurements.
"""

from __future__ import annotations

import math
import threading
from collections import OrderedDict
from dataclasses import dataclass
from typing import Iterable, Optional


# Keep the current pose plus future samples within the upstream Autoware
# PredictedPath capacity even though this ROS1 port exposes an unbounded array.
# This also bounds allocation before a ROS parameter can become a horizon list.
MAX_PREDICTED_PATH_POSES = 100


def build_highway_horizons(base_horizons, time_step, highway_prediction_horizon):
    """Build a bounded uniform highway horizon, or reject the optional mode.

    The ordinary predictor is not reconfigured here.  The caller catches a bad
    optional value and keeps its existing baseline, rather than refusing to
    start.  Exact baseline samples matter: recomputing its first three seconds
    at a different resolution would violate the wire-prefix contract.
    """

    if isinstance(time_step, bool) or isinstance(highway_prediction_horizon, bool):
        raise ValueError("mixed highway horizon/time step must be numeric")
    try:
        step = float(time_step)
        horizon = float(highway_prediction_horizon)
        base = tuple(base_horizons)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("mixed highway horizon/time step must be numeric") from error
    if not math.isfinite(step) or not math.isclose(step, 0.1, abs_tol=1.e-9):
        raise ValueError("mixed highway extension requires a 0.1s time step")
    expected_base = tuple(step * index for index in range(1, len(base) + 1))
    if not base or base != expected_base:
        raise ValueError("mixed highway extension requires a uniform positive baseline")
    maximum_horizon = step * (MAX_PREDICTED_PATH_POSES - 1)
    if (not math.isfinite(horizon) or horizon <= base[-1]
            or horizon > maximum_horizon + 1.e-6):
        raise ValueError("mixed highway horizon must exceed baseline and fit within 100 poses")
    count = int(round(horizon / step))
    if (count <= len(base) or count + 1 > MAX_PREDICTED_PATH_POSES
            or abs(count * step - horizon) > 1.e-6):
        raise ValueError("mixed highway horizon must be an integer 0.1s step within 100 poses")
    return tuple(step * index for index in range(1, count + 1))


@dataclass(frozen=True)
class HorizonDecision:
    extend: bool
    reason: str


@dataclass(frozen=True)
class _TrackState:
    entered_sec: float
    last_sec: float
    x: float
    y: float
    speed_mps: float


def _track_key(track_id) -> Optional[bytes]:
    """Accept only a nonzero, sixteen-byte UUID; never share an unknown ID."""

    if isinstance(track_id, (str, int)):
        return None
    try:
        values = tuple(track_id)
        if len(values) != 16 or any(
            isinstance(value, bool) or not isinstance(value, int)
            or value < 0 or value > 255 for value in values
        ):
            return None
        key = bytes(values)
    except (TypeError, ValueError, OverflowError):
        return None
    return key if any(key) else None


def _is_highway_record(record) -> bool:
    """Missing, malformed or contradictory optional attributes mean unknown."""

    try:
        attributes = getattr(record, "attributes", ())
        items = attributes.items() if hasattr(attributes, "items") else attributes
        speed_values = [str(value).strip() for key, value in items if key == "speed_limit"]
        return bool(speed_values) and all(
            math.isfinite(float(value)) and float(value) == 150.0
            for value in speed_values
        )
    except (TypeError, ValueError, OverflowError):
        return False


class MixedHorizonSelector:
    """Require unambiguous highway map context and stable track identity.

    Construct once for an immutable ``LaneletNetwork``.  Optional map metadata
    being absent is a usable, all-baseline mode, not a startup error.  The node
    should call ``prune(stamp_sec, active_track_ids)`` once per input message so
    disappearance immediately resets entry history.  Lazy expiration in
    ``decide`` also bounds state when that convenience call is unavailable.

    ``current_lanelet_only`` uses the original start lanelet of every candidate
    for native long-horizon prediction. Future branches or non-highway successors
    do not shorten that prediction. The default retains append-only research's
    stricter all-route eligibility rule.

    Entry confirmation takes 0.5 seconds by default for legacy callers. The
    production native caller selects 0.0 seconds: "highway confirmed" then means
    current map classification, not a new observation-count/time gate. Leaving
    the current highway region, unknown/ambiguous map context, a timestamp jump,
    disappearance or an implausible position jump removes history immediately.
    """

    def __init__(
        self,
        network,
        entry_confirm_sec: float = 0.5,
        state_timeout_sec: float = 2.0,
        maximum_tracks: int = 512,
        minimum_speed_mps: float = 0.5,
        current_lanelet_only: bool = False,
    ):
        self.entry_confirm_sec = float(entry_confirm_sec)
        self.state_timeout_sec = float(state_timeout_sec)
        self.minimum_speed_mps = float(minimum_speed_mps)
        if not isinstance(current_lanelet_only, bool):
            raise ValueError("current_lanelet_only must be boolean")
        self.current_lanelet_only = current_lanelet_only
        if not (
            math.isfinite(self.entry_confirm_sec) and self.entry_confirm_sec >= 0.0
            and math.isfinite(self.state_timeout_sec) and self.state_timeout_sec > 0.0
            and math.isfinite(self.minimum_speed_mps) and self.minimum_speed_mps > 0.0
        ):
            raise ValueError("mixed horizon times/speed must be finite and nonnegative")
        if isinstance(maximum_tracks, bool) or int(maximum_tracks) != maximum_tracks or maximum_tracks < 1:
            raise ValueError("maximum_tracks must be a positive integer")
        self.maximum_tracks = int(maximum_tracks)
        self._lock = threading.RLock()
        self._states = OrderedDict()
        self._last_stamp_sec = None

        # Classification is immutable map metadata, not per-frame geometry work.
        # Do not introduce an additional mandatory map file or external service.
        try:
            records = getattr(network, "records", {})
            self._known_lanelet_ids = frozenset(int(key) for key in records)
            self.highway_lanelet_ids = frozenset(
                int(key) for key, record in records.items() if _is_highway_record(record)
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._known_lanelet_ids = frozenset()
            self.highway_lanelet_ids = frozenset()
        self.metadata_available = bool(self.highway_lanelet_ids)
        self.metadata_reason = (
            "highway_lanelets_available" if self.metadata_available
            else "no_highway_metadata" if self._known_lanelet_ids
            else "no_map_records"
        )

    @property
    def state_count(self) -> int:
        with self._lock:
            return len(self._states)

    def _prune_locked(self, stamp_sec: float, active_track_ids=None) -> None:
        previous_stamp = self._last_stamp_sec
        if previous_stamp is not None and stamp_sec < previous_stamp:
            self._states.clear()
        self._last_stamp_sec = stamp_sec
        # All objects of one input message normally share a stamp.  Avoid an
        # O(tracks) expiry scan again for each object after the once-frame prune.
        if previous_stamp != stamp_sec:
            stale = [key for key, state in self._states.items()
                     if stamp_sec - state.last_sec > self.state_timeout_sec]
            for key in stale:
                del self._states[key]
        if active_track_ids is not None:
            active = {_track_key(value) for value in active_track_ids}
            for key in tuple(self._states):
                if key not in active:
                    del self._states[key]

    def prune(self, stamp_sec: float, active_track_ids: Optional[Iterable] = None) -> None:
        """Expire disappeared IDs and all confirmation on backwards ROS time."""

        try:
            stamp_sec = float(stamp_sec)
        except (TypeError, ValueError, OverflowError):
            stamp_sec = float("nan")
        with self._lock:
            if not math.isfinite(stamp_sec):
                self._states.clear()
                self._last_stamp_sec = None
                return
            self._prune_locked(stamp_sec, active_track_ids)

    def decide(
        self, track_id, stamp_sec: float, x: float, y: float,
        speed_mps: float, baseline_paths,
    ) -> HorizonDecision:
        """Return extension eligibility; an all-or-none baseline remains owned by caller."""

        key = _track_key(track_id)
        if key is None:
            return HorizonDecision(False, "invalid_track_id")
        try:
            stamp_sec, x, y, speed_mps = map(float, (stamp_sec, x, y, speed_mps))
        except (TypeError, ValueError, OverflowError):
            with self._lock:
                self._states.pop(key, None)
            return HorizonDecision(False, "invalid_state")
        if not all(math.isfinite(value) for value in (stamp_sec, x, y, speed_mps)):
            with self._lock:
                self._states.pop(key, None)
            return HorizonDecision(False, "invalid_state")

        with self._lock:
            self._prune_locked(stamp_sec)
            reason = self._ineligible_reason(speed_mps, baseline_paths)
            if reason:
                self._states.pop(key, None)
                return HorizonDecision(False, reason)

            previous = self._states.get(key)
            reset_reason = "entry_confirmation"
            if previous is not None:
                elapsed = stamp_sec - previous.last_sec
                maximum_travel = max(
                    10.0, 3.0 + 2.0 * max(abs(previous.speed_mps), abs(speed_mps)) * elapsed
                )
                if math.hypot(x - previous.x, y - previous.y) > maximum_travel:
                    previous = None
                    reset_reason = "position_jump_reset"
            entered_sec = previous.entered_sec if previous is not None else stamp_sec
            self._states[key] = _TrackState(entered_sec, stamp_sec, x, y, speed_mps)
            self._states.move_to_end(key)
            while len(self._states) > self.maximum_tracks:
                self._states.popitem(last=False)
            if stamp_sec - entered_sec + 1.0e-9 >= self.entry_confirm_sec:
                return HorizonDecision(True, "highway_confirmed")
            return HorizonDecision(False, reset_reason)

    def _ineligible_reason(self, speed_mps: float, baseline_paths) -> Optional[str]:
        if not self.metadata_available:
            return "map_metadata_unavailable"
        # Legacy append only extends a moving forward Map-CV route. Native
        # coverage is geographic: a stopped, slow or reversing vehicle can
        # retain the long horizon using the existing signed CTRV fallback.
        if not self.current_lanelet_only and speed_mps < self.minimum_speed_mps:
            return "low_or_reverse_speed"
        try:
            paths = tuple(baseline_paths)
            if not paths:
                return "no_baseline_map_paths"
            for path in paths:
                lanelet_ids = tuple(int(value) for value in path.lanelet_ids)
                if not lanelet_ids:
                    return "missing_route_lanelets"
                if self.current_lanelet_only:
                    lanelet_ids = lanelet_ids[:1]
                if any(value not in self._known_lanelet_ids for value in lanelet_ids):
                    return "route_unknown_lanelet"
                if any(value not in self.highway_lanelet_ids for value in lanelet_ids):
                    return "route_not_highway"
        except (AttributeError, TypeError, ValueError, OverflowError):
            return "invalid_baseline_map_paths"
        return None
