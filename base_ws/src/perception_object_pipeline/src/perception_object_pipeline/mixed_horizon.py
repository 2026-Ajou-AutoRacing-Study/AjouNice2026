"""Optional, conservative highway horizon eligibility without ROS dependencies.

The existing map prediction is always the baseline.  This module only decides
whether a track may request an extension; it never creates, deletes, reorders or
changes a trajectory.  The caller must also reject an extension that leaves the
eligible lanelets, and retain the complete baseline if extension fails.

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
    """Require unambiguous highway baseline routes and stable track identity.

    Construct once for an immutable ``LaneletNetwork``.  Optional map metadata
    being absent is a usable, all-baseline mode, not a startup error.  The node
    should call ``prune(stamp_sec, active_track_ids)`` once per input message so
    disappearance immediately resets entry history.  Lazy expiration in
    ``decide`` also bounds state when that convenience call is unavailable.

    Entry confirmation takes 0.5 seconds by default.  Leaving the highway,
    unknown/ambiguous routes, a timestamp jump, disappearance, a two-second gap,
    or an implausible position jump removes confirmation immediately.  No exit
    hold can keep a six-second path alive outside the highway region.
    """

    def __init__(
        self,
        network,
        entry_confirm_sec: float = 0.5,
        state_timeout_sec: float = 2.0,
        maximum_tracks: int = 512,
        minimum_speed_mps: float = 0.5,
    ):
        self.entry_confirm_sec = float(entry_confirm_sec)
        self.state_timeout_sec = float(state_timeout_sec)
        self.minimum_speed_mps = float(minimum_speed_mps)
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
        # Negative/reversing vehicles do not use the forward-lane map predictor.
        if speed_mps < self.minimum_speed_mps:
            return "low_or_reverse_speed"
        try:
            paths = tuple(baseline_paths)
            if not paths:
                return "no_baseline_map_paths"
            for path in paths:
                lanelet_ids = tuple(int(value) for value in path.lanelet_ids)
                if not lanelet_ids:
                    return "missing_route_lanelets"
                if any(value not in self._known_lanelet_ids for value in lanelet_ids):
                    return "route_unknown_lanelet"
                if any(value not in self.highway_lanelet_ids for value in lanelet_ids):
                    return "route_not_highway"
        except (AttributeError, TypeError, ValueError, OverflowError):
            return "invalid_baseline_map_paths"
        return None
