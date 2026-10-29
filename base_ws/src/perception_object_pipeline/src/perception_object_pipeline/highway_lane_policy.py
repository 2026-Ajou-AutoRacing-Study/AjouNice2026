"""Observed-current-lane selection for the optional highway single-path policy.

The caller owns geographic eligibility. This helper has no highway IDs, NPC
goals, confidence thresholds, ego route or future/GT positions. It selects the
lane containing the *current* center, retaining that choice through a small
spatial boundary margin. Forward seams transfer to the geometric continuation;
the old lane cannot remain selected after its longitudinal end.
"""

from __future__ import annotations

import math
import threading
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

from .map_based_prediction import normalize_angle, polyline_length, tangent_yaw
from .mixed_horizon import _track_key


@dataclass(frozen=True)
class HighwayLaneDecision:
    lanelet_id: Optional[int]
    reason: str


@dataclass(frozen=True)
class _LaneObservation:
    lanelet_id: int
    distance: float
    yaw_error: float
    boundary_distance: Optional[float]
    longitudinal_outside: float

    @property
    def inside(self):
        return (
            self.boundary_distance is not None
            and self.boundary_distance >= -1.e-8
            and self.longitudinal_outside <= 1.e-8
        )


@dataclass(frozen=True)
class _LaneState:
    lanelet_id: int
    stamp_sec: float
    x: float
    y: float
    speed_mps: float


def _polygon_distance(polygon, x, y):
    """Signed XY distance: positive inside, zero boundary, negative outside."""
    if len(polygon) < 3:
        return None
    inside = False
    closest_sq = float("inf")
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        squared = dx * dx + dy * dy
        if squared <= 1.e-18:
            continue
        ratio = max(0., min(1., ((x - first[0]) * dx + (y - first[1]) * dy) / squared))
        closest_sq = min(closest_sq, (x - first[0] - ratio * dx) ** 2
                         + (y - first[1] - ratio * dy) ** 2)
        if (first[1] > y) != (second[1] > y):
            crossing_x = first[0] + (y - first[1]) * dx / dy
            if x < crossing_x:
                inside = not inside
    if not math.isfinite(closest_sq):
        return None
    distance = math.sqrt(closest_sq)
    if distance <= 1.e-8:
        return 0.
    return distance if inside else -distance


class HighwayCurrentLanePolicy:
    """Bounded, per-UUID lane membership history; no time-window GO decisions.

    ``prune`` is called once per input frame. Invalid/backwards stamps, absent
    IDs, a long input gap or an implausible jump remove retained membership.
    ``forget`` lets the caller immediately release a track leaving its policy
    region. Missing optional polygon geometry falls back to current nearest
    centerline geometry, never a path's confidence.
    """

    def __init__(
        self, network, boundary_hysteresis_m=0.2, state_timeout_sec=1.0,
        maximum_tracks=512,
    ):
        self.network = network
        if isinstance(boundary_hysteresis_m, bool) or isinstance(state_timeout_sec, bool):
            raise ValueError("current-lane margin/timeout must be numeric")
        try:
            self.boundary_hysteresis_m = float(boundary_hysteresis_m)
            self.state_timeout_sec = float(state_timeout_sec)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("current-lane margin/timeout must be numeric") from error
        if (not math.isfinite(self.boundary_hysteresis_m)
                or self.boundary_hysteresis_m < 0.
                or not math.isfinite(self.state_timeout_sec)
                or self.state_timeout_sec <= 0.):
            raise ValueError("current-lane margin/timeout must be finite and nonnegative")
        if (isinstance(maximum_tracks, bool) or int(maximum_tracks) != maximum_tracks
                or maximum_tracks < 1):
            raise ValueError("maximum_tracks must be a positive integer")
        self.maximum_tracks = int(maximum_tracks)
        self._states = OrderedDict()
        self._geometry = {}
        self._last_stamp_sec = None
        self._lock = threading.RLock()

    @property
    def state_count(self):
        with self._lock:
            return len(self._states)

    def forget(self, track_id):
        with self._lock:
            self._states.pop(_track_key(track_id), None)

    def _prune_locked(self, stamp_sec, active_track_ids=None):
        previous_stamp = self._last_stamp_sec
        if previous_stamp is not None and stamp_sec < previous_stamp:
            self._states.clear()
        self._last_stamp_sec = stamp_sec
        if previous_stamp != stamp_sec:
            for key, state in tuple(self._states.items()):
                if stamp_sec - state.stamp_sec > self.state_timeout_sec:
                    del self._states[key]
        if active_track_ids is not None:
            active = {_track_key(value) for value in active_track_ids}
            for key in tuple(self._states):
                if key not in active:
                    del self._states[key]

    def prune(self, stamp_sec, active_track_ids=None):
        try:
            stamp_sec = float(stamp_sec)
        except (TypeError, ValueError, OverflowError):
            stamp_sec = float("nan")
        with self._lock:
            if not math.isfinite(stamp_sec):
                self._states.clear()
                self._last_stamp_sec = None
            else:
                self._prune_locked(stamp_sec, active_track_ids)

    def _record_geometry(self, lanelet_id):
        record = self.network.records[lanelet_id]
        cached = self._geometry.get(lanelet_id)
        if cached is not None and cached[0] is record:
            return cached[1:]
        line = record.centerline
        length = polyline_length(line)
        left = tuple(getattr(record, "left_boundary", ()))
        right = tuple(getattr(record, "right_boundary", ()))
        polygon = left + tuple(reversed(right)) if len(left) >= 2 and len(right) >= 2 else ()
        geometry = (
            polygon, length, tangent_yaw(line, 0.), tangent_yaw(line, length),
        )
        self._geometry[lanelet_id] = (record,) + geometry
        return geometry

    def _observations(self, x, y, yaw, previous):
        ids = set(self.network._nearby_record_ids(x, y))
        if previous is not None:
            ids.add(previous.lanelet_id)
            record = self.network.records.get(previous.lanelet_id)
            if record is not None:
                ids.update(record.following_ids)
        observations = []
        for lanelet_id in sorted(ids):
            if lanelet_id not in self.network.records:
                continue
            projection = self.network.project_lanelet(lanelet_id, x, y)
            if (projection is None or not math.isfinite(projection.distance)
                    or projection.distance > self.network.config.maximum_lane_distance_m):
                continue
            yaw_error = abs(normalize_angle(yaw - projection.yaw))
            if yaw_error > self.network.config.maximum_lane_yaw_error_rad:
                continue
            polygon, length, start_yaw, end_yaw = self._record_geometry(lanelet_id)
            line = self.network.records[lanelet_id].centerline
            longitudinal_outside = 0.
            if projection.arc_length <= 1.e-8:
                longitudinal_outside = max(longitudinal_outside, -(
                    (x - line[0][0]) * math.cos(start_yaw)
                    + (y - line[0][1]) * math.sin(start_yaw)
                ))
            if projection.arc_length >= length - 1.e-8:
                longitudinal_outside = max(longitudinal_outside,
                    (x - line[-1][0]) * math.cos(end_yaw)
                    + (y - line[-1][1]) * math.sin(end_yaw))
            observations.append(_LaneObservation(
                lanelet_id, projection.distance, yaw_error,
                _polygon_distance(polygon, x, y), longitudinal_outside,
            ))
        return observations

    def select(self, track_id, stamp_sec, x, y, yaw, speed_mps):
        key = _track_key(track_id)
        if key is None:
            return HighwayLaneDecision(None, "invalid_track_id")
        try:
            stamp_sec, x, y, yaw, speed_mps = map(float, (stamp_sec, x, y, yaw, speed_mps))
        except (TypeError, ValueError, OverflowError):
            self.forget(track_id)
            return HighwayLaneDecision(None, "invalid_state")
        if (not all(math.isfinite(value) for value in (stamp_sec, x, y, yaw, speed_mps))
                or speed_mps < 0.):
            self.forget(track_id)
            return HighwayLaneDecision(None, "invalid_state")
        with self._lock:
            self._prune_locked(stamp_sec)
            previous = self._states.get(key)
            if previous is not None:
                elapsed = max(0., stamp_sec - previous.stamp_sec)
                maximum_travel = max(10., 3. + 2. * max(speed_mps, previous.speed_mps) * elapsed)
                if math.hypot(x - previous.x, y - previous.y) > maximum_travel:
                    previous = None
            observations = self._observations(x, y, yaw, previous)
            if not observations:
                self._states.pop(key, None)
                return HighwayLaneDecision(None, "no_current_lane")
            by_id = {value.lanelet_id: value for value in observations}
            retained = by_id.get(previous.lanelet_id) if previous is not None else None
            choice = None
            reason = "current_polygon"
            if retained is not None and retained.longitudinal_outside <= 1.e-8:
                if (retained.boundary_distance is not None
                        and retained.boundary_distance >= -self.boundary_hysteresis_m):
                    choice, reason = retained, "retained_current_lane"
                elif retained.boundary_distance is None:
                    closest = min(observations, key=lambda value: (value.distance, value.yaw_error))
                    if retained.distance <= closest.distance + self.boundary_hysteresis_m:
                        choice, reason = retained, "retained_centerline"
            if choice is retained and retained is not None and retained.inside:
                # Some maps contain a straight virtual lane polygon laid over
                # a widening physical successor. Membership in the old virtual
                # polygon alone must not pin the object to it until the overlap
                # ends. Where BOTH polygons contain the observed current center,
                # switch only after the other centerline is closer by the same
                # spatial margin. This uses no future sample or path confidence.
                overlapping = [value for value in observations
                               if value.lanelet_id != retained.lanelet_id
                               and value.inside
                               and value.distance + self.boundary_hysteresis_m
                               < retained.distance]
                if overlapping:
                    choice = min(overlapping, key=lambda value: (
                        value.distance, value.yaw_error, value.lanelet_id,
                    ))
                    reason = "overlap_current_center"
            if choice is None and previous is not None:
                # At a forward seam, overlapping sibling polygons must not
                # make a tiny centerline-distance difference choose intention.
                # Retain the incoming lane's straight continuation only while
                # the *observed current center* still lies in its polygon.
                successor = self.network.continuation_lanelet_id(previous.lanelet_id)
                following = by_id.get(successor)
                previous_observation = by_id.get(previous.lanelet_id)
                past_end = (previous_observation is None
                            or previous_observation.longitudinal_outside > 1.e-8)
                if past_end and following is not None and following.inside:
                    choice, reason = following, "forward_lane_seam"
            if choice is None:
                contained = [value for value in observations if value.inside]
                if contained:
                    choice = min(contained, key=lambda value: (
                        value.distance, value.yaw_error, value.lanelet_id,
                    ))
                else:
                    # Lanelets constructed without boundaries are supported
                    # for legacy maps/tests. With polygons present, do not
                    # snap an out-of-road center onto a neighbouring lane.
                    without_polygon = [value for value in observations
                                       if value.boundary_distance is None
                                       and value.longitudinal_outside <= 1.e-8]
                    if without_polygon:
                        choice = min(without_polygon, key=lambda value: (
                            value.distance, value.yaw_error, value.lanelet_id,
                        ))
                        reason = "nearest_centerline"
            if choice is None:
                self._states.pop(key, None)
                return HighwayLaneDecision(None, "outside_lane_polygons")
            self._states[key] = _LaneState(choice.lanelet_id, stamp_sec, x, y, speed_mps)
            self._states.move_to_end(key)
            while len(self._states) > self.maximum_tracks:
                self._states.popitem(last=False)
            return HighwayLaneDecision(choice.lanelet_id, reason)
