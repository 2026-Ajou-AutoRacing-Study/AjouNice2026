"""Lanelet2-backed vehicle prediction core independent from ROS callbacks.

The implementation intentionally keeps the ROS2 Autoware algorithm boundary:
lane matching, routing-graph path enumeration and path confidence live here,
while message conversion stays in the ROS1 node.
"""

from __future__ import annotations

import hashlib
import math
import ctypes
import threading
from bisect import bisect_left
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Sequence, Tuple

import yaml


Point3 = Tuple[float, float, float]


_AUTOWARE_LANELET2_EXTENSION_HANDLE = None


def _load_autoware_lanelet2_extension() -> None:
    """Register Autoware-specific Lanelet2 regulatory elements.

    The Python Lanelet2 bindings only know the upstream regulatory elements.
    Loading the ported extension library registers additional map primitives,
    including ``roundabout``, in Lanelet2's process-global factory.
    """

    global _AUTOWARE_LANELET2_EXTENSION_HANDLE
    if _AUTOWARE_LANELET2_EXTENSION_HANDLE is not None:
        return
    try:
        _AUTOWARE_LANELET2_EXTENSION_HANDLE = ctypes.CDLL(
            "libautoware_lanelet2_extension_lib.so", mode=ctypes.RTLD_GLOBAL
        )
    except OSError as exc:
        raise RuntimeError(
            "Autoware Lanelet2 extension library is unavailable. Source the "
            "AW149 runtime overlay before starting map-based prediction."
        ) from exc


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance(a: Point3, b: Point3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass(frozen=True)
class PolylineProjection:
    x: float
    y: float
    z: float
    yaw: float
    distance: float
    arc_length: float


def project_point_to_polyline(
    points: Sequence[Point3], x: float, y: float
) -> Optional[PolylineProjection]:
    if len(points) < 2:
        return None
    best = None
    walked = 0.0
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        segment_length_sq = dx * dx + dy * dy
        segment_length = math.sqrt(segment_length_sq)
        if segment_length < 1.0e-9:
            continue
        ratio = ((x - first[0]) * dx + (y - first[1]) * dy) / segment_length_sq
        ratio = max(0.0, min(1.0, ratio))
        projected_x = first[0] + ratio * dx
        projected_y = first[1] + ratio * dy
        projected_z = first[2] + ratio * (second[2] - first[2])
        distance = math.hypot(x - projected_x, y - projected_y)
        candidate = PolylineProjection(
            x=projected_x,
            y=projected_y,
            z=projected_z,
            yaw=math.atan2(dy, dx),
            distance=distance,
            arc_length=walked + ratio * segment_length,
        )
        if best is None or candidate.distance < best.distance:
            best = candidate
        walked += segment_length
    return best


class _ProjectionPolyline:
    """Immutable segment invariants; retain scalar projection/tie semantics.

    Only map geometry is cached. The projected point, minimum distance and
    lane match still depend on the current observation and are never reused.
    """

    def __init__(self, points: Sequence[Point3]):
        self.segments = []
        walked = 0.0
        for first, second in zip(points, points[1:]):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            length_sq = dx * dx + dy * dy
            length = math.sqrt(length_sq)
            if length < 1.0e-9:
                continue
            self.segments.append((first, dx, dy, second[2] - first[2],
                                  length_sq, length, math.atan2(dy, dx), walked))
            walked += length

    def project(self, x: float, y: float) -> Optional[PolylineProjection]:
        best = None
        best_distance = None
        for first, dx, dy, dz, length_sq, length, yaw, walked in self.segments:
            ratio = ((x - first[0]) * dx + (y - first[1]) * dy) / length_sq
            ratio = max(0.0, min(1.0, ratio))
            projected_x = first[0] + ratio * dx
            projected_y = first[1] + ratio * dy
            distance = math.hypot(x - projected_x, y - projected_y)
            # Strict comparison keeps the first segment when distances tie,
            # just like project_point_to_polyline (including NaN behavior).
            if best is None or distance < best_distance:
                best_distance = distance
                best = (projected_x, projected_y, first[2] + ratio * dz,
                        yaw, distance, walked + ratio * length)
        return PolylineProjection(*best) if best is not None else None


def polyline_length(points: Sequence[Point3]) -> float:
    return sum(_distance(a, b) for a, b in zip(points, points[1:]))


def sample_polyline(points: Sequence[Point3], distance: float) -> Point3:
    if not points:
        raise ValueError("cannot sample an empty polyline")
    if len(points) == 1:
        return points[0]
    target = max(0.0, float(distance))
    walked = 0.0
    for first, second in zip(points, points[1:]):
        segment_length = _distance(first, second)
        if segment_length < 1.0e-9:
            continue
        if walked + segment_length >= target:
            ratio = (target - walked) / segment_length
            return (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
                first[2] + ratio * (second[2] - first[2]),
            )
        walked += segment_length
    return points[-1]


def tangent_yaw(points: Sequence[Point3], distance: float, epsilon: float = 0.25) -> float:
    length = polyline_length(points)
    before = sample_polyline(points, max(0.0, distance - epsilon))
    after = sample_polyline(points, min(length, distance + epsilon))
    if _distance(before, after) < 1.0e-9:
        if len(points) < 2:
            return 0.0
        before, after = points[-2], points[-1]
    return math.atan2(after[1] - before[1], after[0] - before[0])


class _RoutePolyline:
    """Per-route arc index with the original scalar interpolation semantics.

    Geometry is immutable for one prediction call. Cache lengths, not motion
    or route decisions; avoid rescanning every segment for every future pose.
    Tiny segments remain in total length but are skipped by sampling, exactly
    as in polyline_length/sample_polyline above.
    """

    def __init__(self, points: Sequence[Point3]):
        self.points = points
        lengths = [_distance(a, b) for a, b in zip(points, points[1:])]
        self.length = sum(lengths)
        self.ends = []
        self.segments = []
        walked = 0.0
        for first, second, length in zip(points, points[1:], lengths):
            if length < 1.0e-9:
                continue
            self.segments.append((first, second, walked, length))
            walked += length
            self.ends.append(walked)

    def sample(self, distance: float) -> Point3:
        if not self.points:
            raise ValueError("cannot sample an empty polyline")
        if len(self.points) == 1:
            return self.points[0]
        target = max(0.0, float(distance))
        index = bisect_left(self.ends, target)
        if index == len(self.segments):
            return self.points[-1]
        first, second, walked, length = self.segments[index]
        ratio = (target - walked) / length
        return tuple(first[i] + ratio * (second[i] - first[i]) for i in range(3))

    def tangent(self, distance: float, epsilon: float = 0.25) -> float:
        before = self.sample(max(0.0, distance - epsilon))
        after = self.sample(min(self.length, distance + epsilon))
        if _distance(before, after) < 1.0e-9:
            if len(self.points) < 2:
                return 0.0
            before, after = self.points[-2], self.points[-1]
        return math.atan2(after[1] - before[1], after[0] - before[0])


@dataclass(frozen=True)
class LaneletRecord:
    lanelet_id: int
    centerline: Tuple[Point3, ...]
    following_ids: Tuple[int, ...] = ()
    attributes: Tuple[Tuple[str, str], ...] = ()
    # Optional for lightweight callers constructing synthetic centerlines.
    # Loaded maps retain the actual polygon so current-lane policies do not
    # confuse a close parallel centerline with the lane containing the object.
    left_boundary: Tuple[Point3, ...] = ()
    right_boundary: Tuple[Point3, ...] = ()

    @property
    def length(self) -> float:
        return polyline_length(self.centerline)


@dataclass(frozen=True)
class MapPredictionConfig:
    maximum_lane_distance_m: float = 3.0
    maximum_lane_yaw_error_rad: float = math.radians(45.0)
    lane_distance_sigma_m: float = 0.75
    lane_yaw_sigma_rad: float = math.radians(15.0)
    minimum_speed_mps: float = 0.5
    lateral_convergence_time_sec: float = 1.5
    route_length_padding_m: float = 3.0
    maximum_nearest_lanelets: int = 12
    maximum_start_lanelets: int = 3
    maximum_paths: int = 6
    maximum_route_depth: int = 12
    trajectory_cluster_distance_m: float = 0.75


@dataclass(frozen=True)
class PredictedTrajectory:
    poses: Tuple[Tuple[float, float, float, float], ...]
    confidence: float
    lanelet_ids: Tuple[int, ...]


@dataclass(frozen=True)
class _LaneCandidate:
    lanelet_id: int
    projection: PolylineProjection
    score: float


class LaneletNetwork:
    def __init__(
        self,
        records: Iterable[LaneletRecord],
        config: MapPredictionConfig | None = None,
        map_sha256: str = "",
        lanelet_layer=None,
    ):
        self.config = config or MapPredictionConfig()
        self.records: Dict[int, LaneletRecord] = {
            int(record.lanelet_id): record for record in records
        }
        if not self.records:
            raise ValueError("lanelet network contains no vehicle road lanelets")
        self.map_sha256 = map_sha256
        self._lanelet_layer = lanelet_layer
        self._spatial_cell_size = max(
            5.0, 2.0 * float(self.config.maximum_lane_distance_m)
        )
        self._spatial_index = self._build_spatial_index()
        # The map's frozen LaneletRecord geometry is constant for this network.
        # Cache invariants, never observations, route scores or future poses.
        self._projection_geometry = {
            lanelet_id: _ProjectionPolyline(record.centerline)
            for lanelet_id, record in self.records.items()
        }
        self._lane_lengths = {
            lanelet_id: record.length for lanelet_id, record in self.records.items()
        }
        self._geometry_records = dict(self.records)
        self._transition_scores = OrderedDict()
        self._transition_score_cache_limit = 512
        self._route_geometry_cache = OrderedDict()
        self._route_geometry_cache_limit = 128
        # rospy may invoke one callback per publisher connection. Keep the
        # get/move/evict LRU operations atomic; geometry work is outside locks.
        self._geometry_cache_lock = threading.RLock()

    def _refresh_lane_geometry(self, lanelet_id: int) -> None:
        record = self.records[lanelet_id]
        if self._geometry_records.get(lanelet_id) is record:
            return
        # Production creates a new network on map load. Also keep diagnostics
        # correct when they explicitly replace a frozen LaneletRecord.
        self._geometry_records[lanelet_id] = record
        self._projection_geometry[lanelet_id] = _ProjectionPolyline(record.centerline)
        self._lane_lengths[lanelet_id] = record.length

    def _lane_length(self, lanelet_id: int) -> float:
        self._refresh_lane_geometry(lanelet_id)
        return self._lane_lengths[lanelet_id]

    def _grid_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor(x / self._spatial_cell_size)),
            int(math.floor(y / self._spatial_cell_size)),
        )

    def _build_spatial_index(self) -> Dict[Tuple[int, int], Tuple[int, ...]]:
        cells: Dict[Tuple[int, int], set] = {}
        margin = max(0.0, float(self.config.maximum_lane_distance_m))
        for lanelet_id, record in self.records.items():
            for first, second in zip(record.centerline, record.centerline[1:]):
                minimum = self._grid_cell(
                    min(first[0], second[0]) - margin,
                    min(first[1], second[1]) - margin,
                )
                maximum = self._grid_cell(
                    max(first[0], second[0]) + margin,
                    max(first[1], second[1]) + margin,
                )
                for cell_x in range(minimum[0], maximum[0] + 1):
                    for cell_y in range(minimum[1], maximum[1] + 1):
                        cells.setdefault((cell_x, cell_y), set()).add(lanelet_id)
        return {cell: tuple(sorted(ids)) for cell, ids in cells.items()}

    @classmethod
    def load(
        cls,
        map_file: str | Path,
        projector_file: str | Path,
        config: MapPredictionConfig | None = None,
    ) -> "LaneletNetwork":
        # Import lazily so pure geometry tests do not require a sourced ROS setup.
        _load_autoware_lanelet2_extension()
        import lanelet2

        map_path = Path(map_file).expanduser().resolve()
        projector_path = Path(projector_file).expanduser().resolve()
        if not map_path.is_file():
            raise FileNotFoundError(f"Lanelet2 map does not exist: {map_path}")
        if not projector_path.is_file():
            raise FileNotFoundError(f"map projector does not exist: {projector_path}")

        projector_data = yaml.safe_load(projector_path.read_text(encoding="utf-8")) or {}
        if projector_data.get("projector_type") != "LocalCartesianUTM":
            raise ValueError(
                "map-based predictor currently requires LocalCartesianUTM; got "
                f"{projector_data.get('projector_type')!r}"
            )
        origin_data = projector_data.get("map_origin") or {}
        origin = lanelet2.io.Origin(
            float(origin_data["latitude"]), float(origin_data["longitude"])
        )
        lanelet_map = lanelet2.io.load(
            str(map_path), lanelet2.projection.UtmProjector(origin)
        )
        rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)

        vehicle_lanelets = {}
        for lanelet in lanelet_map.laneletLayer:
            subtype = lanelet.attributes["subtype"] if "subtype" in lanelet.attributes else ""
            if subtype != "road":
                continue
            if (
                "participant:vehicle" in lanelet.attributes
                and lanelet.attributes["participant:vehicle"] != "yes"
            ):
                continue
            points = tuple(
                (float(point.x), float(point.y), float(point.z))
                for point in lanelet.centerline
            )
            if len(points) >= 2:
                vehicle_lanelets[int(lanelet.id)] = (lanelet, points)

        records = []
        for lanelet_id, (lanelet, points) in vehicle_lanelets.items():
            following = tuple(
                int(candidate.id)
                for candidate in routing_graph.following(lanelet)
                if int(candidate.id) in vehicle_lanelets
            )
            attributes = tuple(
                (key, str(lanelet.attributes[key]))
                for key in ("subtype", "speed_limit")
                if key in lanelet.attributes
            )
            left_boundary = tuple(
                (float(point.x), float(point.y), float(point.z))
                for point in lanelet.leftBound
            )
            right_boundary = tuple(
                (float(point.x), float(point.y), float(point.z))
                for point in lanelet.rightBound
            )
            records.append(LaneletRecord(
                lanelet_id, points, following, attributes,
                left_boundary, right_boundary,
            ))

        return cls(
            records,
            config=config,
            map_sha256=hashlib.sha256(map_path.read_bytes()).hexdigest(),
            lanelet_layer=lanelet_map.laneletLayer,
        )

    def _nearby_record_ids(self, x: float, y: float) -> Iterable[int]:
        indexed_ids = self._spatial_index.get(self._grid_cell(x, y), ())
        if indexed_ids:
            return indexed_ids
        if self._lanelet_layer is None:
            return self.records.keys()
        import lanelet2

        point = lanelet2.core.BasicPoint2d(float(x), float(y))
        nearest = self._lanelet_layer.nearest(
            point, max(1, int(self.config.maximum_nearest_lanelets))
        )
        return [int(lanelet.id) for lanelet in nearest if int(lanelet.id) in self.records]

    def lane_candidates(self, x: float, y: float, yaw: float) -> Tuple[_LaneCandidate, ...]:
        candidates = []
        for lanelet_id in self._nearby_record_ids(x, y):
            self._refresh_lane_geometry(lanelet_id)
            projection = self._projection_geometry[lanelet_id].project(x, y)
            if projection is None or projection.distance > self.config.maximum_lane_distance_m:
                continue
            yaw_error = abs(normalize_angle(yaw - projection.yaw))
            if yaw_error > self.config.maximum_lane_yaw_error_rad:
                continue
            distance_sigma = max(self.config.lane_distance_sigma_m, 1.0e-3)
            yaw_sigma = max(self.config.lane_yaw_sigma_rad, 1.0e-3)
            score = math.exp(
                -0.5 * (projection.distance / distance_sigma) ** 2
                -0.5 * (yaw_error / yaw_sigma) ** 2
            )
            candidates.append(_LaneCandidate(lanelet_id, projection, score))
        candidates.sort(key=lambda candidate: candidate.score, reverse=True)
        return tuple(candidates[: max(1, int(self.config.maximum_start_lanelets))])

    def project_lanelet(self, lanelet_id: int, x: float, y: float):
        """Project onto one named road using the immutable geometry cache."""
        if lanelet_id not in self.records:
            return None
        self._refresh_lane_geometry(lanelet_id)
        return self._projection_geometry[lanelet_id].project(x, y)

    def _explicit_lane_candidate(self, lanelet_id, x, y, yaw):
        """Validate an already selected current lane without re-ranking lanes."""
        projection = self.project_lanelet(lanelet_id, x, y)
        if projection is None or projection.distance > self.config.maximum_lane_distance_m:
            return None
        yaw_error = abs(normalize_angle(yaw - projection.yaw))
        if yaw_error > self.config.maximum_lane_yaw_error_rad:
            return None
        distance_sigma = max(self.config.lane_distance_sigma_m, 1.0e-3)
        yaw_sigma = max(self.config.lane_yaw_sigma_rad, 1.0e-3)
        score = math.exp(
            -0.5 * (projection.distance / distance_sigma) ** 2
            -0.5 * (yaw_error / yaw_sigma) ** 2
        )
        return _LaneCandidate(lanelet_id, projection, score)

    def recover_lane_direction(self, x, y, allowed_lanelet_ids):
        """Geometry-only direction for an explicitly allowed one-way region.

        Keep the existing distance gate. Require the closest road to be allowed
        and equally plausible nearby roads to agree on direction. This is not
        a global nearest-road snap and does not alter a tracked state.
        """
        nearby = []
        for lanelet_id in self._nearby_record_ids(x, y):
            self._refresh_lane_geometry(lanelet_id)
            p = self._projection_geometry[lanelet_id].project(x, y)
            if p is not None and p.distance <= self.config.maximum_lane_distance_m:
                nearby.append((p.distance, lanelet_id, p.yaw))
        nearby.sort()
        if not nearby or nearby[0][1] not in allowed_lanelet_ids:
            return None
        yaw = nearby[0][2]
        for distance, lanelet_id, heading in nearby:
            if distance > nearby[0][0] + self.config.lane_distance_sigma_m:
                continue
            if (lanelet_id not in allowed_lanelet_ids or
                    abs(normalize_angle(heading-yaw)) > self.config.maximum_lane_yaw_error_rad):
                return None
        return yaw

    def _transition_score(self, first_id: int, second_id: int) -> float:
        key = (first_id, second_id)
        first_record = self.records[first_id]
        second_record = self.records[second_id]
        with self._geometry_cache_lock:
            cached = self._transition_scores.get(key)
            if cached is not None and cached[0] is first_record and cached[1] is second_record:
                self._transition_scores.move_to_end(key)
                return cached[2]
        first = first_record.centerline
        second = second_record.centerline
        first_yaw = tangent_yaw(first, polyline_length(first))
        second_yaw = tangent_yaw(second, 0.0)
        delta = abs(normalize_angle(second_yaw - first_yaw))
        score = math.exp(-0.5 * (delta / math.radians(30.0)) ** 2)
        with self._geometry_cache_lock:
            self._transition_scores[key] = (first_record, second_record, score)
            if len(self._transition_scores) > self._transition_score_cache_limit:
                self._transition_scores.popitem(last=False)
        return score

    def _route_sequences(
        self, start_id: int, start_arc_length: float, required_length: float
    ) -> Tuple[Tuple[Tuple[int, ...], float], ...]:
        start_remaining = max(0.0, self._lane_length(start_id) - start_arc_length)
        completed = []
        # A nested recursive function captures its own closure, retaining one
        # reference cycle per start-lanelet call until cyclic GC. An explicit
        # DFS stack preserves visitation/tie order without cyclic allocations.
        pending = [([start_id], start_remaining, 1.0, {start_id: 1})]
        while pending and len(completed) < self.config.maximum_paths * 3:
            ids, available_length, score, occurrences = pending.pop()
            last_id = ids[-1]
            if available_length >= required_length or len(ids) >= self.config.maximum_route_depth:
                completed.append((tuple(ids), score))
                continue
            followers = [
                lanelet_id
                for lanelet_id in self.records[last_id].following_ids
                if lanelet_id in self.records and occurrences.get(lanelet_id, 0) < 2
            ]
            if not followers:
                completed.append((tuple(ids), score))
                continue
            branch_prior = 1.0 / len(followers)
            followers.sort(
                key=lambda lanelet_id: self._transition_score(last_id, lanelet_id),
                reverse=True,
            )
            # Reversed push preserves recursive depth-first follower order.
            for lanelet_id in reversed(followers):
                next_occurrences = dict(occurrences)
                next_occurrences[lanelet_id] = next_occurrences.get(lanelet_id, 0) + 1
                pending.append((ids + [lanelet_id],
                                available_length + self._lane_length(lanelet_id),
                                score * branch_prior * self._transition_score(last_id, lanelet_id),
                                next_occurrences))
        completed.sort(key=lambda item: item[1], reverse=True)
        return tuple(completed[: max(1, int(self.config.maximum_paths))])

    def _single_route_sequence(
        self, start_id: int, start_arc_length: float, required_length: float
    ):
        """Continue the current lane geometrically, without future Top-K scores.

        This is an explicitly requested highway policy. At a forward fork the
        continuation with least lateral departure from the incoming direction
        is retained. No lateral routing edge, NPC goal, ego route, confidence,
        or future observation is consulted. Ordinary route enumeration is not
        changed. A later observation may select a different current lane.
        """
        ids = [start_id]
        available = max(0.0, self._lane_length(start_id) - start_arc_length)
        occurrences = {start_id: 1}
        while available < required_length and len(ids) < self.config.maximum_route_depth:
            last_id = ids[-1]
            followers = tuple(dict.fromkeys(
                lanelet_id for lanelet_id in self.records[last_id].following_ids
                if lanelet_id in self.records and occurrences.get(lanelet_id, 0) < 2
            ))
            if not followers:
                break
            successor = self.continuation_lanelet_id(last_id, followers)
            ids.append(successor)
            occurrences[successor] = occurrences.get(successor, 0) + 1
            available += self._lane_length(successor)
        return ((tuple(ids), 1.0),)

    def continuation_lanelet_id(self, lanelet_id: int, following_ids=None):
        """Return the geometric same-direction forward continuation, if any."""
        if lanelet_id not in self.records:
            return None
        followers = tuple(
            value for value in (
                self.records[lanelet_id].following_ids
                if following_ids is None else following_ids
            ) if value in self.records
        )
        if not followers:
            return None
        first = self.records[lanelet_id].centerline
        incoming_yaw = tangent_yaw(first, self._lane_length(lanelet_id))
        origin = first[-1]

        def continuity_key(value):
            second = self.records[value].centerline
            lookahead = min(20.0, self._lane_length(value))
            point = sample_polyline(second, lookahead)
            dx, dy = point[0] - origin[0], point[1] - origin[1]
            lateral = abs(-math.sin(incoming_yaw) * dx + math.cos(incoming_yaw) * dy)
            heading = abs(normalize_angle(tangent_yaw(second, lookahead) - incoming_yaw))
            join_gap = _distance(origin, second[0])
            # Routing graph order breaks an exact geometric tie, not score.
            return (lateral, heading, join_gap)

        return min(followers, key=continuity_key)

    def _stitch_route(self, lanelet_ids: Sequence[int]) -> Tuple[Point3, ...]:
        points = []
        for lanelet_id in lanelet_ids:
            centerline = self.records[lanelet_id].centerline
            if points and _distance(points[-1], centerline[0]) < 1.0e-3:
                points.extend(centerline[1:])
            else:
                points.extend(centerline)
        return tuple(points)

    def _route_geometry(self, lanelet_ids: Sequence[int]):
        # Include the geometry class in the key so scalar reference patches in
        # offline regression tools cannot accidentally reuse an indexed route.
        key = (_RoutePolyline, tuple(lanelet_ids))
        records = tuple(self.records[lanelet_id] for lanelet_id in lanelet_ids)
        with self._geometry_cache_lock:
            entry = self._route_geometry_cache.get(key)
            if entry is not None and all(old is current for old, current in zip(entry[0], records)):
                self._route_geometry_cache.move_to_end(key)
                return entry[1]
        points = self._stitch_route(lanelet_ids)
        geometry = (_RoutePolyline(points), _ProjectionPolyline(points))
        with self._geometry_cache_lock:
            self._route_geometry_cache[key] = (records, geometry)
            if len(self._route_geometry_cache) > self._route_geometry_cache_limit:
                self._route_geometry_cache.popitem(last=False)
        return geometry

    def _same_trajectory_hypothesis(
        self,
        first: Sequence[Tuple[float, float, float, float]],
        second: Sequence[Tuple[float, float, float, float]],
    ) -> bool:
        """Return whether two routes express the same observable motion.

        Route enumeration can reach the same physical future through several
        nearby start-lanelet matches.  Exact lanelet-ID equality would retain
        those map-matching duplicates, so clustering is performed on the
        predicted XY samples within the actual output horizon instead.
        """
        if not first or len(first) != len(second):
            return False
        threshold = max(0.0, self.config.trajectory_cluster_distance_m)
        return max(
            math.hypot(a[0] - b[0], a[1] - b[1])
            for a, b in zip(first, second)
        ) <= threshold

    def _distinct_trajectory_hypotheses(self, raw_paths):
        """Keep the highest-scoring representative of each geometric mode.

        Using the maximum raw score is intentionally conservative.  The
        current lane and route scores are heuristic rather than calibrated
        probabilities, so summing duplicate scores would reward hypotheses
        merely because the map enumerated them more often.  A later revision
        should normalize start-lane probabilities and conditional branch
        probabilities, then marginalize probability mass per cluster.
        """
        representatives = []
        for candidate in sorted(raw_paths, key=lambda item: item[0], reverse=True):
            if any(
                # Geometry alone is insufficient near the beginning of a
                # fork: two legitimate sibling branches can remain closer
                # than the clustering threshold for most of a short/slow
                # horizon.  Only routes reaching the same downstream
                # Lanelet are eligible to collapse.
                candidate[1][-1] == existing[1][-1]
                and self._same_trajectory_hypothesis(candidate[2], existing[2])
                for existing in representatives
            ):
                continue
            representatives.append(candidate)
            if len(representatives) >= max(1, int(self.config.maximum_paths)):
                break
        return representatives

    def predict(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        speed: float,
        horizons: Sequence[float],
        truncate_at_route_end: bool = False,
        allow_low_speed: bool = False,
        *,
        start_lanelet_ids: Optional[Iterable[int]] = None,
        single_route: bool = False,
    ) -> Tuple[PredictedTrajectory, ...]:
        if not horizons or speed < 0. or (speed < self.config.minimum_speed_mps and not allow_low_speed):
            return ()
        required_length = speed * max(horizons) + self.config.route_length_padding_m
        raw_paths = []
        if start_lanelet_ids is None:
            candidates = self.lane_candidates(x, y, yaw)
        else:
            candidates = tuple(
                candidate for lanelet_id in dict.fromkeys(start_lanelet_ids)
                for candidate in (self._explicit_lane_candidate(lanelet_id, x, y, yaw),)
                if candidate is not None
            )
        if single_route:
            candidates = candidates[:1]
        route_sequences = self._single_route_sequence if single_route else self._route_sequences
        for candidate in candidates:
            for lanelet_ids, route_score in route_sequences(
                candidate.lanelet_id,
                candidate.projection.arc_length,
                required_length,
            ):
                geometry, projection_geometry = self._route_geometry(lanelet_ids)
                start_projection = projection_geometry.project(x, y)
                if start_projection is None:
                    continue
                available = geometry.length - start_projection.arc_length
                route_horizons = horizons
                if truncate_at_route_end and available < speed * max(horizons) - 1.e-9:
                    # A genuine graph terminal represents the end of this NPC
                    # route. Do not invent movement past it or discard the
                    # valid prefix. A search-depth/cycle limit is NOT a terminal.
                    if self.records[lanelet_ids[-1]].following_ids:
                        continue
                    route_horizons = tuple(t for t in horizons if speed * t <= available + 1.e-9)
                elif available + 0.5 < speed * max(horizons):
                    continue
                offset_x = x - start_projection.x
                offset_y = y - start_projection.y
                offset_z = z - start_projection.z
                poses = []
                for horizon in route_horizons:
                    if speed <= 1.e-9:
                        # A genuinely stationary, map-matched NPC remains at
                        # its observed location. Do not invent lateral motion.
                        poses.append((x, y, z, geometry.tangent(start_projection.arc_length)))
                        continue
                    route_distance = start_projection.arc_length + speed * horizon
                    center = geometry.sample(route_distance)
                    if self.config.lateral_convergence_time_sec <= 1.0e-6:
                        residual = 0.0
                    else:
                        residual = max(
                            0.0,
                            1.0 - horizon / self.config.lateral_convergence_time_sec,
                        )
                    poses.append(
                        (
                            center[0] + residual * offset_x,
                            center[1] + residual * offset_y,
                            center[2] + residual * offset_z,
                            geometry.tangent(route_distance),
                        )
                    )
                raw_paths.append(
                    (candidate.score * route_score, tuple(lanelet_ids), tuple(poses))
                )

        # Enumerate generously first, then cap the number of *distinct*
        # geometric futures.  Capping raw routes before clustering can let
        # duplicate start-lane matches crowd a real sibling branch out of the
        # published Top-K set.
        raw_paths = self._distinct_trajectory_hypotheses(raw_paths)
        total_score = sum(max(score, 0.0) for score, _, _ in raw_paths)
        if total_score <= 1.0e-12:
            return ()
        return tuple(
            PredictedTrajectory(
                poses=poses,
                confidence=float(score / total_score),
                lanelet_ids=lanelet_ids,
            )
            for score, lanelet_ids, poses in raw_paths
        )

    def extend_trajectories(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        speed: float,
        base_horizons: Sequence[float],
        extended_horizons: Sequence[float],
        trajectories: Tuple[PredictedTrajectory, ...],
        allowed_lanelet_ids: Optional[Iterable[int]] = None,
    ) -> Tuple[PredictedTrajectory, ...]:
        """Append samples to already selected routes without changing their prefix.

        ``extended_horizons`` is the *full* horizon sequence and must start
        with ``base_horizons`` exactly. Route selection, clustering and score
        normalization have already happened in :meth:`predict`; none is
        repeated here. A branch not selected by that original prediction is
        deliberately not guessed. If any retained path cannot be extended,
        the original tuple for the whole object is returned unchanged.

        This optional operation never pads/clamps a new sample at a route
        end. Map construction/loading errors retain their normal exceptions;
        there is intentionally no blanket exception handler here.
        """
        base = tuple(base_horizons)
        extended = tuple(extended_horizons)
        if (
            not trajectories
            or not base
            or len(extended) <= len(base)
            or extended[: len(base)] != base
            or not all(math.isfinite(value) and value >= 0.0 for value in extended)
            or any(first >= second for first, second in zip(extended, extended[1:]))
            or not all(math.isfinite(value) for value in (x, y, z, yaw, speed))
            or speed < self.config.minimum_speed_mps
            or speed <= 0.0
        ):
            return trajectories
        allowed = None if allowed_lanelet_ids is None else frozenset(allowed_lanelet_ids)
        suffix_horizons = extended[len(base):]
        maximum_depth = max(1, int(self.config.maximum_route_depth))
        completed = []
        for trajectory in trajectories:
            route_ids = tuple(trajectory.lanelet_ids)
            if (
                not route_ids
                or len(route_ids) > maximum_depth
                or len(trajectory.poses) != len(base)
                or any(lanelet_id not in self.records for lanelet_id in route_ids)
                or (allowed is not None and any(i not in allowed for i in route_ids))
                or any(
                    second not in self.records[first].following_ids
                    for first, second in zip(route_ids, route_ids[1:])
                )
            ):
                return trajectories
            base_geometry, projection_geometry = self._route_geometry(route_ids)
            start = projection_geometry.project(x, y)
            if start is None:
                return trajectories
            # Production predict() tolerates up to 0.5 m at a route end. That
            # legacy tolerance may already clamp its final baseline sample;
            # retain it as-is rather than extending a temporarily stopped
            # prefix into a newly moving suffix.
            if base_geometry.length < start.arc_length + speed * base[-1]:
                return trajectories
            # Keep the original projection even if a later looping segment
            # happens to pass closer to the object. Extending must not move
            # the route's arc origin or change the retained 3-second path.
            required_distance = start.arc_length + speed * extended[-1]
            geometry = base_geometry
            occurrences = {i: route_ids.count(i) for i in set(route_ids)}
            while geometry.length < required_distance:
                if len(route_ids) >= maximum_depth:
                    return trajectories
                followers = tuple(dict.fromkeys(self.records[route_ids[-1]].following_ids))
                # Inspect all graph successors before the region filter: a
                # branch outside the allowed region must not make its sibling
                # look like an unambiguous route continuation.
                if len(followers) != 1:
                    return trajectories
                successor = followers[0]
                if (
                    successor not in self.records
                    or occurrences.get(successor, 0) >= 2
                    or (allowed is not None and successor not in allowed)
                ):
                    return trajectories
                route_ids += (successor,)
                occurrences[successor] = occurrences.get(successor, 0) + 1
                geometry, _ = self._route_geometry(route_ids)
            offset_x, offset_y, offset_z = x - start.x, y - start.y, z - start.z
            suffix = []
            for horizon in suffix_horizons:
                route_distance = start.arc_length + speed * horizon
                center = geometry.sample(route_distance)
                residual = (
                    0.0
                    if self.config.lateral_convergence_time_sec <= 1.0e-6
                    else max(0.0, 1.0 - horizon / self.config.lateral_convergence_time_sec)
                )
                suffix.append((
                    center[0] + residual * offset_x,
                    center[1] + residual * offset_y,
                    center[2] + residual * offset_z,
                    geometry.tangent(route_distance),
                ))
            completed.append(PredictedTrajectory(
                poses=trajectory.poses + tuple(suffix),
                confidence=trajectory.confidence,
                lanelet_ids=route_ids,
            ))
        return tuple(completed)
