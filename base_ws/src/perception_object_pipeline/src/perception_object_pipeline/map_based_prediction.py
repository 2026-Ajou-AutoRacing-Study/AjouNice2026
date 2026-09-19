"""Lanelet2-backed vehicle prediction core independent from ROS callbacks.

The implementation intentionally keeps the ROS2 Autoware algorithm boundary:
lane matching, routing-graph path enumeration and path confidence live here,
while message conversion stays in the ROS1 node.
"""

from __future__ import annotations

import hashlib
import math
import ctypes
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


@dataclass(frozen=True)
class LaneletRecord:
    lanelet_id: int
    centerline: Tuple[Point3, ...]
    following_ids: Tuple[int, ...] = ()

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
            records.append(LaneletRecord(lanelet_id, points, following))

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
            record = self.records[lanelet_id]
            projection = project_point_to_polyline(record.centerline, x, y)
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

    def _transition_score(self, first_id: int, second_id: int) -> float:
        first = self.records[first_id].centerline
        second = self.records[second_id].centerline
        first_yaw = tangent_yaw(first, polyline_length(first))
        second_yaw = tangent_yaw(second, 0.0)
        delta = abs(normalize_angle(second_yaw - first_yaw))
        return math.exp(-0.5 * (delta / math.radians(30.0)) ** 2)

    def _route_sequences(
        self, start_id: int, start_arc_length: float, required_length: float
    ) -> Tuple[Tuple[Tuple[int, ...], float], ...]:
        start_remaining = max(0.0, self.records[start_id].length - start_arc_length)
        completed = []

        def visit(ids, available_length, score, occurrences):
            if len(completed) >= self.config.maximum_paths * 3:
                return
            last_id = ids[-1]
            if available_length >= required_length or len(ids) >= self.config.maximum_route_depth:
                completed.append((tuple(ids), score))
                return
            followers = [
                lanelet_id
                for lanelet_id in self.records[last_id].following_ids
                if lanelet_id in self.records and occurrences.get(lanelet_id, 0) < 2
            ]
            if not followers:
                completed.append((tuple(ids), score))
                return
            branch_prior = 1.0 / len(followers)
            followers.sort(
                key=lambda lanelet_id: self._transition_score(last_id, lanelet_id),
                reverse=True,
            )
            for lanelet_id in followers:
                next_occurrences = dict(occurrences)
                next_occurrences[lanelet_id] = next_occurrences.get(lanelet_id, 0) + 1
                visit(
                    ids + [lanelet_id],
                    available_length + self.records[lanelet_id].length,
                    score * branch_prior * self._transition_score(last_id, lanelet_id),
                    next_occurrences,
                )

        visit([start_id], start_remaining, 1.0, {start_id: 1})
        completed.sort(key=lambda item: item[1], reverse=True)
        return tuple(completed[: max(1, int(self.config.maximum_paths))])

    def _stitch_route(self, lanelet_ids: Sequence[int]) -> Tuple[Point3, ...]:
        points = []
        for lanelet_id in lanelet_ids:
            centerline = self.records[lanelet_id].centerline
            if points and _distance(points[-1], centerline[0]) < 1.0e-3:
                points.extend(centerline[1:])
            else:
                points.extend(centerline)
        return tuple(points)

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
    ) -> Tuple[PredictedTrajectory, ...]:
        if not horizons or speed < self.config.minimum_speed_mps:
            return ()
        required_length = speed * max(horizons) + self.config.route_length_padding_m
        raw_paths = []
        for candidate in self.lane_candidates(x, y, yaw):
            for lanelet_ids, route_score in self._route_sequences(
                candidate.lanelet_id,
                candidate.projection.arc_length,
                required_length,
            ):
                route = self._stitch_route(lanelet_ids)
                start_projection = project_point_to_polyline(route, x, y)
                if start_projection is None:
                    continue
                available = polyline_length(route) - start_projection.arc_length
                if available + 0.5 < speed * max(horizons):
                    continue
                offset_x = x - start_projection.x
                offset_y = y - start_projection.y
                offset_z = z - start_projection.z
                poses = []
                for horizon in horizons:
                    route_distance = start_projection.arc_length + speed * horizon
                    center = sample_polyline(route, route_distance)
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
                            tangent_yaw(route, route_distance),
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
