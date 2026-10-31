"""Opt-in, causal stationary-UNKNOWN identity continuity at the output boundary.

This is not detector NMS or a track merger. A *new* confirmed tracker UUID may
inherit one uniquely absent, recently retired stationary UNKNOWN UUID. Every
current object, pose, shape, covariance, confidence and stamp is retained.
Support is from raw tracker output samples before the UNKNOWN zero-motion
policy; distinct output timestamps are not claimed to be fresh detections.
"""
from __future__ import annotations

from collections import deque
import copy
from dataclasses import dataclass, field
import io
import math

from .kinematic_prediction import dominant_label


@dataclass(frozen=True)
class ContinuityConfig:
    minimum_support_sec: float = .6
    minimum_samples: int = 5
    support_window_sec: float = 1.
    maximum_raw_speed_mps: float = .5
    maximum_position_spread_m: float = .4
    maximum_center_distance_m: float = .35
    maximum_vertical_distance_m: float = .6
    maximum_dimension_ratio: float = 1.6
    minimum_footprint_iou: float = .35
    maximum_gap_sec: float = 2.
    ambiguity_distance_m: float = 1.

    def __post_init__(self):
        values = [value for key, value in vars(self).items() if key != 'minimum_samples']
        if (not all(math.isfinite(value) and value > 0. for value in values) or
                type(self.minimum_samples) is not int or self.minimum_samples < 2 or
                self.support_window_sec < self.minimum_support_sec or
                self.maximum_dimension_ratio < 1. or self.minimum_footprint_iou > 1. or
                self.ambiguity_distance_m < self.maximum_center_distance_m):
            raise ValueError('Invalid stationary UUID continuity configuration')


@dataclass(frozen=True)
class _Observation:
    raw_uuid: bytes
    label: int
    center: tuple
    dimensions: tuple
    yaw: float
    raw_speed: float


@dataclass
class _Record:
    logical_uuid: bytes
    last_seen: float
    observation: _Observation
    known_class_seen: bool = False
    # A close, distinct current raw UUID makes later re-identification
    # ambiguous even after that neighbour leaves the short retired pool.
    # This latch follows only this raw track's lifetime; output is unchanged.
    nearby_other_uuid_seen: bool = False
    samples: deque = field(default_factory=lambda: deque(maxlen=128))


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _footprint(observation):
    x, y = observation.center[:2]
    length, width = observation.dimensions[:2]
    c, s = math.cos(observation.yaw), math.sin(observation.yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((-length / 2., -width / 2.),
                           (length / 2., -width / 2.),
                           (length / 2., width / 2.),
                           (-length / 2., width / 2.))]


def _area(polygon):
    if len(polygon) < 3:
        return 0.
    # Relative coordinates avoid cancellation for small footprints located
    # at large map/UTM coordinates; polygon area is translation invariant.
    origin = polygon[0]
    return abs(sum((a[0] - origin[0]) * (b[1] - origin[1]) -
                   (a[1] - origin[1]) * (b[0] - origin[0])
                   for a, b in zip(polygon, polygon[1:] + polygon[:1]))) / 2.


def _footprint_iou(first, second):
    polygon = _footprint(first)
    clip = _footprint(second)
    for edge_start, edge_end in zip(clip, clip[1:] + clip[:1]):
        source, polygon = polygon, []
        if not source:
            return 0.
        previous = source[-1]
        before = _cross(edge_start, edge_end, previous)
        for point in source:
            after = _cross(edge_start, edge_end, point)
            if (after >= -1.e-9) != (before >= -1.e-9):
                fraction = before / (before - after)
                polygon.append((previous[0] + fraction * (point[0] - previous[0]),
                                previous[1] + fraction * (point[1] - previous[1])))
            if after >= -1.e-9:
                polygon.append(point)
            previous, before = point, after
    intersection = _area(polygon)
    union = (_area(_footprint(first)) + _area(_footprint(second)) - intersection)
    return intersection / union if union > 1.e-12 else 0.


def _observe(obj):
    """Return validated global raw-output evidence, never zeroed policy speed."""
    try:
        raw_uuid = bytes(obj.object_id.uuid)
        pose = obj.kinematics.pose_with_covariance.pose
        p, q = pose.position, pose.orientation
        dimensions = obj.shape.dimensions
        velocity = obj.kinematics.twist_with_covariance.twist.linear
        center = (float(p.x), float(p.y), float(p.z))
        size = (float(dimensions.x), float(dimensions.y), float(dimensions.z))
        speed = math.hypot(velocity.x, velocity.y)
        quaternion = (q.x, q.y, q.z, q.w)
        norm = sum(value * value for value in quaternion)
        if (len(raw_uuid) != 16 or not obj.classification or obj.shape.type != 0 or
                not all(math.isfinite(value) for value in (*center, *size, speed, *quaternion)) or
                min(size) <= 0. or norm <= 1.e-12):
            return None
        # Normalise the quaternion rather than relying on rounded debug data.
        x, y, z, w = (value / math.sqrt(norm) for value in quaternion)
        yaw = math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))
        return _Observation(raw_uuid, dominant_label(obj), center, size, yaw, speed)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


class StationaryUuidContinuity:
    """Only rewrite outgoing IDs, with default OFF and bounded retired memory."""

    def __init__(self, enabled=False, config=None):
        self.enabled = bool(enabled)
        self.config = config or ContinuityConfig()
        self.active = {}
        self.retired = {}
        self.last_stamp = None
        self.last_payload = None
        self.last_sequence = None
        self.last_events = []
        self.last_rejections = []

    def reset(self):
        self.active.clear()
        self.retired.clear()
        self.last_stamp = None
        self.last_payload = None
        self.last_sequence = None

    def _stable(self, record):
        samples = list(record.samples)
        config = self.config
        if (record.known_class_seen or record.nearby_other_uuid_seen or
                record.observation.label != 0 or
                len(samples) < config.minimum_samples or
                samples[-1][0] - samples[0][0] + 1.e-6 < config.minimum_support_sec):
            return False
        if max(sample[2] for sample in samples) > config.maximum_raw_speed_mps:
            return False
        spread = max(math.dist(a[1][:2], b[1][:2]) for a in samples for b in samples)
        return spread <= config.maximum_position_spread_m

    def _compatible(self, current, previous):
        config = self.config
        if (current.label != 0 or previous.label != 0 or
                math.dist(current.center[:2], previous.center[:2]) > config.maximum_center_distance_m or
                abs(current.center[2] - previous.center[2]) > config.maximum_vertical_distance_m):
            return False
        first = (*sorted(current.dimensions[:2]), current.dimensions[2])
        second = (*sorted(previous.dimensions[:2]), previous.dimensions[2])
        if any(max(a, b) / min(a, b) > config.maximum_dimension_ratio for a, b in zip(first, second)):
            return False
        return _footprint_iou(current, previous) >= config.minimum_footprint_iou

    def _output(self, message):
        replacements = {raw: record.logical_uuid for raw, record in self.active.items()
                        if raw != record.logical_uuid}
        if not replacements:
            return message
        targets = [replacements.get(bytes(obj.object_id.uuid), bytes(obj.object_id.uuid))
                   for obj in message.objects]
        if len(set(targets)) != len(targets):
            # A tracker restart/raw UUID reuse must never create two equal
            # outgoing IDs. Fail open to unchanged raw UUIDs and fresh history.
            self.reset()
            self.last_events.append(dict(kind='reset', reason='outgoing_uuid_collision'))
            return message
        output = copy.copy(message)
        output.objects = list(message.objects)
        for index, (obj, target) in enumerate(zip(message.objects, targets)):
            if target != bytes(obj.object_id.uuid):
                replacement = copy.copy(obj)
                replacement.object_id = copy.copy(obj.object_id)
                replacement.object_id.uuid = list(target)
                output.objects[index] = replacement
        return output

    def apply(self, message):
        self.last_events = []
        self.last_rejections = []
        if not self.enabled:
            return message
        try:
            stamp = float(message.header.stamp.to_sec())
            if not math.isfinite(stamp) or message.header.frame_id.lstrip('/') != 'map':
                raise ValueError('Global finite timestamp required')
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.reset()
            self.last_events.append(dict(kind='reset', reason='invalid_frame_or_stamp'))
            return message
        try:
            raw_keys = [bytes(obj.object_id.uuid) for obj in message.objects]
            sequence = int(message.header.seq)
            if not 0 <= sequence <= 0xffffffff:
                raise ValueError('Invalid uint32 publisher sequence')
            normalised = copy.copy(message)
            normalised.header = copy.copy(message.header)
            normalised.header.seq = 0
            stream = io.BytesIO()
            normalised.serialize(stream)
            payload = stream.getvalue()
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.reset()
            self.last_events.append(dict(kind='reset', reason='invalid_input_payload'))
            return message
        if len(raw_keys) != len(set(raw_keys)):
            self.reset()
            self.last_events.append(dict(kind='reset', reason='duplicate_input_uuid'))
            return message
        if (self.last_sequence is not None and sequence < self.last_sequence and
                not (self.last_sequence >= 0xfffffff0 and sequence <= 0xf)):
            # Native ROS publisher restart normally restarts header.seq even
            # when ROS time continues forward and raw numeric IDs are reused.
            # A producer that always sends seq=0 cannot expose such an epoch.
            self.reset()
            self.last_events.append(dict(kind='reset', reason='publisher_sequence_regression'))
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.reset()
            self.last_events.append(dict(kind='reset', reason='backward_time'))
        elif self.last_stamp is not None and stamp - self.last_stamp > self.config.maximum_gap_sec:
            self.reset()
            self.last_events.append(dict(kind='reset', reason='forward_callback_gap'))
        if self.last_stamp is not None and stamp == self.last_stamp:
            # A repeated scheduler packet is not new stability support.
            if payload == self.last_payload:
                self.last_sequence = sequence
                return self._output(message)
            # Same time but new class/geometry must not bypass known-class or
            # slot-reuse protection. Discard old support, then seed only this
            # changed packet as a fresh record, with no inherited mapping.
            self.reset()
            self.last_events.append(dict(kind='reset', reason='changed_payload_same_stamp'))
        self.last_stamp = stamp
        self.last_payload = payload
        self.last_sequence = sequence
        observations = {observation.raw_uuid: observation for obj in message.objects
                        for observation in [_observe(obj)] if observation is not None}
        # Crowding is about another *present raw ID*, not whether that
        # neighbour is itself valid for identity reuse. A malformed shape,
        # class or velocity must not erase evidence of a nearby second actor.
        positions, unlocated = {}, set()
        for obj, raw in zip(message.objects, raw_keys):
            try:
                point = obj.kinematics.pose_with_covariance.pose.position
                xy = (float(point.x), float(point.y))
                if not all(math.isfinite(value) for value in xy):
                    raise ValueError('Unlocated current raw object')
                positions[raw] = xy
            except (AttributeError, TypeError, ValueError, OverflowError):
                unlocated.add(raw)
        crowded = {raw for raw, observation in observations.items()
                   if any(other != raw for other in unlocated) or
                   any(other != raw and math.dist(observation.center[:2], xy) <=
                          self.config.ambiguity_distance_m
                          for other, xy in positions.items())}
        for raw in crowded.intersection(self.active):
            self.active[raw].nearby_other_uuid_seen = True
        for raw in list(self.active):
            if raw not in raw_keys:
                record = self.active.pop(raw)
                if self._stable(record):
                    self.retired[raw] = record
            elif raw not in observations:
                # Invalid geometry/class evidence cannot retain eligibility.
                self.active.pop(raw)
        self.retired = {raw: record for raw, record in self.retired.items()
                        if 0. < stamp - record.last_seen <= self.config.maximum_gap_sec}
        births = {raw: observation for raw, observation in observations.items() if raw not in self.active}
        # A retired source UUID returning as an actual raw object takes its
        # own identity, not another outgoing alias of the same logical ID.
        for raw in observations:
            self.retired.pop(raw, None)
        reserved = {record.logical_uuid for raw, record in self.active.items() if raw in raw_keys}
        assignments = {}
        for raw, observation in births.items():
            if observation.label != 0:
                continue
            if len(observations) != len(raw_keys):
                self.last_rejections.append(dict(raw_uuid=raw.hex(), reason='invalid_current_observation'))
                continue
            near_current = [other for other, value in observations.items() if other != raw and
                            math.dist(observation.center[:2], value.center[:2]) <= self.config.ambiguity_distance_m]
            near_retired = [old for old, record in self.retired.items() if
                            math.dist(observation.center[:2], record.observation.center[:2]) <= self.config.ambiguity_distance_m]
            candidates = [old for old in near_retired
                          if self.retired[old].logical_uuid not in reserved and
                          self.retired[old].logical_uuid not in raw_keys and
                          self._compatible(observation, self.retired[old].observation)]
            if near_current:
                reason = 'nearby_current_object'
            elif len(near_retired) > 1 or len(candidates) > 1:
                reason = 'ambiguous_retired_objects'
            elif len(candidates) == 1:
                assignments[raw] = candidates[0]
                continue
            else:
                reason = 'no_unique_supported_geometry_match'
            self.last_rejections.append(dict(raw_uuid=raw.hex(), reason=reason))
        uses = {old: sum(value == old for value in assignments.values()) for old in assignments.values()}
        for raw, observation in observations.items():
            if raw in self.active:
                record = self.active[raw]
            else:
                old = assignments.get(raw)
                if old is not None and uses[old] == 1:
                    previous = self.retired.pop(old)
                    logical = previous.logical_uuid
                    self.last_events.append(dict(kind='reuse', raw_uuid=raw.hex(),
                        previous_raw_uuid=old.hex(), logical_uuid=logical.hex(),
                        gap_sec=stamp - previous.last_seen,
                        center_distance_m=math.dist(observation.center[:2], previous.observation.center[:2]),
                        footprint_iou=_footprint_iou(observation, previous.observation)))
                else:
                    logical = raw
                    if old is not None:
                        self.last_rejections.append(dict(raw_uuid=raw.hex(), reason='competing_new_objects'))
                record = _Record(logical, stamp, observation)
                self.active[raw] = record
            record.last_seen, record.observation = stamp, observation
            record.known_class_seen |= observation.label != 0
            record.nearby_other_uuid_seen |= raw in crowded
            record.samples.append((stamp, observation.center, observation.raw_speed))
            while len(record.samples) > 1 and record.samples[0][0] < stamp - self.config.support_window_sec - 1.e-6:
                record.samples.popleft()
        return self._output(message)
