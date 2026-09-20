"""Timing and ego-motion helpers for a rolling LiDAR scan."""

import math
import threading
from collections import deque

import numpy as np


def scan_time_contract(point_time_offsets, max_span_sec=0.15, reference="end"):
    """Validate one rolling-scan timing vector.

    Returns a small dictionary that can be serialized into dataset/runtime
    audits, or ``None`` when deskew must not be applied.  Keeping this gate in
    the shared math module prevents offline conversion and online inference
    from silently accepting different scans.
    """
    offsets = np.asarray(point_time_offsets, dtype=np.float64).reshape(-1)
    if offsets.size == 0 or not np.isfinite(offsets).all():
        return None
    minimum = float(offsets.min())
    maximum = float(offsets.max())
    span = maximum - minimum
    if span <= 1.0e-9 or span > max(0.0, float(max_span_sec)):
        return None
    if reference == "end":
        reference_offset = maximum
    elif reference == "median":
        reference_offset = float(np.median(offsets))
    elif reference == "header":
        reference_offset = 0.0
    else:
        raise ValueError("reference must be end, median, or header")
    return {
        "minimum_offset_sec": minimum,
        "maximum_offset_sec": maximum,
        "span_sec": span,
        "reference": reference,
        "reference_offset_sec": reference_offset,
        "point_count": int(offsets.size),
    }


def normalize_angle(value):
    return np.arctan2(np.sin(value), np.cos(value))


class PlanarPoseBuffer:
    """Thread-safe time-ordered planar ego-pose buffer.

    The buffer intentionally stores poses rather than velocity.  MORAI bags
    provide `/Ego_topic` at a substantially higher rate than the LiDAR and
    pose interpolation avoids assumptions about the frame/unit of the
    velocity fields.
    """

    def __init__(self, duration_sec=2.0):
        self.duration_sec = max(0.2, float(duration_sec))
        self._poses = deque()
        self._lock = threading.Lock()

    def add(self, stamp_sec, x, y, z, yaw):
        sample = tuple(float(v) for v in (stamp_sec, x, y, z, yaw))
        if not all(math.isfinite(v) for v in sample):
            return False
        with self._lock:
            if self._poses and sample[0] < self._poses[-1][0] - 1.0e-9:
                self._poses.clear()
            if self._poses and abs(sample[0] - self._poses[-1][0]) <= 1.0e-9:
                self._poses[-1] = sample
            else:
                self._poses.append(sample)
            cutoff = sample[0] - self.duration_sec
            while len(self._poses) > 2 and self._poses[1][0] < cutoff:
                self._poses.popleft()
        return True

    def snapshot(self):
        with self._lock:
            if len(self._poses) < 2:
                return None
            values = np.asarray(tuple(self._poses), dtype=np.float64)
        order = np.argsort(values[:, 0], kind="stable")
        values = values[order]
        unique = np.concatenate(([True], np.diff(values[:, 0]) > 1.0e-9))
        values = values[unique]
        if len(values) < 2:
            return None
        values[:, 4] = np.unwrap(values[:, 4])
        return values


def interpolate_planar_poses(pose_samples, query_times, maximum_extrapolation_sec=0.03):
    """Interpolate ``[x, y, z, yaw]`` at query times, or return ``None``."""
    if pose_samples is None or len(pose_samples) < 2:
        return None
    query = np.asarray(query_times, dtype=np.float64)
    if query.size == 0 or not np.isfinite(query).all():
        return None
    stamps = pose_samples[:, 0]
    tolerance = max(0.0, float(maximum_extrapolation_sec))
    if query.min() < stamps[0] - tolerance or query.max() > stamps[-1] + tolerance:
        return None
    clipped = np.clip(query, stamps[0], stamps[-1])
    result = np.column_stack(
        [np.interp(clipped, stamps, pose_samples[:, index]) for index in range(1, 5)]
    )
    # A LiDAR callback can arrive a few milliseconds after its latest causal
    # Ego sample. Extrapolate only inside the explicitly bounded tolerance,
    # using the last two poses rather than freezing a fast-moving Ego pose.
    before = query < stamps[0]
    after = query > stamps[-1]
    if before.any():
        span = stamps[1] - stamps[0]
        if span <= 1.0e-9:
            return None
        ratio = (query[before] - stamps[0]) / span
        result[before] = pose_samples[0, 1:5] + ratio[:, None] * (
            pose_samples[1, 1:5] - pose_samples[0, 1:5]
        )
    if after.any():
        span = stamps[-1] - stamps[-2]
        if span <= 1.0e-9:
            return None
        ratio = (query[after] - stamps[-1]) / span
        result[after] = pose_samples[-1, 1:5] + ratio[:, None] * (
            pose_samples[-1, 1:5] - pose_samples[-2, 1:5]
        )
    result[:, 3] = normalize_angle(result[:, 3])
    return result


def _lidar_world_pose(ego_poses, lidar_position, lidar_yaw):
    ego_poses = np.asarray(ego_poses, dtype=np.float64)
    lidar_position = np.asarray(lidar_position, dtype=np.float64)
    cosine = np.cos(ego_poses[:, 3])
    sine = np.sin(ego_poses[:, 3])
    x = ego_poses[:, 0] + cosine * lidar_position[0] - sine * lidar_position[1]
    y = ego_poses[:, 1] + sine * lidar_position[0] + cosine * lidar_position[1]
    z = ego_poses[:, 2] + lidar_position[2]
    yaw = normalize_angle(ego_poses[:, 3] + float(lidar_yaw))
    return np.column_stack((x, y, z, yaw))


def deskew_points_to_reference_pose(
    points,
    point_time_offsets,
    scan_stamp_sec,
    reference_time_offset_sec,
    pose_samples,
    lidar_position=(0.0, 0.0, 0.0),
    lidar_yaw=0.0,
    maximum_extrapolation_sec=0.03,
):
    """Express every LiDAR point in the LiDAR frame at one reference time.

    Point coordinates are transformed using the interpolated ego pose at the
    individual acquisition time.  Intensity and any columns after XYZ remain
    unchanged.  Returns ``(corrected, displacement, reference_pose)`` or
    ``None`` when timing/pose coverage is insufficient.
    """
    values = np.asarray(points)
    offsets = np.asarray(point_time_offsets, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < 3 or len(values) != len(offsets):
        return None
    finite = np.isfinite(offsets)
    if not finite.all() or not len(values):
        return None

    point_times = float(scan_stamp_sec) + offsets
    reference_time = float(scan_stamp_sec) + float(reference_time_offset_sec)
    query_times = np.concatenate((point_times, np.asarray([reference_time])))
    ego_poses = interpolate_planar_poses(
        pose_samples, query_times, maximum_extrapolation_sec
    )
    if ego_poses is None:
        return None

    point_lidar_poses = _lidar_world_pose(ego_poses[:-1], lidar_position, lidar_yaw)
    reference_lidar_pose = _lidar_world_pose(
        ego_poses[-1:].reshape(1, 4), lidar_position, lidar_yaw
    )[0]

    point_yaw = point_lidar_poses[:, 3]
    cos_point = np.cos(point_yaw)
    sin_point = np.sin(point_yaw)
    world_x = (
        point_lidar_poses[:, 0]
        + cos_point * values[:, 0]
        - sin_point * values[:, 1]
    )
    world_y = (
        point_lidar_poses[:, 1]
        + sin_point * values[:, 0]
        + cos_point * values[:, 1]
    )
    world_z = point_lidar_poses[:, 2] + values[:, 2]

    dx = world_x - reference_lidar_pose[0]
    dy = world_y - reference_lidar_pose[1]
    cos_ref = math.cos(reference_lidar_pose[3])
    sin_ref = math.sin(reference_lidar_pose[3])

    corrected = np.array(values, copy=True)
    corrected[:, 0] = cos_ref * dx + sin_ref * dy
    corrected[:, 1] = -sin_ref * dx + cos_ref * dy
    corrected[:, 2] = world_z - reference_lidar_pose[2]
    displacement = np.linalg.norm(corrected[:, :3] - values[:, :3], axis=1)
    return corrected, displacement, reference_lidar_pose


def transform_boxes_between_lidar_times(
    boxes,
    source_time_sec,
    target_time_sec,
    pose_samples,
    lidar_position=(0.0, 0.0, 0.0),
    lidar_yaw=0.0,
    maximum_extrapolation_sec=0.03,
):
    """Re-express rigid boxes from one time-indexed LiDAR frame to another."""
    values = np.asarray(boxes)
    if values.ndim != 2 or values.shape[1] < 7 or not len(values):
        return np.array(values, copy=True)
    poses = interpolate_planar_poses(
        pose_samples,
        np.asarray([source_time_sec, target_time_sec], dtype=np.float64),
        maximum_extrapolation_sec,
    )
    if poses is None:
        return None
    lidar_poses = _lidar_world_pose(poses, lidar_position, lidar_yaw)
    source, target = lidar_poses

    cos_source = math.cos(source[3])
    sin_source = math.sin(source[3])
    world_x = source[0] + cos_source * values[:, 0] - sin_source * values[:, 1]
    world_y = source[1] + sin_source * values[:, 0] + cos_source * values[:, 1]
    world_z = source[2] + values[:, 2]

    dx = world_x - target[0]
    dy = world_y - target[1]
    cos_target = math.cos(target[3])
    sin_target = math.sin(target[3])
    output = np.array(values, copy=True)
    output[:, 0] = cos_target * dx + sin_target * dy
    output[:, 1] = -sin_target * dx + cos_target * dy
    output[:, 2] = world_z - target[2]
    output[:, 6] = normalize_angle(values[:, 6] + source[3] - target[3])
    if values.shape[1] >= 9:
        # vx/vy are absolute planar velocity components expressed in the
        # source LiDAR axes.  Re-expression at a different Ego yaw rotates
        # the vector but does not apply the frame translation.
        delta_yaw = source[3] - target[3]
        cosine = math.cos(delta_yaw)
        sine = math.sin(delta_yaw)
        output[:, 7] = cosine * values[:, 7] - sine * values[:, 8]
        output[:, 8] = sine * values[:, 7] + cosine * values[:, 8]
    return output


def transform_points_between_lidar_times(
    points,
    source_time_sec,
    target_time_sec,
    pose_samples,
    lidar_position=(0.0, 0.0, 0.0),
    lidar_yaw=0.0,
    maximum_extrapolation_sec=0.03,
):
    """Re-express a stored sweep in another time-indexed LiDAR frame.

    All points share the stored sweep reference time.  This aligns complete
    past sweeps to the current Ego pose without performing intra-scan deskew,
    and preserves intensity and painted semantic features after XYZ.
    """
    values = np.asarray(points)
    if values.ndim != 2 or values.shape[1] < 3 or not len(values):
        return np.array(values, copy=True)
    transformed = deskew_points_to_reference_pose(
        values,
        np.zeros(len(values), dtype=np.float64),
        float(source_time_sec),
        float(target_time_sec) - float(source_time_sec),
        pose_samples,
        lidar_position=lidar_position,
        lidar_yaw=lidar_yaw,
        maximum_extrapolation_sec=maximum_extrapolation_sec,
    )
    if transformed is None:
        return None
    return transformed[0]


def representative_detection_time_offset(
    points: np.ndarray,
    point_time_offsets: np.ndarray,
    boxes: np.ndarray,
    extent_scale: float = 0.65,
    minimum_points_per_box: int = 3,
) -> float:
    """Return one robust measurement-time offset for a detection array.

    A standard DetectedObjects array has one header although a rolling scan
    observes different boxes at different instants.  Calculate each box's
    median point time first, then take their median so large/dense boxes do not
    dominate the array timestamp.  Fall back to the scan median when no box
    contains enough timed points, and to zero when no usable timing exists.
    """
    if len(points) != len(point_time_offsets) or len(points) == 0:
        return 0.0
    finite_time = np.isfinite(point_time_offsets)
    if not finite_time.any():
        return 0.0

    object_offsets = []
    for box in boxes:
        x, y, _, length, width, _, yaw = [float(value) for value in box[:7]]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        dx = points[:, 0] - x
        dy = points[:, 1] - y
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        mask = (
            finite_time
            & (np.abs(local_x) <= max(0.2, length) * extent_scale)
            & (np.abs(local_y) <= max(0.2, width) * extent_scale)
        )
        if int(mask.sum()) >= minimum_points_per_box:
            object_offsets.append(float(np.median(point_time_offsets[mask])))

    if object_offsets:
        return float(np.median(np.asarray(object_offsets, dtype=np.float64)))
    return float(np.median(point_time_offsets[finite_time]))
