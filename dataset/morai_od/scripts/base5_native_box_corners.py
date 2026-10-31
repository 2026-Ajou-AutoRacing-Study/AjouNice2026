"""Strict native 8-corner geometry derivation; never rewrites raw annotations.

MORAI can store stale scale-1 dimensions in Instance.txt while the companion
Instance_8Points.txt already contains the scaled box. This module derives local
extents from those measured corners, not from scenario scale factors. Adoption
is an explicit caller policy: changed dimensions are rejected by default.
"""
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np


POLICY = 'native-eight-corner-local-extent-v155'
SCHEMA = 'base5_native_box_corners_v1'
CENTER_TOLERANCE_M = .001
CORNER_TOLERANCE_M = .001
ROTATION_TOLERANCE_RAD = 2.e-5
SIZE_ABSOLUTE_TOLERANCE_M = .001
SIZE_RELATIVE_TOLERANCE = .001


class BoxCornerContractError(RuntimeError):
    """An invalid/mismatched native geometry source; never an ordinary deficit."""


def implementation_sha256():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _vector(value, positive=False):
    try:
        result = np.asarray(value, dtype=np.float64)
    except (ValueError, TypeError) as error:
        raise BoxCornerContractError('Numeric three-axis native vector required') from error
    if (result.shape != (3,) or not np.isfinite(result).all()
            or (positive and (result <= 0).any())):
        raise BoxCornerContractError('Finite native vectors and positive dimensions required')
    return result


def _uid(value):
    if isinstance(value, bool):
        raise BoxCornerContractError('Integer native parent UID required')
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise BoxCornerContractError('Integer native parent UID required') from error
    if str(result) != str(value):
        raise BoxCornerContractError('Nonintegral native parent UID')
    return result


def _rotation(rpy):
    r, p, y = _vector(rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.asarray([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]])


def _read(path):
    if not path.is_file() or path.is_symlink():
        raise BoxCornerContractError('Exact native annotation companion file required; no symlinks')
    before = path.stat()
    data = path.read_bytes()
    after = path.stat()
    stamp = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
    if stamp(before) != stamp(after):
        raise BoxCornerContractError('Native annotation changed while reading')
    try:
        text = data.decode('utf-8', errors='strict')
    except UnicodeDecodeError as error:
        raise BoxCornerContractError('Invalid native annotation encoding') from error
    return text, dict(path=str(path), sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))


def load_matching_corners(annotation_path):
    """Return records keyed by ``(parent_uid, raw_class, component_ordinal)``.

    Both files must share the *exact* timestamp/basename. Malformed records,
    unmatched classes/UIDs/component counts or missing companions are fatal.
    No nearest-time filename search, basename renaming, or permissive parsing.
    """
    annotation_path = Path(annotation_path).absolute()
    suffix = '_Instance.txt'
    if not annotation_path.name.endswith(suffix):
        raise BoxCornerContractError('Expected exact *_Instance.txt source path')
    companion = annotation_path.with_name(annotation_path.name[:-len(suffix)] + '_Instance_8Points.txt')
    annotation_text, annotation_pin = _read(annotation_path)
    corner_text, corner_pin = _read(companion)
    sources, corners = defaultdict(list), defaultdict(list)
    for text, count, target, label in ((annotation_text, 15, sources, 'Instance'),
                                       (corner_text, 33, corners, '8Points')):
        for number, line in enumerate(text.splitlines(), 1):
            if not line.strip():
                continue
            fields = line.split()
            if len(fields) != count:
                raise BoxCornerContractError(f'Malformed {label} record at line {number}')
            try:
                numeric = np.asarray([float(v) for v in fields[1:-1]], dtype=np.float64)
            except ValueError as error:
                raise BoxCornerContractError(f'Invalid {label} numeric record') from error
            if not np.isfinite(numeric).all():
                raise BoxCornerContractError(f'Nonfinite {label} record')
            key = (_uid(fields[-1]), fields[0])
            target[key].append((numeric, number))
    if not sources or set(sources) != set(corners) or any(len(sources[k]) != len(corners[k]) for k in sources):
        raise BoxCornerContractError('Instance and 8Points class/UID/component counts differ')
    parent_counts = Counter()
    for (uid, _), values in sources.items():
        parent_counts[uid] += len(values)
    result = {}
    for (uid, raw_class), values in sources.items():
        for ordinal, ((annotation, annotation_line), (corner, corner_line)) in enumerate(zip(values, corners[(uid, raw_class)])):
            record = dict(schema=SCHEMA, parent_uid=uid,
                raw_class=raw_class, component_ordinal=ordinal, parent_component_count=parent_counts[uid],
                corners_lidar_m=corner[:24].reshape(8, 3).tolist(), rotation_rpy=corner[24:27].tolist(),
                annotation_center_lidar_m=annotation[:3].tolist(),
                annotation_rotation_rpy=annotation[3:6].tolist(), annotation_size_m=annotation[6:9].tolist(),
                annotation_line=annotation_line, corner_line=corner_line,
                source_files=dict(annotation=annotation_pin, corners=corner_pin),
                implementation_sha256=implementation_sha256())
            record['record_sha256'] = _digest(record)
            result[(uid, raw_class, ordinal)] = record
    return result


def derive_geometry(annotation_box, corner_record, *, allow_dimension_change=False):
    """Return center/size/RPY/evidence, preserving supplied native center and RPY.

    Only a single physical Obstacle component is supported in this policy.
    The caller must bind nonunit scenario metadata and its policy/checksum
    before explicitly setting ``allow_dimension_change=True``. This function
    performs no scale multiplication and is idempotent on already-correct size.
    """
    record = corner_record
    if (not isinstance(record, dict) or record.get('record_sha256') != _digest(
            {key: value for key, value in record.items() if key != 'record_sha256'})):
        raise BoxCornerContractError('Native corner record changed after parsing')
    for pin in record.get('source_files', {}).values():
        path = Path(pin['path'])
        _, current = _read(path)
        if current != pin:
            raise BoxCornerContractError('Native geometry source checksum changed after parsing')
    uid = _uid(annotation_box.get('unique_id', annotation_box.get('parent_uid')))
    if (not isinstance(record, dict) or record.get('schema') != SCHEMA
            or record.get('implementation_sha256') != implementation_sha256()
            or uid != record.get('parent_uid') or annotation_box.get('raw_class') != record.get('raw_class')
            or record.get('raw_class') != 'Obstacle' or record.get('component_ordinal') != 0
            or record.get('parent_component_count') != 1):
        raise BoxCornerContractError('Only matched single-component native Obstacle boxes are supported')
    center = _vector(annotation_box.get('center', annotation_box.get('center_lidar_m')))
    rpy = _vector(annotation_box.get('rotation_rpy'))
    size = _vector(annotation_box.get('size', annotation_box.get('size_m')), positive=True)
    source_center = _vector(record.get('annotation_center_lidar_m'))
    source_rpy = _vector(record.get('annotation_rotation_rpy'))
    corner_rpy = _vector(record.get('rotation_rpy'))
    angle_difference = lambda a, b: np.abs((a - b + np.pi) % (2 * np.pi) - np.pi)
    if (np.linalg.norm(center - source_center) > CENTER_TOLERANCE_M
            or max(angle_difference(rpy, source_rpy)) > ROTATION_TOLERANCE_RAD
            or max(angle_difference(rpy, corner_rpy)) > ROTATION_TOLERANCE_RAD):
        raise BoxCornerContractError('Native box/corner center or rotation indicates mismatched frames')
    try:
        corners = np.asarray(record.get('corners_lidar_m'), dtype=np.float64)
    except (ValueError, TypeError) as error:
        raise BoxCornerContractError('Eight finite native corners required') from error
    if corners.shape != (8, 3) or not np.isfinite(corners).all():
        raise BoxCornerContractError('Eight finite native corners required')
    if np.linalg.norm(corners.mean(0) - center) > CENTER_TOLERANCE_M:
        raise BoxCornerContractError('Eight-corner center differs from native annotation center')
    local = (corners - center) @ _rotation(rpy)
    low, high = local.min(0), local.max(0)
    extent = high - low
    if (extent <= CORNER_TOLERANCE_M * 2).any():
        raise BoxCornerContractError('Degenerate native corner box')
    tolerance = np.maximum(CORNER_TOLERANCE_M, extent * 1.e-4)
    if (np.abs((high + low) / 2) > tolerance).any():
        raise BoxCornerContractError('Native corner box is not symmetric about its center')
    signs = local >= 0
    expected = np.where(signs, extent / 2, -extent / 2)
    if (np.abs(local - expected) > tolerance).any() or len({tuple(s) for s in signs.tolist()}) != 8:
        raise BoxCornerContractError('Native corners do not form all eight vertices of the declared RPY box')
    same = bool(np.allclose(extent, size, atol=SIZE_ABSOLUTE_TOLERANCE_M,
                            rtol=SIZE_RELATIVE_TOLERANCE))
    if not same and not allow_dimension_change:
        raise BoxCornerContractError('Native corner dimensions differ; explicit bound variant policy required')
    result_size = size if same else extent
    return dict(center=center.tolist(), size=result_size.tolist(), rotation_rpy=rpy.tolist(),
        evidence=dict(policy=POLICY, schema=SCHEMA, helper_sha256=implementation_sha256(),
            parent_uid=uid, raw_class='Obstacle', component_ordinal=0,
            source_files=record['source_files'], annotation_size_m=record['annotation_size_m'],
            input_size_m=size.tolist(), corner_extent_local_m=extent.tolist(),
            dimensions_changed=not same, original_matching_size_preserved=same,
            raw_center_preserved=True, raw_rotation_preserved=True, raw_files_modified=False,
            scenario_scale_multiplied=False, center_residual_m=float(np.linalg.norm(corners.mean(0)-center)),
            maximum_corner_residual_m=float(np.max(np.abs(local-expected))),
            limitation='Native box geometry only; caller qualification must establish semantic/mesh/sensor agreement and nonunit metadata binding.'))
