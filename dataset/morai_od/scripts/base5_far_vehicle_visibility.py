"""Proposal-only 3-D blockage heuristic for a declared far secondary vehicle.

Unlike the older physical-obstacle proxy, vehicle center Z uses the measured
pivot-to-box-center offset, not half the body height. Native GT/QA are untouched.
"""
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import base5_primary_visibility_proposal as rays
import dynamic_capture_metadata as sensor_metadata


POLICY = 'far-secondary-single-foreground-3d-proposal-v1'
DEFAULT_GEOMETRY = Path(__file__).resolve().parents[1]/'configs/morai_asset_bbox_offsets_base3_v1.json'
DEFAULT_SENSOR = Path(__file__).resolve().parents[3]/'configs/sensors/Sensor_competition_local.json'
TARGET_EXPAND_M = .05
FOREGROUND_SHRINK_M = .10
LIMITATION = ('Proposal heuristic from measured upright parent boxes and 27 rays, '
    'not mesh visibility, sensor returns, GT exclusion, native QA approval or a QA waiver. '
    'Vehicle windows/voids and reference uncertainty are not mesh-certified; '
    'unknown height, unsupported motion, compound and partial occlusion stay eligible.')


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _pin(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


@lru_cache(maxsize=8)
def _calibration(path, sensor_sha256, normalizer_sha256):
    # The caller rehashes files; only parsing and trigonometry are cached.
    return sensor_metadata.normalized_sensor_calibration(path)


def configuration(sensor=DEFAULT_SENSOR, geometry_reference=DEFAULT_GEOMETRY):
    """Explicit proposal provenance; no native approval or implicit fallback."""
    source = _pin(sensor)
    normalizer = _pin(sensor_metadata.__file__)
    calibration = _calibration(source['path'], source['sha256'], normalizer['sha256'])
    return dict(policy=POLICY, implementation=_pin(__file__), ray_helper=_pin(rays.__file__),
        geometry_reference=_pin(geometry_reference), sensor=source, normalizer=normalizer,
        normalized_calibration_sha256=_digest(calibration), model_input=[512, 1056],
        target_expand_m=TARGET_EXPAND_M, foreground_shrink_per_surface_m=FOREGROUND_SHRINK_M,
        partial_and_uncertain_retained=True, native_qa_unchanged=True)


def validate_policy(policy, calibration=None):
    """Return the pinned normalized input or halt on any dependency drift."""
    try:
        expected = configuration(policy['sensor']['path'], policy['geometry_reference']['path'])
        if policy != expected:
            raise RuntimeError('Far visibility proposal source/reference/calibration contract changed')
        if calibration is not None and _digest(calibration) != policy['normalized_calibration_sha256']:
            raise RuntimeError('Far visibility supplied calibration differs from frozen policy')
        return _calibration(policy['sensor']['path'], policy['sensor']['sha256'],
                            policy['normalizer']['sha256'])
    except (OSError, KeyError, TypeError, ValueError) as error:
        raise RuntimeError('Malformed far visibility proposal policy') from error


@lru_cache(maxsize=8)
def _load_reference(path, expected):
    return json.loads(Path(path).read_text())['assets']


def _reference(value):
    try:
        if value is None or isinstance(value, (str, Path)):
            path = Path(value or DEFAULT_GEOMETRY).resolve()
            data = path.read_bytes()
            sha = hashlib.sha256(data).hexdigest()
            return _load_reference(str(path), sha), dict(path=str(path), sha256=sha)
        if not isinstance(value, dict):
            raise RuntimeError('Far visibility geometry reference must be a path or mapping')
        assets = value.get('assets', value)
        if not isinstance(assets, dict):
            raise RuntimeError('Far visibility geometry assets must be a mapping')
        return assets, dict(kind='explicit-inline-geometry', sha256=_digest(assets))
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise RuntimeError('Far visibility geometry reference is malformed/unavailable') from error


def _vehicle_box(actor, ego, assets):
    if (actor.get('source_list') != 'vehicleList' or actor.get('motion') != 'static'
            or not rays._single_known_actor(actor) or actor.get('height_uncertain')):
        return None
    ref = assets.get(actor.get('name'))
    if (not isinstance(ref, dict) or str(ref.get('class', '')).lower() != 'vehicle'
            or not isinstance(ref.get('observations'), (float, int)) or ref['observations'] <= 0
            or ref.get('height_uncertain')):
        return None
    if 'pivot_to_bbox_center_local_m' not in ref or 'bbox_size_m' not in ref:
        return None
    offset = rays._finite(ref['pivot_to_bbox_center_local_m'], (3,), 'vehicle center offset')
    reference_size = rays._finite(ref['bbox_size_m'], (3,), 'reference vehicle size')
    if (reference_size <= 0).any():
        raise RuntimeError('Far visibility reference vehicle size is nonpositive')
    center, half, rotation = rays._obb(actor, ego)
    if not np.allclose(half*2, reference_size, atol=1e-5, rtol=1e-4):
        return None  # Different placement envelope is not the measured body.
    center[2] = float(actor['position']['z'])-float(ego['z'])+offset[2]
    if not np.isfinite(center).all():
        raise RuntimeError('Far visibility vehicle center is nonfinite')
    return center, half, rotation


def _origins512(corners, calibration):
    cameras = calibration.get('cameras')
    if not isinstance(cameras, dict) or not cameras:
        raise RuntimeError('Far visibility requires normalized camera calibration')
    origins, names, uncertain = [], [], False
    for name, camera in sorted(cameras.items()):
        try:
            transform = rays._transform(camera['ego_to_sensor'], 'camera transform')
            intrinsic = rays._finite(camera['intrinsic'], (3, 3), 'camera intrinsic')
            width, height = rays._finite(camera['resolution_px'], (2,), 'camera resolution')
        except KeyError as error:
            raise RuntimeError('Far visibility camera calibration missing') from error
        if (width <= 0 or height <= 0 or intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0
                or not np.allclose(intrinsic[2], [0., 0., 1.], atol=1e-8)):
            raise RuntimeError('Far visibility camera projection is invalid')
        scale = 1056./width
        resized_height = int(height*scale)
        if resized_height < 512:
            raise RuntimeError('Far visibility current crop exceeds resized camera height')
        top, bottom = resized_height-512, resized_height
        optical = (np.c_[corners, np.ones(len(corners))]@transform.T)[:, :3]
        projected = optical@intrinsic.T
        if not np.isfinite(projected).all():
            raise RuntimeError('Far visibility projection is nonfinite')
        x, y, z = projected[:, 0]*scale, projected[:, 1]*scale, optical[:, 2]
        # A linear half-space can prove even a near-plane-straddling box is
        # outside the view. Otherwise a true clipping ambiguity stays eligible.
        planes = np.c_[x, 1056*z-x, y-top*z, bottom*z-y, z-.1]
        if np.any(np.all(planes < 0, axis=0)):
            continue
        if not np.all(z > .1):
            uncertain = True
            continue
        pixels = np.c_[x/z, y/z]
        low = np.maximum(pixels.min(axis=0), [0., top])
        high = np.minimum(pixels.max(axis=0), [1056., bottom])
        if (high > low).all():
            origins.append(-transform[:3, :3].T@transform[:3, 3])
            names.append(str(name))
    return origins, names, uncertain


def evaluate(meta, calibration, geometry_reference=None):
    """Return diagnostics; reject only a complete *single* known NPC blocker.

    A nominal LiDAR beam intersecting the target does not bypass this proof:
    that beam can itself be blocked by the foreground vehicle.
    """
    proof = dict(policy=POLICY, applicable=False, rejected=False, reason='unplanned_source',
        samples_per_sensor=27, target_expand_m=TARGET_EXPAND_M,
        foreground_shrink_per_surface_m=FOREGROUND_SHRINK_M, model_input=[512, 1056],
        foreground_candidates=[], limitation=LIMITATION)
    plan = meta.get('spec', {}).get('secondary_far_vehicle')
    if not isinstance(plan, dict):
        return proof
    actors = meta.get('runtime_actors', [])
    targets = [a for a in actors if a.get('unique_id') == plan.get('target_uid')]
    if (len(targets) != 1 or targets[0].get('data_id') != plan.get('target_data_id')
            or targets[0].get('name') != plan.get('target_name')):
        proof['reason'] = 'unknown_or_ambiguous_target_identity'
        return proof
    target = targets[0]
    assets, reference = _reference(geometry_reference)
    proof.update(applicable=True, geometry_reference=reference, target_uid=target['unique_id'],
        target_data_id=target['data_id'], reason='unknown_or_uncertain_vehicle_height')
    box = _vehicle_box(target, meta['ego'], assets)
    if box is None:
        return proof
    center, half, rotation = box
    samples = (rays._GRID*(half+TARGET_EXPAND_M))@rotation.T+center
    corners = (rays._CORNERS*(half+TARGET_EXPAND_M))@rotation.T+center
    origins, names, uncertain = _origins512(corners, calibration)
    proof.update(target_center_ego_m=center.tolist(), center_z_policy='saved-pivot-offset-z; not half-height')
    if uncertain or not origins:
        proof['reason'] = 'uncertain_camera_near_plane' if uncertain else 'no_current_camera_witness'
        return proof
    try:
        lidar = rays._transform(calibration['lidar']['ego_to_lidar'], 'LiDAR transform')
    except KeyError as error:
        raise RuntimeError('Far visibility requires normalized LiDAR calibration') from error
    origins = np.asarray([-lidar[:3, :3].T@lidar[:3, 3]]+origins)
    names = ['lidar']+names
    proof.update(sensor_names=names, reason='no_complete_single_foreground')
    for foreground in actors:
        if foreground is target:
            continue
        box = _vehicle_box(foreground, meta['ego'], assets)
        if box is None:
            continue
        other_center, other_half, other_rotation = box
        if other_half[0]*2 < 3. or other_half[1]*2 < 1.4:
            continue
        other_half = other_half-FOREGROUND_SHRINK_M
        if (other_half <= 0).any():
            continue
        hits = rays._ray_hits(origins, samples, other_center, other_half, other_rotation)
        candidate = dict(unique_id=foreground.get('unique_id'), data_id=foreground.get('data_id'),
            name=foreground.get('name'), center_ego_m=other_center.tolist(),
            blocked_samples_by_sensor={name: int(n) for name, n in zip(names, hits.sum(axis=1))})
        proof['foreground_candidates'].append(candidate)
        if hits.all():
            proof.update(rejected=True, reason='same_foreground_blocks_all_expanded_samples', foreground=candidate)
            break
    return proof


def score(meta, calibration, kind, score_fn, geometry_reference=None):
    original = score_fn(meta, calibration, kind)
    if 'secondary_far_vehicle' not in meta.get('spec', {}):
        return original
    proof = evaluate(meta, calibration, geometry_reference)
    result = dict(original, far_vehicle_visibility=proof)
    if not math.isfinite(float(original['score'])):
        raise RuntimeError('Far visibility original ranker returned a nonfinite score')
    if proof['rejected']:
        result['score'] = -float('inf')
    return result
