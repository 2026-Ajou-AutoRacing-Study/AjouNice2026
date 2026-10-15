"""Conservative 3-D proposal rejection for frozen primary-range recovery only.

This does not approve a capture or change native QA. An estimated OBB is not a
mesh: partial/uncertain occlusion is retained, and ordinary 2-D angular overlap
is never a rejection reason. The original scorer remains the ranking source.
"""
import itertools
import math

import numpy as np


POLICY = 'primary-range-single-foreground-3d-proposal-v1'
PRIMARY_EXPAND_M = .05
FOREGROUND_SHRINK_M = .10
_GRID = np.asarray(list(itertools.product((-1., 0., 1.), repeat=3)))
_CORNERS = np.asarray(list(itertools.product((-1., 1.), repeat=3)))
LIMITATION = ('Estimated upright parent OBBs and 27 rays, not rendered mesh visibility, '
              'native annotation evidence, collision approval or a QA waiver; '
              'candidate-generation heuristic only; uncertain height, partial coverage, '
              'compound actors and non-solid/unknown foreground shapes are retained.')


def _finite(value, shape, label):
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise RuntimeError('Invalid visibility proposal ' + label) from error
    if array.shape != shape or not np.isfinite(array).all():
        raise RuntimeError('Nonfinite or malformed visibility proposal ' + label)
    return array


def _transform(value, label):
    result = _finite(value, (4, 4), label)
    rotation = result[:3, :3]
    if (not np.allclose(result[3], [0., 0., 0., 1.], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not math.isclose(float(np.linalg.det(rotation)), 1., abs_tol=1e-6)):
        raise RuntimeError('Nonrigid visibility proposal ' + label)
    return result


def _single_known_actor(actor):
    """Explicit uncertainty cannot be used to prove a complete blocker."""
    if (actor.get('geometry_reference_known') is not True
            or actor.get('source_list') not in ('objectList', 'vehicleList')
            or actor.get('motion') != 'static'
            or actor.get('is_compound') or actor.get('compound')):
        return False
    for key in ('component_count', 'annotation_component_count'):
        if key in actor and actor[key] != 1:
            return False
    ground = actor.get('map_ground_initializer')
    if not isinstance(ground, dict) or ground.get('usable') is not True:
        return False
    if ground.get('footprint', {}).get('all_supported') is False:
        return False
    initializer = actor.get('static_height_initializer')
    if isinstance(initializer, dict) and initializer.get('usable') is False:
        return False
    return all(key in actor for key in ('estimated_size', 'position',
        'longitudinal_m', 'lateral_m', 'placement_yaw_deg'))


def _obb(actor, ego):
    size = _finite(actor['estimated_size'], (3,), 'actor size')
    if (size <= 0).any():
        raise RuntimeError('Nonpositive visibility proposal actor size')
    try:
        center = _finite([actor['longitudinal_m'], actor['lateral_m'],
            actor['position']['z'] - ego['z'] + size[2] / 2], (3,), 'actor center')
        angle = math.radians(float(actor['placement_yaw_deg']) - float(ego['yaw']))
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError('Malformed visibility proposal actor pose') from error
    if not math.isfinite(angle):
        raise RuntimeError('Nonfinite visibility proposal yaw')
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.asarray([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    return center, size / 2, rotation


def _camera_origins(corners, calibration):
    """Use exactly the legacy 320x896 bottom-crop pinhole convention."""
    cameras = calibration.get('cameras')
    if not isinstance(cameras, dict) or not cameras:
        raise RuntimeError('Visibility proposal requires camera calibration')
    origins, names, uncertain = [], [], False
    homogeneous = np.c_[corners, np.ones(len(corners))]
    for name, camera in sorted(cameras.items()):
        try:
            transform = _transform(camera['ego_to_sensor'], 'camera transform')
            intrinsic = _finite(camera['intrinsic'], (3, 3), 'camera intrinsic')
            width, height = _finite(camera['resolution_px'], (2,), 'camera resolution')
        except KeyError as error:
            raise RuntimeError('Missing visibility proposal camera calibration') from error
        if (width <= 0 or height <= 0 or intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0
                or not np.allclose(intrinsic[2], [0., 0., 1.], atol=1e-8)):
            raise RuntimeError('Invalid visibility proposal camera FoV')
        optical = (homogeneous @ transform.T)[:, :3]
        forward = optical[:, 2] > .1
        if not forward.any():
            continue
        if not forward.all():
            # Near-plane clipping needs more than the existing bounding proxy.
            uncertain = True
            continue
        pixels = optical @ intrinsic.T
        pixels = pixels[:, :2] / pixels[:, 2:3]
        if not np.isfinite(pixels).all():
            raise RuntimeError('Nonfinite visibility proposal camera projection')
        scale = 896. / width
        top = max(0., height - 320. / scale)
        low = np.maximum(pixels.min(axis=0), [0., top])
        high = np.minimum(pixels.max(axis=0), [width, height])
        if (high > low).all():
            origins.append(-transform[:3, :3].T @ transform[:3, 3])
            names.append(str(name))
    return origins, names, uncertain


def _ray_hits(origins, targets, center, half, rotation):
    """Segment/OBB slab intervals, vectorized across sensors and 27 targets."""
    local_origins = (origins - center) @ rotation
    directions = (targets[None, :, :] - origins[:, None, :]) @ rotation
    start = local_origins[:, None, :]
    parallel = np.abs(directions) <= 1e-12
    outside = np.any(parallel & (np.abs(start) > half), axis=2)
    safe_direction = np.where(parallel, 1., directions)
    first = (-half - start) / safe_direction
    second = (half - start) / safe_direction
    entry = np.max(np.where(parallel, -np.inf, np.minimum(first, second)), axis=2)
    leave = np.min(np.where(parallel, np.inf, np.maximum(first, second)), axis=2)
    # The sensor and each expanded primary sample must be outside the blocker.
    # This is deliberately stricter than merely intersecting the line segment.
    origin_inside = np.all(np.abs(local_origins) <= half, axis=1)
    closer = np.linalg.norm(center - origins, axis=1)[:, None] < np.linalg.norm(
        targets[None, :, :] - origins[:, None, :], axis=2)
    return (~outside & ~origin_inside[:, None] & closer & (entry > 1e-9)
            & (leave >= entry) & (leave < 1. - 1e-9))


def score(meta, calibration, kind, score_fn):
    """Wrap ``score_fn(meta, calibration, kind)`` without changing legacy rows.

    A rejection requires zero predicted LiDAR beams, a supported physical
    single-component primary, at least one relevant camera, and the *same*
    supported foreground covering all 27 expanded samples from every camera
    and LiDAR origin. Native capture acceptance is entirely unaffected.
    """
    original = score_fn(meta, calibration, kind)
    plan = meta.get('spec', {}).get('primary_obstacle_range')
    actors = meta.get('runtime_actors', [])
    if (not isinstance(plan, dict) or not actors
            or plan.get('semantic_class') != 'obstacle'
            or plan.get('source_list') != 'objectList'
            or actors[0].get('source_list') != 'objectList'
            or actors[0].get('motion') != 'static'
            or plan.get('primary_data_id') != actors[0].get('data_id')):
        return original
    result = dict(original)
    proof = dict(policy=POLICY, rejected=False, reason='uncertain_primary_geometry',
        primary_expand_m=PRIMARY_EXPAND_M, foreground_shrink_per_surface_m=FOREGROUND_SHRINK_M,
        samples_per_sensor=27, foreground_candidates=[], limitation=LIMITATION)
    result['primary_visibility_proposal'] = proof
    try:
        base_score = float(original['score'])
        beam_proxy = float(original['primary']['lidar_beam_proxy'])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError('Malformed original visibility proposal score') from error
    if not math.isfinite(base_score) or not math.isfinite(beam_proxy) or beam_proxy < 0:
        raise RuntimeError('Nonfinite original visibility proposal score/beam proxy')
    if beam_proxy != 0:
        proof['reason'] = 'primary_lidar_beam_proxy_positive'
        return result
    if not _single_known_actor(actors[0]):
        return result
    center, half, rotation = _obb(actors[0], meta['ego'])
    targets = (_GRID * (half + PRIMARY_EXPAND_M)) @ rotation.T + center
    corners = (_CORNERS * (half + PRIMARY_EXPAND_M)) @ rotation.T + center
    origins, names, uncertain = _camera_origins(corners, calibration)
    if uncertain or not origins:
        proof['reason'] = 'uncertain_camera_near_plane' if uncertain else 'no_in_crop_camera_witness'
        return result
    try:
        lidar = _transform(calibration['lidar']['ego_to_lidar'], 'LiDAR transform')
    except KeyError as error:
        raise RuntimeError('Missing visibility proposal LiDAR calibration') from error
    origins = np.asarray([-lidar[:3, :3].T @ lidar[:3, 3]] + origins)
    names = ['lidar'] + names
    proof.update(sensor_names=names, sensor_count=len(names), reason='no_complete_single_foreground')
    for actor in actors[1:]:
        if actor.get('source_list') != 'vehicleList' or not _single_known_actor(actor):
            continue
        other_center, other_half, other_rotation = _obb(actor, meta['ego'])
        # Limit this heuristic to ordinary NPC-sized solid vehicle bodies. An
        # obstacle/pedestrian parent envelope can contain large empty regions.
        if other_half[0] * 2 < 3. or other_half[1] * 2 < 1.4:
            continue
        other_half = other_half - FOREGROUND_SHRINK_M
        if (other_half <= 0).any():
            continue
        hits = _ray_hits(origins, targets, other_center, other_half, other_rotation)
        candidate = dict(unique_id=actor.get('unique_id'), data_id=actor.get('data_id'),
            name=actor.get('name'), blocked_samples_by_sensor={name: int(n)
                for name, n in zip(names, hits.sum(axis=1))})
        proof['foreground_candidates'].append(candidate)
        if hits.all():
            proof.update(rejected=True, reason='same_foreground_blocks_all_expanded_samples',
                foreground=candidate)
            result['score'] = -float('inf')
            break
    return result
