"""Explicit small/low alongside proposal policy; never visibility or GT approval.

The original conservative placement envelope is retained. Its 0.30m OBB gap
uses the existing 4.8x2m Ego proxy, not a measured mesh/collider clearance.
Native unit sizes select a bounded policy population; they do not replace that
envelope, annotation, actor identity, phase, side, road support or capture QA.
"""
import copy
import itertools
import json
import math
from pathlib import Path

import numpy as np

import base5_unit_geometry_contract as unit


KEY = 'close_primary_observation'
POLICY = 'small-low-unit-alongside-forward-side-v158'


class ClosePrimaryContractError(RuntimeError):
    """An opt-in contract mismatch is fatal, not an ordinary proposal deficit."""


def configuration(unit_reference):
    return dict(policy=POLICY, native_unit_reference=copy.deepcopy(unit_reference),
        maximum_native_height_m=1., maximum_native_horizontal_size_m=1.5,
        primary_ego_x_bounds_m=[1., 1.5], effective_placement_surface_gap_m=.30,
        scope='static-single-physical-obstacle-alongside; original-gap-upper-bound<=0.30',
        placement_envelope_changed=False, native_gt_modified=False,
        camera_proposal_only=True, native_acceptance_unchanged=True)


def policy_for(config):
    policy = config.get(KEY)
    if (not isinstance(policy, dict)
            or policy != configuration(config.get(unit.KEY))
            or config.get(unit.KEY) is None):
        raise ClosePrimaryContractError('Explicit close-primary config/unit policy differs')
    return policy, unit.load(policy['native_unit_reference'])


def eligible(metadata, policy, native):
    """Outcome-independent inventory criterion; no ID or observed QA exceptions."""
    row = metadata.get('spec', {})
    actors = metadata.get('runtime_actors') or []
    if not actors:
        return False
    actor = actors[0]
    record = native['assets'].get(actor.get('name'), {})
    size = record.get('native_size_lwh_m')
    bounds = row.get('avoidance_gap_bounds_m')
    return bool(row.get('family') == 'obstacle' and row.get('motion') == 'static'
        and row.get('avoidance_phase') == 'alongside'
        and row.get('role') in ('left', 'right')
        and row.get('avoidance_side') == ('right' if row['role'] == 'left' else 'left')
        and isinstance(bounds, list) and len(bounds) == 2
        and all(type(x) in (int, float) and math.isfinite(x) for x in bounds)
        and 0 <= bounds[0] < bounds[1] <= .30
        and metadata.get(unit.KEY) == policy['native_unit_reference']
        and actor.get('source_list') == record.get('source_list') == 'objectList'
        and actor.get('data_id') == record.get('data_id')
        and actor.get('motion') == 'static' and actor.get('geometry_reference_known') is True
        and not actor.get('asset_scale') and not row.get('asset_scale_plan')
        and not actor.get('compound') and not actor.get('is_compound')
        and actor.get('component_count', 1) == 1
        and isinstance(size, list) and len(size) == 3
        and size[2] <= policy['maximum_native_height_m']
        and max(size[:2]) <= policy['maximum_native_horizontal_size_m'])


def tagged_row(metadata, config, x_bounds=None):
    """Factory for a newly frozen spec; never mutate the supplied source row."""
    policy, native = policy_for(config)
    if not eligible(metadata, policy, native):
        raise ClosePrimaryContractError('Source outside explicit close-primary scope')
    bounds = list(x_bounds if x_bounds is not None else policy['primary_ego_x_bounds_m'])
    _bounds(bounds)
    row = copy.deepcopy(metadata['spec'])
    actor = metadata['runtime_actors'][0]
    row[KEY] = dict(policy=POLICY, primary_name=actor['name'], primary_data_id=actor['data_id'],
        native_unit_reference=copy.deepcopy(policy['native_unit_reference']), primary_ego_x_bounds_m=bounds)
    return row


def _bounds(bounds):
    if (not isinstance(bounds, list) or len(bounds) != 2
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in bounds)
            or not 1. <= bounds[0] < bounds[1] <= 1.5):
        raise ClosePrimaryContractError('Close-primary Ego-X band outside [1.0,1.5]m')


def validate(row, config, asset, known):
    """Only tagged rows enter this path; malformed tags cannot fall back."""
    policy, native = policy_for(config or {})
    tag = row.get(KEY)
    expected_keys = {'policy', 'primary_name', 'primary_data_id',
        'native_unit_reference', 'primary_ego_x_bounds_m'}
    if not isinstance(tag, dict) or set(tag) != expected_keys:
        raise ClosePrimaryContractError('Explicit close-primary row tag required')
    _bounds(tag['primary_ego_x_bounds_m'])
    record = native['assets'].get(asset.get('name'), {})
    pseudo = dict(spec=row, **{unit.KEY: policy['native_unit_reference']}, runtime_actors=[dict(
        name=asset.get('name'), data_id=asset.get('data_id'), source_list=asset.get('source_list'),
        motion='static', geometry_reference_known=known)])
    if (tag['policy'] != POLICY or tag['primary_name'] != asset.get('name')
            or tag['primary_data_id'] != asset.get('data_id')
            or tag['native_unit_reference'] != policy['native_unit_reference']
            or not record or not eligible(pseudo, policy, native)):
        raise ClosePrimaryContractError('Close-primary row/config/physical identity differs')
    return tag


def validate_entry(entry, manifest):
    """Read-only opt-in capture boundary; untagged legacy rows are unchanged."""
    metadata = json.loads(Path(entry['metadata']).read_text())
    row = metadata['spec']
    proof = metadata.get('avoidance_observation', {}).get(KEY)
    if KEY not in row:
        if proof is not None:
            raise ClosePrimaryContractError('Close-primary proof lost its frozen row tag')
        return dict(enabled=False)
    actor = metadata['runtime_actors'][0]
    tag = validate(row, manifest.get('config'), actor, actor.get('geometry_reference_known'))
    if (metadata.get(unit.KEY) != tag['native_unit_reference']
            or not isinstance(proof, dict) or any(proof.get(k) != v for k,v in tag.items())
            or proof.get('placement_envelope_changed') is not False
            or proof.get('native_gt_modified') is not False):
        raise ClosePrimaryContractError('Close-primary metadata/row/native policy differs')
    x,y,gap = (actor.get(k) for k in ('longitudinal_m','lateral_m','planned_surface_clearance_m'))
    lo,hi = tag['primary_ego_x_bounds_m']
    if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (x,y,gap))
            or not lo-1e-6 <= x <= hi+1e-6 or abs(x) >= abs(y)
            or actor.get('actual_ego_role') != row['role']
            or (y>0) != (row['role']=='left') or abs(gap-.30) > .001):
        raise ClosePrimaryContractError('Close-primary prepared band/side/gap differs')
    return dict(enabled=True, primary_ego_xy_m=[x,y], placement_obb_gap_m=gap)


def projection_proposal(metadata, calibration):
    """Conservative whole placement-OBB pinhole check, not a native mask.

    Require all eight proxy vertices ahead of one camera's 0.10m near plane
    and inside its uncropped image with a four-pixel margin. Mesh occlusion,
    writer output, Ego body masking and ground contact still need native QA.
    """
    actor, ego = metadata['runtime_actors'][0], metadata['ego']
    size = np.asarray(actor['estimated_size'], dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or (size <= 0).any():
        raise ClosePrimaryContractError('Invalid close-primary placement proxy')
    ground = actor.get('map_ground_initializer', {})
    if ground.get('usable') is not True:
        return dict(eligible=False, reason='placement_ground_unverified', cameras=[])
    yaw = math.radians(actor['placement_yaw_deg'] - ego['yaw'])
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    center = np.asarray([actor['longitudinal_m'], actor['lateral_m'],
        ground['height_m'] - ego['z'] + size[2] / 2.])
    vertices = np.asarray(list(itertools.product((-1., 1.), repeat=3))) * size / 2.
    vertices = vertices @ rotation.T + center
    rows = []
    for name, camera in sorted(calibration['cameras'].items()):
        optical = (np.c_[vertices, np.ones(8)] @ np.asarray(camera['ego_to_sensor']).T)[:, :3]
        depth = optical[:, 2]
        result = dict(name=name, optical_depth_range_m=[float(depth.min()), float(depth.max())],
            all_vertices_positive_near_plane=bool((depth > .10).all()), proposal_pass=False)
        if (depth > .10).all():
            pixels = optical @ np.asarray(camera['intrinsic']).T
            pixels = pixels[:, :2] / pixels[:, 2:3]
            lo, hi = pixels.min(axis=0), pixels.max(axis=0)
            width, height = camera['resolution_px']
            result.update(projected_proxy_bounds_px=[*lo.tolist(), *hi.tolist()],
                proposal_pass=bool((lo >= [4., 4.]).all() and (hi <= [width - 4., height - 4.]).all()))
        rows.append(result)
    return dict(eligible=any(row['proposal_pass'] for row in rows), cameras=rows,
        proxy='unchanged-conservative-placement-OBB; MGeo-bottom-height estimate',
        native_visibility_certified=False, native_annotation_synthesized=False,
        limitation='No mesh/Ego-body occlusion, mask identity, exact sensor clipping or contact certificate')
