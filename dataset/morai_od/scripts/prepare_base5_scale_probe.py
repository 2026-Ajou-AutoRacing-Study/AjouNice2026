#!/usr/bin/env python3
"""Prepare bounded native scale diagnostics, never a production scene deck.

Every pair uses the same safe parked Ego and parent XY/yaw. Only the scale and
declared provisional pivot-height initializer differ. Geometry is a hypothesis
until the native sensor annotations and images have been reviewed.
"""
import argparse
import copy
import json
import math
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_asset_scale import WHITELIST
from base5_map_surface import MapSurface, configure, ego_road_support, initial_pose
from base5_reference_frames import references
from base5_static_height import saved_height_candidate
from run_base5_failure_aware_replenishment_fast import prewarm


def validate_inputs(assets, axis, factors, yaws, distance_m=18.):
    if (type(axis) is not bool or not isinstance(assets, (list, tuple)) or not assets
            or any(not isinstance(name, str) or name not in WHITELIST for name in assets)
            or len(set(assets)) != len(assets)):
        raise ValueError('Unique whitelisted physical obstacle asset names required')
    if axis and list(assets) != ['CargoBox']:
        raise ValueError('Axis isolation is restricted to CargoBox')
    for values, label in ((factors, 'factors'), (yaws, 'yaws')):
        if (not isinstance(values, (list, tuple)) or not values
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in values)):
            raise ValueError('Finite numeric nonempty ' + label + ' required')
    if any(not 1. < factor <= 3. for factor in factors) or len(set(factors)) != len(factors):
        raise ValueError('Unique enlargement factors in (1,3] required')
    if len({round(factor * 100) for factor in factors}) != len(factors):
        raise ValueError('Distinct percentage-profile identifiers required')
    normalized_yaws = [float(yaw) % 360. for yaw in yaws]
    if len(set(normalized_yaws)) != len(normalized_yaws):
        raise ValueError('Duplicate yaw orientations are not independent diagnostics')
    count = len(assets) * len(yaws) * (6 if axis else 1 + len(factors))
    if not 1 <= count <= 64:
        raise ValueError('Native scale diagnostic must contain 1 to 64 scenes')
    if (isinstance(distance_m, bool) or not isinstance(distance_m, (int, float))
            or not math.isfinite(distance_m) or not 6. <= distance_m <= 25.):
        raise ValueError('Probe pivot distance must be finite and within 6 to 25 metres')
    return count


def load_unit_reference(path):
    if path is None:
        return {}, None
    path = Path(path).resolve(strict=True)
    value = json.loads(path.read_text())
    if not isinstance(value.get('assets'), dict) or not value['assets']:
        raise ValueError('Explicit nonempty unit-reference assets required')
    for name, record in value['assets'].items():
        if name not in WHITELIST or not isinstance(record, dict):
            raise ValueError('Unit reference contains a non-whitelisted physical obstacle')
        for key in ('bbox_size_m', 'pivot_to_bbox_center_local_m'):
            vector = record.get(key)
            if (not isinstance(vector, (list, tuple)) or len(vector) != 3
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) or (key == 'bbox_size_m' and v <= 0) for v in vector)):
                raise ValueError('Finite three-axis unit reference size/pivot required')
        if (record.get('data_id', WHITELIST[name][0]) != WHITELIST[name][0]
                or record.get('source_list', 'objectList') != 'objectList'):
            raise ValueError('Unit reference name/DataID/container differ')
    return value['assets'], dict(path=str(path), sha256=sha256(path), diagnostic_only=True,
                                native_qa_waived=False, production_qualification=False)


def prepare(source, output, assets, *, axis=False, factors=(1.2, 1.5), yaws=(0., 90.), distance_m=18.,
            unit_reference=None):
    expected_count = validate_inputs(assets, axis, factors, yaws, distance_m)
    unit_geometry, unit_pin = load_unit_reference(unit_reference)
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Use a fresh diagnostic output')
    deck = json.loads(source.read_text())
    original = next(e for e in deck['scenes'] if e['id'] == 'sunny13_sunny11_b5v2_03942')
    if sha256(original['scenario']) != original['scenario_sha256']:
        raise ValueError('Source scene changed')
    old_scene = json.loads(Path(original['scenario']).read_text())
    old_meta = json.loads(Path(original['metadata']).read_text())
    source_paths = [source, Path(original['scenario']), Path(original['metadata'])]
    if unit_pin is not None:
        source_paths.append(Path(unit_pin['path']))
    source_before = {str(path.resolve()): sha256(path) for path in source_paths}
    if any(output == path.resolve().parent or path.resolve().parent in output.parents
           for path in source_paths[1:3]):
        raise ValueError('Probe output cannot be inside the preserved source scene directory')
    ctx = prewarm(gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO), deck['config'])
    from base5_placement_geometry import configure as configure_static_geometry
    ctx = configure_static_geometry(ctx, deck['config'], old_meta['spec'])
    ctx = configure(ctx, deck['config'].get('placement_ground'))
    ctx.setdefault('map_surface_index', MapSurface(ctx['links'], ctx.get('lane_boundaries')))
    ego = dict(old_meta['ego'], yaw=old_meta['ego']['yaw'] + 180.)
    support = ego_road_support(ctx['map_surface_index'], ego)
    if not support['road_supported']:
        raise ValueError('Probe parked Ego footprint is not road supported')
    ego['z'] = support['height_m']
    original_point = old_meta['runtime_actors'][0]['position']
    dx, dy = original_point['x'] - ego['x'], original_point['y'] - ego['y']
    original_distance = math.hypot(dx, dy)
    if original_distance < .1:
        raise ValueError('Source actor has no usable direction from the Ego pivot')
    point = dict(original_point, x=ego['x'] + dx / original_distance * distance_m,
                 y=ego['y'] + dy / original_distance * distance_m)
    config = dict(capture=copy.deepcopy(deck['config']['capture']),
                  placement_ground=copy.deepcopy(deck['config']['placement_ground']),
                  dataset_id='scale_native_diagnostic_261016')
    # These probes do not claim the original family's distribution/range quota.
    # Existing native safety, motion, visibility and ground gates are unchanged.
    config['capture'].pop('static_ground_reference', None)
    config['capture']['asset_scale_probe'] = True
    selected = {}
    for name in assets:
        matches = [a for a in ctx['pools']['obstacle'] if a['name'] == name]
        if (len(matches) != 1 or matches[0]['data_id'] != WHITELIST[name][0]
                or matches[0].get('source_list') != 'objectList'
                or matches[0].get('semantic_class') != 'obstacle'):
            raise ValueError('Probe asset catalog identity/class/container differs')
        selected[name] = matches[0]
    entries = []
    for name in assets:
        asset = selected[name]
        base_size, base_off, known = gen.geometry(asset, ctx)
        unit_proof = None
        if name in unit_geometry:
            reference = unit_geometry[name]
            before_size, before_offset = list(base_size), list(base_off)
            base_size = list(reference['bbox_size_m'])
            base_off = list(reference['pivot_to_bbox_center_local_m'])
            saved = saved_height_candidate(asset, ctx)
            saved_proxy = float(saved['pivot_height_candidate_m']) if saved['usable'] else 0.
            base_off[2] = base_size[2] / 2 - saved_proxy
            known = True
            unit_proof = dict(reference=unit_pin, original_size_m=before_size,
                original_pivot_proxy_m=before_offset, supplied_unit_geometry=reference,
                applied_unit_size_m=base_size, applied_unit_pivot_proxy_m=base_off,
                saved_height_initializer=saved, saved_pivot_height_proxy_m=saved_proxy,
                pose_height_policy_preserved=True, production_qualification=False)
        profiles = [('original', [1., 1., 1.])]
        if axis:
            if name != 'CargoBox':
                raise ValueError('Axis isolation is restricted to CargoBox')
            profiles += [('axis_x', [1.5, 1., 1.]), ('axis_y', [1., 1.5, 1.]),
                         ('axis_z', [1., 1., 1.5]), ('iso150', [1.5]*3),
                         ('sample_2_3_2', [2., 3., 2.])]
        elif WHITELIST[name][1] == 'barrier':
            # Native long horizontal axis is Y in the current scale-1
            # reference. Raw native corner probes support Unity X for that
            # axis; each asset/profile still requires its own qualification.
            profiles += [('length'+str(round(f*100)), [f, 1., 1.]) for f in factors]
        else:
            profiles += [('iso'+str(round(f*100)), [f]*3) for f in factors]
        for yaw in yaws:
            yaw_token = format(float(yaw), '.12g').replace('-', 'm').replace('.', 'p')
            for profile, scale in profiles:
                # Native L/W/H <- Unity Z/X/Y. This is a declared probe
                # hypothesis, not an all-asset qualified placement policy.
                bbox_factors = [scale[2], scale[0], scale[1]]
                size = [s*f for s, f in zip(base_size, bbox_factors)]
                off = [s*f for s, f in zip(base_off, bbox_factors)]
                pose = dict(point, yaw=yaw, link_idx=old_meta['runtime_actors'][0]['map_ground_initializer']['preferred_link'])
                pose, height, ground = initial_pose(pose, asset, size, off, ctx)
                # Scale the native-envelope bottom relative to the pivot, not
                # the saved scale1 cache entry. Confirm/reject in native images.
                delta = (base_off[2]-base_size[2]/2)*(1.-bbox_factors[2])
                pose['z'] += delta
                scene = copy.deepcopy(old_scene)
                gen.geo.clear_actor_lists(scene)
                gen.geo.set_ego_pose(scene['egoVehicle'], ego, initial_speed_kph=0.)
                from prepare_base5_intersection_manual_ground import park
                park(scene['egoVehicle'])
                actor = copy.deepcopy(asset['template'])
                gen.set_pose(actor, pose, 8500, asset, scene, 'static')
                actor['scale'] = gen.geo.formatted_vector(*scale)
                theta = math.radians(yaw)
                center = [pose['x']+math.cos(theta)*off[0]-math.sin(theta)*off[1],
                          pose['y']+math.sin(theta)*off[0]+math.cos(theta)*off[1]]
                ego_shape = gen.rectangle([ego['x'] + 1.35 * math.cos(math.radians(ego['yaw'])),
                                           ego['y'] + 1.35 * math.sin(math.radians(ego['yaw']))],
                                          [4.8, 2.], ego['yaw'])
                actor_shape = gen.rectangle(center, size, yaw)
                surface_gap = actor_shape.distance(ego_shape)
                if surface_gap < .3:
                    raise ValueError('Probe enlarged placement envelope is too close to stationary Ego')
                frame = references(center, ego, old_meta['base']['yaw'])
                distance = math.hypot(center[0]-ego['x'], center[1]-ego['y'])
                meta = dict(spec=dict(family='obstacle', mission=False, motion='static',
                           seed=261016, role='front', distance_bounds=[0., 40.],
                           actors=1, index=len(entries), subfamily=None),
                    ego=ego, base=old_meta['base'], ego_ground_initializer=support,
                    split=old_meta['split'], split_group=old_meta['split_group'],
                    runtime_actors=[dict(unique_id=8500, data_id=asset['data_id'], name=name,
                        source_list='objectList', motion='static', requested_motion='static',
                        position={k:pose[k] for k in ('x','y','z')}, placement_yaw_deg=yaw,
                        estimated_size=size, geometry_reference_known=known,
                        geometric_center_estimate=center, planned_center_distance_m=distance,
                        planned_surface_clearance_m=surface_gap,
                        training_class='component_annotation', reference_frames=frame,
                        longitudinal_m=frame['ego_longitudinal_m'], lateral_m=frame['ego_lateral_m'],
                        actual_ego_role='front', static_height_initializer=height,
                        map_ground_initializer=ground, runtime_mapping_verified=True)],
                    ground_support_check=dict(enabled=True, max_range_m=20., maximum_gap_m=.15,
                        required_uids=[8500], policy='all-actors-observed-ground-v1',
                        static_policy=config['capture'].get('static_ground_policy')),
                    scale_probe=dict(pair_key=name+'_yaw'+yaw_token,
                        condition='control' if profile=='original' else 'scaled',
                        asset=name, parent_uid=8500, requested_scale=scale,
                        requested_yaw_deg=yaw, profile=profile,
                        requested_pivot_distance_m=distance_m,
                        original_source_pivot=original_point, probe_pivot_xy_m=[point['x'], point['y']],
                        geometry_axis_hypothesis=[2,0,1], pivot_height_delta_m=delta,
                        native_qualified=False),
                    diagnostic_provenance=dict(source_manifest=str(source), source_sha256=sha256(source),
                        baseline_id=original['id'], source_scene_sha256=original['scenario_sha256'],
                        source_metadata_sha256=source_before[str(Path(original['metadata']).resolve())],
                        source_files=source_before,
                        preparation_script_sha256=sha256(Path(__file__)),
                        ego_reversed_at_same_supported_pivot=True, production_quota_claim=False))
                if unit_proof is not None:
                    meta['runtime_actors'][0]['probe_unit_geometry_override'] = copy.deepcopy(unit_proof)
                    meta['scale_probe']['unit_reference'] = copy.deepcopy(unit_pin)
                ident = f'scaleprobe_{len(entries):03d}_{name}_y{yaw_token}_{profile}'
                sp, mp = output/'scenarios'/(ident+'.json'), output/'scenarios'/(ident+'_metadata.json')
                atomic_json(sp, scene); atomic_json(mp, meta)
                entries.append(dict(id=ident, scenario=str(sp), metadata=str(mp), scenario_sha256=sha256(sp),
                    family='obstacle', role='front', distance_bounds=[0.,40.], subfamily=None,
                    motion='static', mission=False, split=old_meta['split']))
    if len(entries) != expected_count:
        raise RuntimeError('Probe scene count differs from the validated bounded plan')
    source_after = {str(path.resolve()): sha256(path) for path in source_paths}
    if source_before != source_after:
        raise RuntimeError('Source changed during read-only diagnostic preparation')
    manifest = dict(schema_version=2, dataset_id=config['dataset_id'], config=config,
                    scenes=entries, count=len(entries), status='DIAGNOSTIC_ONLY; no bulk approval')
    manifest['scale_probe_registration'] = dict(source_files=source_before, source_unchanged=True,
        preparation_script_sha256=sha256(Path(__file__)), assets=list(assets), axis_isolation=axis,
        requested_factors=list(factors), requested_yaws=list(yaws), maximum_scenes=64,
        requested_pivot_distance_m=distance_m,
        unit_reference=unit_pin,
        production_quota_claim=False, native_geometry_qualified=False)
    atomic_json(output/'manifest.json', manifest)
    return output/'manifest.json'


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--assets', nargs='+', required=True)
    p.add_argument('--axis', action='store_true')
    p.add_argument('--factors', type=float, nargs='+', default=[1.2,1.5])
    p.add_argument('--yaws', type=float, nargs='+', default=[0.,90.])
    p.add_argument('--distance-m', type=float, default=18., help='Stationary Ego pivot to parent pivot, 6 to 25 metres')
    p.add_argument('--unit-reference', type=Path, help='Pinned diagnostic unit geometry, never a QA waiver')
    args = p.parse_args()
    print(prepare(args.manifest, args.output, args.assets, axis=args.axis,
                  factors=args.factors, yaws=args.yaws, distance_m=args.distance_m,
                  unit_reference=args.unit_reference))
