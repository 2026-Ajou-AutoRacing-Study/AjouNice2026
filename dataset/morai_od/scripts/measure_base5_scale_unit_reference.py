#!/usr/bin/env python3
"""Measure scale-1 placement envelopes from two accepted native probe yaws.

This does not rewrite native GT, infer geometry from Object_topic, or approve
any scale profile. XY pivot offsets remain explicitly calibrated-placement
proxies; Z uses the serialized parent pivot above the recorded map initializer.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

import generate_base5_independent as gen
import base5_native_box_corners as native
from base5_asset_scale import WHITELIST
from base5_capture_safety import atomic_json, sha256
from audit_morai_lidar_gt_visibility import parse_instances


SCHEMA = 'base5_scale_unit_geometry_reference_v1'


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def read(path):
    return json.loads(Path(path).read_text())


def vector(value, positive=False):
    result = np.asarray(value, dtype=float)
    require(result.shape == (3,) and np.isfinite(result).all()
            and (not positive or (result > 0).all()), 'Finite three-axis unit geometry required')
    return result


def pin(path, pins):
    path = Path(path).absolute()
    require(path.is_file() and not path.is_symlink(), 'Exact nonsymlink source file required: '+str(path))
    checksum = sha256(path)
    require(str(path) not in pins or pins[str(path)] == checksum, 'Source changed during measurement')
    pins[str(path)] = checksum
    return path


def unit_geometry(deck, manifest, names, pins):
    """Read the same scale-1 calibrated geometry precedence as the probe maker."""
    pin(manifest, pins)
    inventory = read(gen.catalog.DEFAULT_CATALOG)
    files = {gen.catalog.DEFAULT_CATALOG, gen.GEOMETRY, gen.APPEARANCE_POLICY, gen.HIGHWAY_SAMPLE,
             gen.geo.DEFAULT_TEMPLATE, gen.geo.DEFAULT_ROUTE}
    if gen.LIVE_GEOMETRY.exists():
        files.add(gen.LIVE_GEOMETRY)
    files.update(gen.geo.DEFAULT_MGEO.glob('*.json'))
    files.update(gen.geo.DEFAULT_RUNTIME_ASSET_DIR/name for name in inventory['source_sha256'])
    files.update(gen.geo.PROJECT_ROOT/asset['geometric_gt_reference'] for asset in inventory['assets']
                 if asset.get('geometric_gt_reference'))
    reference = deck['config'].get('static_placement_geometry')
    if reference:
        require(sha256(reference['path']) == reference['sha256'], 'Scale-1 placement reference changed')
        files.add(Path(reference['path']))
    for path in files:
        pin(path, pins)
    ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
    from base5_placement_geometry import configure
    ctx = configure(ctx, deck['config'], dict(family='obstacle', mission=False, motion='static'))
    result = {}
    for name in names:
        matches = [asset for asset in ctx['pools']['obstacle'] if asset['name'] == name]
        require(len(matches) == 1 and matches[0]['data_id'] == WHITELIST[name][0], 'Unit source asset identity differs')
        size, offset, known = gen.geometry(matches[0], ctx)
        require(known, 'Unit placement source is not calibrated')
        result[name] = dict(bbox_size_m=vector(size, True).tolist(),
                            pivot_to_bbox_center_local_m=vector(offset).tolist())
    return result


def observation(row, metadata, root, pins):
    require(row.get('status') == 'accepted_candidate', 'Original control must be accepted, not review/failed')
    scenario = read(pin(root/'scenario.json', pins))
    require(sha256(root/'scenario.json') == row.get('scenario_sha256'), 'Control scenario checksum differs from ledger')
    audit = read(pin(root/'audit.json', pins))
    require(audit == row.get('audit') and audit.get('accepted_candidate') is True
            and not audit.get('review_reasons'), 'Saved control audit differs or was not accepted')
    collision = audit.get('capture_safety', {}).get('collision', {})
    require(collision.get('observed') is True and collision.get('no_collision_verified') is True,
            'Control collision safety evidence missing')
    probe = metadata['scale_probe']
    uid, name = probe['parent_uid'], probe['asset']
    actors = [actor for actor in metadata['runtime_actors'] if actor.get('unique_id') == uid]
    payloads = [actor for actor in scenario.get('objectList', []) if actor.get('UNIQUEID') == uid]
    require(len(actors) == len(payloads) == 1, 'Exact unit parent identity required')
    actor, payload = actors[0], payloads[0]
    scale = payload.get('scale', {})
    require(probe.get('profile') == 'original' and probe.get('condition') == 'control'
            and probe.get('requested_scale') == [1., 1., 1.]
            and [scale.get(key) for key in 'xyz'] == [1., 1., 1.], 'Only exact original scale-1 controls may calibrate')
    require(actor.get('name') == name and actor.get('data_id') == WHITELIST[name][0]
            and payload.get('DataID') == WHITELIST[name][0] and actor.get('source_list') == 'objectList'
            and actor.get('motion') == 'static' and not actor.get('motion_path'), 'Unit control asset/motion differs')
    yaw = float(probe['requested_yaw_deg']) % 360.
    require(yaw in (0., 90.) and abs(float(actor['placement_yaw_deg'])-yaw) < 1.e-6
            and abs(float(payload['rot']['yaw'])-yaw) < 1.e-6, 'Exact original yaw 0/90 controls required')
    pos = actor['position']
    require(all(math.isfinite(pos[key]) for key in 'xyz'), 'Nonfinite serialized actor pivot')
    require(all(abs(float(payload['pos'][key])-pos[key]) <= .001 for key in 'xyz'),
            'Serialized actor pivot differs from scenario')
    ground = actor.get('map_ground_initializer', {})
    require(ground.get('usable') is True and math.isfinite(ground.get('height_m', float('nan'))),
            'Usable recorded ground initializer required')
    annotations = list(root.glob('LIDAR_*/*_Instance.txt'))
    require(len(annotations) == 1, 'One exact native annotation timestamp required')
    records = native.load_matching_corners(annotations[0])
    matches = [(box, records[(uid, 'Obstacle', 0)]) for box in parse_instances(annotations[0])
               if box['unique_id'] == uid and box['raw_class'] == 'Obstacle']
    require(len(matches) == 1, 'Unit native parent must be one Obstacle component')
    derived = native.derive_geometry(*matches[0], allow_dimension_change=False)
    size = vector(derived['evidence']['corner_extent_local_m'], True)
    components = [component for component in audit.get('components', []) if component.get('parent_uid') == uid]
    require(len(components) == 1 and components[0].get('raw_class') == 'Obstacle'
            and components[0].get('asset') == name
            and np.allclose(components[0]['size_m'], size, atol=.001, rtol=.001),
            'Accepted audit geometry differs from actual unit native corners')
    above = float(pos['z']-ground['height_m'])
    return dict(id=row['id'], capture_root=str(root), requested_yaw_deg=yaw, bbox_size_m=size.tolist(),
        pivot_above_recorded_ground_m=above, pivot_to_bbox_center_local_z_m=float(size[2]/2-above),
        native_corner_evidence=derived['evidence'])


def measure(manifest, ledger, assets, output):
    manifest, ledger, output = Path(manifest).resolve(), Path(ledger).resolve(), Path(output).resolve()
    require(not output.exists(), 'New unit geometry reference output required')
    require(assets and len(set(assets)) == len(assets) and all(name in WHITELIST for name in assets),
            'Unique whitelisted physical obstacle assets required')
    pins = {}
    pin(manifest, pins)
    pin(ledger, pins)
    status = read(pin(ledger.parent/'status.json', pins))
    shutdown = read(pin(ledger.parent/'shutdown_safety.json', pins))
    rows = [json.loads(line) for line in ledger.read_text().splitlines() if line.strip()]
    require(status.get('stage') == 'complete' and status.get('error') is None
            and status.get('finished') == len(rows) and len({row['id'] for row in rows}) == len(rows)
            and shutdown.get('park_verified') is True, 'Completed unique probe ledger and verified shutdown required')
    observed = {name: {} for name in assets}
    for row in rows:
        root = Path(row['capture_root']).resolve()
        require(not (root == output or root in output.parents), 'Output cannot modify capture input')
        metadata = read(pin(root/'metadata.json', pins))
        probe = metadata.get('scale_probe', {})
        name = probe.get('asset')
        if name not in observed or probe.get('profile') != 'original':
            continue
        for path in root.rglob('*'):
            require(not path.is_symlink(), 'Raw native source symlinks are not immutable evidence')
            if path.is_file():
                pin(path, pins)
        item = observation(row, metadata, root, pins)
        yaw = item['requested_yaw_deg']
        require(yaw not in observed[name], 'Duplicate original asset/yaw cannot count as independent evidence')
        observed[name][yaw] = item
    require(all(set(values) == {0., 90.} for values in observed.values()), 'Two accepted original yaws 0 and 90 required for every asset')
    old = unit_geometry(read(manifest), manifest, assets, pins)
    helpers = (Path(__file__).resolve(), Path(native.__file__).resolve(), Path(gen.__file__).resolve(),
               Path(sys_modules_file(parse_instances)), Path(gen.geo.__file__).resolve())
    for path in helpers:
        pin(path, pins)
    result = {}
    for name in assets:
        observations = [observed[name][yaw] for yaw in (0., 90.)]
        sizes = np.asarray([item['bbox_size_m'] for item in observations])
        measured = np.median(sizes, axis=0)
        require(np.all(np.ptp(sizes, axis=0) <= np.maximum(.005, measured*.02)),
                'Original yaw geometry is inconsistent; do not hide it with a median')
        old_size, old_offset = vector(old[name]['bbox_size_m'], True), vector(old[name]['pivot_to_bbox_center_local_m'])
        offset = old_offset*measured/old_size
        offset[2] = np.median([item['pivot_to_bbox_center_local_z_m'] for item in observations])
        result[name] = dict(bbox_size_m=measured.tolist(), pivot_to_bbox_center_local_m=offset.tolist(),
            observations=2, stationary_only=True, scale_xyz=[1., 1., 1.], data_id=WHITELIST[name][0],
            source_pins=dict(pins), measurements=observations,
            method=dict(size='q50 of verified native Instance_8Points local extents at original yaw0/yaw90',
                xy='source calibrated XY pivot offset * measured unit size / source unit size; placement proxy only',
                z='q50(native H/2 - (serialized actor.position.z - recorded map_ground_initializer.height_m))',
                source_geometry=old[name], raw_gt_modified=False, object_topic_used=False,
                world_pivot_transform_used=False, contact_certified=False, automatic_profile_approval=False))
    for path, checksum in pins.items():
        require(sha256(path) == checksum, 'Source changed during measurement: '+path)
    document = dict(schema=SCHEMA, training_gt_modified=False, native_gt_source='native eight corners remain authoritative',
        placement_only=True, production_approved=False, assets=result, source_pins=pins,
        source_manifest=str(manifest), probe_ledger=str(ledger), raw_preserved=True,
        limitation='Two yaw observations calibrate placement proxies only; no mesh contact, profile naturalness, or AP approval.')
    atomic_json(output, document)
    return output


def sys_modules_file(function):
    import sys
    return sys.modules[function.__module__].__file__


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--ledger', type=Path, required=True)
    parser.add_argument('--assets', nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(measure(args.manifest, args.ledger, args.assets, args.output))
