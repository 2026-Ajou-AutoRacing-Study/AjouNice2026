#!/usr/bin/env python3
"""CPU-only full selected-ledger size audit; writes new derived reports only.

Keep four contracts separate: scale-1 local unit dimensions, conservative
placement envelopes, pivot/height initializers, and per-capture native GT.
Eight-corner dimension-change permission is analysis-only, not GT adoption.
No ROS/GPU imports, simulator calls, profile approval, or source modifications.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

import base5_native_box_corners as corners
from base5_capture_safety import atomic_json
from base5_upright_boxes import component_bbox


REPO = Path(__file__).resolve().parents[3]
CONFIG = REPO / 'dataset/morai_od/configs'
EVIDENCE = REPO / 'artifacts/evaluation/base5_mission_scale_261016'
LEDGER = Path.home() / ('morai_cap/base5_v2_261008/final_appearance_5k_v132_release/'
                        'collection_5k_sunny13_flexible_v140/selected_ledger.jsonl')
SCHEMA = 'base5_geometry_reference_audit_v1'


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def vector(value, positive=False):
    require(isinstance(value, (list, tuple)) and len(value) == 3
            and not any(isinstance(x, bool) for x in value), 'Numeric three-axis vector required')
    result = np.asarray(value, dtype=float)
    require(np.isfinite(result).all() and (not positive or (result > 0).all()),
            'Finite dimensions and positive sizes required')
    return result


class Sources:
    """Hash every consumed input and prove it unchanged before writing output."""
    def __init__(self):
        self.pins = {}

    def data(self, path):
        path = Path(path).absolute()
        require(path.is_file() and not path.is_symlink(), 'Exact nonsymlink input required: ' + str(path))
        before = path.stat()
        content = path.read_bytes()
        after = path.stat()
        stamp = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
        require(stamp(before) == stamp(after), 'Input changed while reading: ' + str(path))
        pin = dict(sha256=hashlib.sha256(content).hexdigest(), bytes=len(content))
        require(str(path) not in self.pins or self.pins[str(path)] == pin,
                'Input changed during audit: ' + str(path))
        self.pins[str(path)] = pin
        return content

    def json(self, path):
        return json.loads(self.data(path))

    def verify(self):
        for path in list(self.pins):
            self.data(path)


def unique(rows, key, label):
    result = {}
    for row in rows:
        value = key(row)
        require(value not in result, 'Duplicate ' + label + ': ' + str(value))
        result[value] = row
    return result


def classify(actor, payload, parent_components):
    """Classification never turns a nonunit/compound observation into a unit."""
    scale = vector([payload['scale'][axis] for axis in 'xyz'], positive=True)
    if parent_components != 1:
        return 'compound_parent'
    if not np.array_equal(scale, [1., 1., 1.]):
        return 'nonunit_single'
    if actor.get('motion') not in ('static', 'standing') or actor.get('motion_path'):
        return 'moving_unit_single'
    return 'static_unit_single'


def distribution(values):
    if not values:
        return None
    rows = np.asarray(values, dtype=float)
    return dict(n=len(rows), minimum_m=rows.min(0).tolist(),
                q10_m=np.percentile(rows, 10, axis=0).tolist(), q50_m=np.median(rows, axis=0).tolist(),
                q90_m=np.percentile(rows, 90, axis=0).tolist(), maximum_m=rows.max(0).tolist())


def size_modes(rows, field='corner_local_size_m'):
    """Keep distinct native size states separate, independent of tilt/yaw.

    2 mm / 0.5 percent is a diagnostic clustering tolerance, not a QA waiver.
    Compare to every member so a gradual chain cannot hide a broad variation.
    """
    groups = []
    for row in sorted(rows, key=lambda item: tuple(item[field])):
        size = vector(row[field], positive=True)
        match = next((group for group in groups if all(np.all(
            np.abs(size - vector(item[field])) <= np.maximum(.002, vector(item[field]) * .005))
            for item in group)), None)
        if match is None:
            groups.append([row])
        else:
            match.append(row)
    return [dict(mode=index, distribution=distribution([item[field] for item in group]),
                 observations=[dict(scene_id=item['scene_id'], parent_uid=item['parent_uid'])
                               for item in group]) for index, group in enumerate(groups)]


def placement_references(catalog, source_manifest, sources):
    """Mirror generator precedence without loading generator, ROS, map, or templates."""
    rows, origins = {}, {}
    paths = [CONFIG / 'morai_asset_bbox_offsets_base3_v1.json']
    paths += [REPO / item['geometric_gt_reference'] for item in catalog['assets']
              if item.get('geometric_gt_reference')]
    paths.append(CONFIG / 'base5_stationary_geometry_mr_261006.json')
    for path in dict.fromkeys(paths):
        for name, item in sources.json(path)['assets'].items():
            rows[name], origins[name] = item, str(path.absolute())
    reference = source_manifest['config'].get('static_placement_geometry')
    if reference:
        document = sources.json(reference['path'])
        require(sources.pins[str(Path(reference['path']).absolute())]['sha256'] == reference['sha256'],
                'Frozen conservative placement reference changed')
        for name, item in document['assets'].items():
            if name not in rows:
                rows[name], origins[name] = item, str(Path(reference['path']).absolute())
    return rows, origins


def native_observation(component, actor, payload, root, records, kind):
    base = dict(scene_id=root.name, parent_uid=int(component['parent_uid']),
                component_id=component['component_id'], asset=component['asset'],
                data_id=int(component['data_id']), classification=kind,
                scenario_scale_xyz=[payload['scale'][axis] for axis in 'xyz'],
                native_rotation_rpy=component['rotation_rpy'],
                placement_yaw_deg=actor.get('placement_yaw_deg'),
                native_summary_local_size_m=vector(component['size_m'], True).tolist(),
                upright_yaw_removed_size_m=component_bbox(component)[3:6],
                lidar_points=component['lidar_points'],
                audit_positive_lidar1_camera0=component['positive_lidar1_camera0'],
                capture_root=str(root), estimated_placement_size_m=actor.get('estimated_size'),
                placement_pivot_proxy_m=actor.get('pivot_to_bbox_center_local_m'),
                static_height_initializer=actor.get('static_height_initializer'),
                map_ground_initializer=actor.get('map_ground_initializer'))
    if kind == 'compound_parent':
        base['corner_analysis'] = 'not a single-component Obstacle; no forced parent merge'
        return base
    parts = str(component['component_id']).split(':')
    require(len(parts) == 3, 'Explicit native component ordinal required')
    key = (int(component['parent_uid']), component['raw_class'], int(parts[2]))
    require(key in records, 'Native component absent from matching timestamp companion')
    record = records[key]
    require(np.allclose(component['size_m'], record['annotation_size_m'], rtol=1e-6, atol=1e-6),
            'Saved audit/native summary size differs')
    derived = corners.derive_geometry(component, record, allow_dimension_change=True)
    proof = derived['evidence']
    base.update(corner_local_size_m=proof['corner_extent_local_m'],
                summary_corner_dimensions_differ=proof['dimensions_changed'],
                center_residual_m=proof['center_residual_m'],
                maximum_corner_residual_m=proof['maximum_corner_residual_m'],
                source_files=proof['source_files'], analysis_only=True,
                gt_adoption=False, scenario_scale_multiplied=False)
    return base


def candidate_reference(name, asset, rows, envelope, known_variation=False):
    units = [row for row in rows if row['classification'] == 'static_unit_single'
             and 'corner_local_size_m' in row]
    modes = size_modes(units)
    reasons = []
    if len(units) < 2:
        reasons.append('fewer_than_two_unit_observations')
    if len(modes) != 1:
        reasons.append('multiple_or_missing_native_size_states')
    if any(row['summary_corner_dimensions_differ'] for row in units):
        reasons.append('unit_summary_and_corners_differ')
    if known_variation:
        reasons.append('independent_native_size_variation_evidence')
    if not envelope:
        reasons.append('missing_existing_pivot_proxy')
    if reasons:
        return None, reasons
    size = vector(modes[0]['distribution']['q50_m'], positive=True)
    # Only re-express the old proxy around the measured box. This is not a
    # measured pivot/height or a reduction of the conservative envelope.
    offset = vector(envelope['pivot_to_bbox_center_local_m']) * size / vector(envelope['bbox_size_m'], True)
    return dict(data_id=asset['data_id'], source_list='objectList', scale_xyz=[1., 1., 1.],
                bbox_size_m=size.tolist(), pivot_to_bbox_center_local_m=offset.tolist(),
                observations=len(units), stationary_only=True, diagnostic_only=True,
                production_approved=False, native_qa_waived=False,
                axis_mapping_qualified=False,
                pivot_source='existing placement proxy scaled by native unit/local-envelope ratio; unmeasured',
                height_contact_certified=False, conservative_placement_envelope_modified=False), []


def independent_variation_observations(document, sources):
    """Recompute reported RedBarrel states from their hashed native files."""
    result, cache = [], {}
    for item in document['observations']:
        root = Path(item['capture_root']).absolute()
        if root not in cache:
            metadata = sources.json(root / 'metadata.json')
            scenario = sources.json(root / 'scenario.json')
            annotations = list(root.glob('LIDAR_*/*_Instance.txt'))
            require(len(annotations) == 1, 'One independent RedBarrel native timestamp required')
            cache[root] = (metadata, scenario, corners.load_matching_corners(annotations[0]))
        metadata, scenario, records = cache[root]
        uid = int(item['parent_uid'])
        actors = [actor for actor in metadata['runtime_actors'] if int(actor['unique_id']) == uid]
        payloads = [payload for payload in scenario['objectList'] if int(payload['UNIQUEID']) == uid]
        require(len(actors) == len(payloads) == 1 and actors[0]['name'] == 'RedBarrel'
                and int(payloads[0]['DataID']) == 40100003, 'Independent RedBarrel identity differs')
        require(item['scenario_scale_xyz'] == [payloads[0]['scale'][axis] for axis in 'xyz'],
                'Independent RedBarrel scale differs')
        record = records[(uid, 'Obstacle', 0)]
        annotation = dict(parent_uid=uid, raw_class='Obstacle',
            center_lidar_m=record['annotation_center_lidar_m'],
            rotation_rpy=record['annotation_rotation_rpy'], size_m=record['annotation_size_m'])
        proof = corners.derive_geometry(annotation, record, allow_dimension_change=True)['evidence']
        require(np.allclose(proof['corner_extent_local_m'], item['native_eight_corner_local_extent_m'],
                            rtol=0, atol=1e-6), 'Independent RedBarrel report/native corners differ')
        result.append(dict(scene_id=item['case'] + ':' + item['stage'], parent_uid=uid,
            classification=classify(actors[0], payloads[0], record['parent_component_count']),
            scenario_scale_xyz=item['scenario_scale_xyz'],
            corner_local_size_m=proof['corner_extent_local_m'],
            native_summary_local_size_m=record['annotation_size_m'],
            source_files=proof['source_files'], analysis_only=True, gt_adoption=False))
    return result


def audit(ledger, output, *, expected_frames=5000, catalog=CONFIG / 'morai_asset_catalog_mr_base5_261005.json',
          historical=REPO / 'artifacts/evaluation/base5_obstacle_range_261015/range_size_reference.json',
          source_manifest=EVIDENCE / 'registered_scale_v155m/manifest.json',
          unit_references=None, variation=EVIDENCE / 'redbarrel_native_variation_v155l.json',
          qualified=EVIDENCE / 'qualified_scale_7assets_v155m.json'):
    output = Path(output).absolute()
    require(not output.exists(), 'New audit output directory required; no historical overwrite')
    sources = Sources()
    for helper in (__file__, corners.__file__, REPO / 'dataset/morai_od/scripts/base5_upright_boxes.py'):
        sources.data(helper)
    inventory = sources.json(catalog)
    assets = {int(item['data_id']): item for item in inventory['assets']
              if item.get('semantic_class') == 'obstacle' and item.get('collection_role') == 'positive'}
    require(len(assets) == 19, 'Explicit 19 physical obstacle catalog scope required')
    old = sources.json(historical)
    frozen = sources.json(source_manifest)
    envelopes, envelope_paths = placement_references(inventory, frozen, sources)
    units, unit_paths = {}, {}
    for path in unit_references or (EVIDENCE / 'unit_geometry_base4_v155.json',
                                    EVIDENCE / 'unit_geometry_reference_v155e2.json'):
        for name, item in sources.json(path)['assets'].items():
            units[name], unit_paths[name] = item, str(Path(path).absolute())
    policy = sources.json(qualified)
    known_variation = sources.json(variation)
    require(known_variation.get('passed') is True and known_variation['asset']['name'] == 'RedBarrel',
            'Explicit validated RedBarrel variation evidence required')
    for path, checksum in known_variation['source_files'].items():
        sources.data(path)
        require(sources.pins[str(Path(path).absolute())]['sha256'] == checksum,
                'Independent RedBarrel source changed')
    independent_red = independent_variation_observations(known_variation, sources)
    selected = unique((json.loads(line) for line in sources.data(ledger).splitlines() if line.strip()),
                      lambda item: item['id'], 'selected scene ID')
    require(len(selected) == expected_frames, 'Selected ledger expected frame count differs')
    rows, global_counts, split_counts, sensors = [], Counter({kind: 0 for kind in
        ('static_unit_single', 'nonunit_single', 'moving_unit_single', 'compound_parent')}), Counter(), Counter()
    seen_roots, observed_frames = set(), set()
    for scene_id, entry in sorted(selected.items()):
        require(entry['status'] == 'accepted_candidate', 'Canonical selected ledger must contain accepted captures only')
        root = Path(entry['capture_root']).absolute()
        require(root not in seen_roots, 'Duplicate selected capture root')
        seen_roots.add(root)
        meta = sources.json(root / 'metadata.json')
        saved = sources.json(root / 'audit.json')
        scenario = sources.json(root / 'scenario.json')
        require(sources.pins[str(root / 'scenario.json')]['sha256'] == entry['scenario_sha256'],
                'Selected scenario hash differs')
        require(saved == entry['audit'], 'Selected ledger/raw saved audit differs')
        split_counts[meta['split']] += 1
        sensors[str(meta.get('sensor_source_sha256'))] += 1
        actors = unique(meta['runtime_actors'], lambda item: int(item['unique_id']), 'runtime UID')
        components = saved['components']
        parent_counts = Counter(int(item['parent_uid']) for item in components)
        global_counts['all_native_components'] += len(components)
        global_counts['all_compound_native_components'] += sum(parent_counts[int(item['parent_uid'])] > 1 for item in components)
        records = None
        for component in components:
            data_id = int(component['data_id'])
            if data_id not in assets:
                global_counts['nonphysical_components_excluded'] += 1
                continue
            uid = int(component['parent_uid'])
            require(uid in actors, 'Physical component missing runtime parent')
            actor = actors[uid]
            asset = assets[data_id]
            require(actor['source_list'] == 'objectList' and actor['name'] == component['asset'] == asset['name']
                    and int(actor['data_id']) == data_id and component['training_class'] == 'obstacle',
                    'Physical catalog/runtime/component identity differs')
            payloads = unique(scenario.get('objectList', []), lambda item: int(item['UNIQUEID']), 'scenario object UID')
            require(uid in payloads and int(payloads[uid]['DataID']) == data_id, 'Scenario physical identity differs')
            kind = classify(actor, payloads[uid], parent_counts[uid])
            if records is None:
                annotations = list(root.glob('LIDAR_*/*_Instance.txt'))
                require(len(annotations) == 1, 'One native annotation timestamp required')
                records = corners.load_matching_corners(annotations[0])
                for pin in next(iter(records.values()))['source_files'].values():
                    sources.data(pin['path'])
                    require(sources.pins[pin['path']]['sha256'] == pin['sha256'], 'Native source changed during parse')
            row = native_observation(component, actor, payloads[uid], root, records, kind)
            row.update(scene_id=scene_id, split=meta['split'])
            rows.append(row)
            global_counts[kind] += 1
            observed_frames.add(scene_id)
    by_asset, candidates, deferred = {}, {}, {}
    for data_id, asset in sorted(assets.items()):
        name = asset['name']
        observed = [item for item in rows if item['data_id'] == data_id]
        unit = [item for item in observed if item['classification'] == 'static_unit_single']
        old_item = old['assets'].get(str(data_id), {})
        observed_upright = distribution([item['upright_yaw_removed_size_m'] for item in unit])
        old_reproduced = bool(observed_upright and old_item.get('observations') == len(unit)
            and np.allclose(old_item.get('size_q50_m'), observed_upright['q50_m'], rtol=0, atol=1e-10))
        candidate, reasons = candidate_reference(name, asset, observed, envelopes.get(name), name == 'RedBarrel')
        if candidate:
            candidates[name] = candidate
        else:
            deferred[name] = reasons
        measured_unit = units.get(name)
        local_distribution = distribution([item['corner_local_size_m'] for item in unit
                                           if 'corner_local_size_m' in item])
        local_q50 = np.asarray(local_distribution['q50_m']) if local_distribution else None
        envelope = envelopes.get(name)
        by_asset[name] = dict(data_id=data_id, native_components=len(observed),
            classification=dict(Counter(item['classification'] for item in observed)),
            historical_distance_reference=old_item, historical_upright_q50_reproduced=old_reproduced,
            conservative_placement_envelope=dict(source=envelope_paths.get(name), record=envelopes.get(name), modified=False),
            current_measured_unit_reference=None if not measured_unit else dict(source=unit_paths[name],
                bbox_size_m=measured_unit['bbox_size_m'],
                pivot_to_bbox_center_local_m=measured_unit['pivot_to_bbox_center_local_m'],
                contact_certified=False),
            unit_native_summary_distribution=distribution([item['native_summary_local_size_m'] for item in unit]),
            unit_upright_distribution=observed_upright,
            unit_corner_distribution=local_distribution,
            unit_corner_modes=size_modes([item for item in unit if 'corner_local_size_m' in item]),
            comparison=dict(
                observed_unit_to_conservative_envelope_axis_ratio=(local_q50 / vector(envelope['bbox_size_m'], True)).tolist()
                    if local_q50 is not None and envelope else None,
                measured_unit_reference_minus_observed_local_q50_m=(vector(measured_unit['bbox_size_m'], True) - local_q50).tolist()
                    if local_q50 is not None and measured_unit else None,
                historical_upright_q50_minus_observed_local_q50_m=(vector(old_item['size_q50_m'], True) - local_q50).tolist()
                    if local_q50 is not None and old_item else None,
                different_statistics_not_global_replacement=True),
            summary_corner_mismatch_observations=sum(item.get('summary_corner_dimensions_differ', False) for item in observed),
            axis=dict(native_local_axes='native L/W/H = x/y/z; yaw and tilt retained in raw',
                native_bbox_axis_to_unity_scale_axis=policy.get('assets', {}).get(name, {}).get('bbox_axis_to_scale_axis'),
                new_axis_qualification=False, unit_observations_do_not_prove_unity_scale_axes=True),
            diagnostic_unit_reference_available=candidate is not None, deferred_reasons=reasons,
            uncertainty=['Historical sensor/version cohort; not current simulator mesh-contact qualification.',
                         'Pivot and map height remain placement initializers, not native GT or contact truth.'])
    require(all(output != Path(path) and output not in Path(path).parents for path in sources.pins),
            'Output overlaps protected input')
    sources.verify()
    report = dict(schema=SCHEMA, passed=True, selected_ledger=str(Path(ledger).absolute()),
        selected_ledger_sha256=sources.pins[str(Path(ledger).absolute())]['sha256'],
        selected_scenes=len(selected), physical_observed_scenes=len(observed_frames),
        split_counts=dict(split_counts), sensor_hash_cohorts=dict(sensors),
        counts=dict(global_counts), physical_assets=len(assets), by_asset=by_asset,
        all_historical_upright_medians_reproduced=all(item['historical_upright_q50_reproduced'] for item in by_asset.values()),
        independent_redbarrel_variation=dict(source=str(Path(variation).absolute()),
            summary=known_variation['summary'], observations=independent_red,
            recomputed_from_native_files=len(independent_red),
            classification=dict(Counter(item['classification'] for item in independent_red)),
            unit_corner_modes=size_modes([item for item in independent_red if item['classification'] == 'static_unit_single']),
            mixed_into_canonical_unit_distribution=False),
        diagnostic_reference_assets=len(candidates), deferred=deferred,
        source_files_verified_unchanged=len(sources.pins), raw_gt_modified=False,
        conservative_placement_envelope_modified=False, simulator_or_gpu_used=False,
        four_contracts=dict(unit='native local dimensions for diagnostic scale probes only',
            envelope='existing conservative spacing/footprint; unchanged',
            pivot_height='existing calibrated proxy/map initializer; contact unverified',
            gt='per-capture native full RPY, explicit qualified corner policy, upright derived labels; unchanged'))
    reference = dict(schema='base5_diagnostic_native_unit_reference_v1', assets=candidates,
        deferred=deferred, source_audit_schema=SCHEMA,
        source_audit=str(output / 'report.json'), source_pins=str(output / 'source_pins.json'),
        source_ledger_sha256=report['selected_ledger_sha256'],
        production_approved=False, placement_only=True, training_gt_modified=False,
        limitation='Diagnostic probes only. New live unit/axis/ground/visual qualification remains mandatory; do not replace conservative placement envelopes.')
    output.mkdir(parents=True)
    atomic_json(output / 'report.json', report)
    atomic_json(output / 'observations.json', rows)
    atomic_json(output / 'source_pins.json', sources.pins)
    atomic_json(output / 'diagnostic_unit_reference.json', reference)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ledger', type=Path, default=LEDGER)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-frames', type=int, default=5000)
    args = parser.parse_args()
    result = audit(args.ledger, args.output, expected_frames=args.expected_frames)
    print(json.dumps({key: result[key] for key in ('passed', 'selected_scenes', 'physical_observed_scenes',
        'counts', 'diagnostic_reference_assets', 'deferred', 'source_files_verified_unchanged')}, indent=2))
