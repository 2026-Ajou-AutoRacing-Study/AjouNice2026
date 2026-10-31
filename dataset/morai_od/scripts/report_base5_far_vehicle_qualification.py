#!/usr/bin/env python3
"""Read-only native-support report for the registered far-secondary cohort.

The terminal parent is reconstructed through its official offline verifier.
An optional terminal completion is verified attempt by attempt, never executed.
Only a new JSON report is written. This is sensor/geometry QA, not detector AP,
training qualification, a bulk-capture authorization or a ground-contact proof.
"""
import argparse
from collections import Counter
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import base5_far_vehicle_native as native
import base5_front_release_inputs as pins
import base5_secondary_far_vehicle as far
from base5_capture_audit import camera_evidence, rotation_matrix
from base5_capture_safety import atomic_json, sha256
import prepare_base5_far_vehicle_qualification as qualification
import run_base5_fresh_completion as completion


SCHEMA = 'base5_far_vehicle_native_qualification_report_v1'
ROI = (-100., -30., -3., 100., 30., 2.)
TERMINAL = ('complete', 'complete_with_deficit')


def read(path):
    return json.loads(Path(path).read_text())


def ledger(path):
    """Never invoke the collector's torn-append repair from a read-only report."""
    raw = Path(path).read_bytes()
    if raw and not raw.endswith(b'\n'):
        raise ValueError('Read-only report refuses an unterminated ledger: ' + str(path))
    rows = [json.loads(line) for line in raw.splitlines()]
    if any(not isinstance(row, dict) for row in rows):
        raise ValueError('Ledger rows must be JSON objects')
    return rows


def snapshot(root):
    """Hash every existing file, including status/ledgers and rejected RAW."""
    root = Path(root).resolve()
    if not root.is_dir():
        raise ValueError('Existing capture root required: ' + str(root))
    result = {}
    for path in sorted(root.rglob('*')):
        if path.is_file():
            if path.suffix == '.jsonl':
                ledger(path)  # Fail before a nested verifier could repair it.
            stat = path.stat()
            result[str(path.relative_to(root))] = dict(
                sha256=sha256(path), bytes=stat.st_size,
                inode=stat.st_ino, mtime_ns=stat.st_mtime_ns)
    return result


def source_inputs(registration, manifest):
    registration = Path(registration)
    if registration.is_dir():
        registration = registration / 'registration.json'
    document, deck = read(registration), read(manifest)
    selected = document.get('selected', [])
    count = len(selected)
    if (document.get('policy') != qualification.POLICY
            or document.get('subset_manifest_sha256') != sha256(manifest)
            or document.get('source_count') != 5000
            or not 1 <= count <= 64
            or deck.get('count') != count or len(deck.get('scenes', [])) != count
            or document.get('live_qualified') is not False
            or document.get('bulk_started') is not False
            or document.get('helper_sha256') != sha256(qualification.__file__)):
        raise ValueError('Registered native-cohort source/subset/helper contract changed')
    protected = {str(registration.resolve()): sha256(registration),
                 str(Path(manifest).resolve()): sha256(manifest)}
    for name in ('core_local_import_closure_sha256', 'frozen_source_dependency_sha256',
                 'full_source_offline_audit_sha256'):
        group = document.get(name)
        pins.verify_hashes(group, 'Far qualification ' + name)
        for path, digest in group.items():
            if path in protected and protected[path] != digest:
                raise ValueError('Conflicting registered source pins')
            protected[path] = digest
    candidate = Path(document['source_candidate'])
    if sha256(candidate) != document['source_candidate_sha256']:
        raise ValueError('Registered full candidate changed')
    protected[str(candidate.resolve())] = document['source_candidate_sha256']
    full = read(candidate)
    if len(full.get('scenes', [])) != 5000 or full.get('count') != 5000:
        raise ValueError('Registered full candidate is incomplete')
    full_entries = {entry['id']: entry for entry in full['scenes']}
    if len(full_entries) != 5000:
        raise ValueError('Duplicate full candidate quota slot')
    if deck['config'] != full['config']:
        raise ValueError('Subset changed the frozen full-candidate configuration')
    inventory_digest = hashlib.sha256()
    for entry in full['scenes']:
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('Full candidate scenario changed: ' + entry['id'])
        metadata_hash = sha256(entry['metadata'])
        inventory_digest.update(entry['id'].encode() + b'\0'
            + entry['scenario_sha256'].encode() + metadata_hash.encode())
        protected[str(Path(entry['scenario']).resolve())] = entry['scenario_sha256']
        protected[str(Path(entry['metadata']).resolve())] = metadata_hash
    if inventory_digest.hexdigest() != document.get('full_source_payload_metadata_digest'):
        raise ValueError('Registered full-candidate payload/metadata inventory changed')
    for name, expected in document.get('native_qa_sha256', {}).items():
        if Path(name).name != name or sha256(Path(__file__).with_name(name)) != expected:
            raise ValueError('Registered native QA source pin changed')
    ids = [entry['id'] for entry in deck['scenes']]
    if len(set(ids)) != count or [row['id'] for row in selected] != ids:
        raise ValueError('Registered subset order/unique selection changed')
    cases = {}
    expected_counts = Counter()
    for entry, registered in zip(deck['scenes'], selected):
        ident = entry['id']
        if full_entries.get(ident) != entry:
            raise ValueError('Subset is not the unchanged full-candidate entry: ' + ident)
        meta = read(entry['metadata'])
        if (sha256(entry['scenario']) != registered['scenario_sha256']
                or entry['scenario_sha256'] != registered['scenario_sha256']
                or sha256(entry['metadata']) != registered['metadata_sha256']
                or meta['spec'].get(far.PLAN_KEY) != registered['plan']):
            raise ValueError('Registered target payload/metadata/plan changed: ' + ident)
        actual = qualification.candidate_row(entry, meta)
        expected_stratum = registered['stratum']
        if (list(actual['stratum'][0]) != expected_stratum['distance_bounds_m']
                or actual['stratum'][1] != expected_stratum['role']
                or actual['stratum'][2] != expected_stratum['lane_relation']
                or any(actual[key] != registered[key] for key in
                       ('asset', 'data_id', 'target_uid', 'target_ordinal', 'seed', 'split'))):
            raise ValueError('Registered target identity/stratum changed: ' + ident)
        cases[ident] = dict(entry=entry, metadata=meta, registered=registered)
        lo, hi = actual['stratum'][0]
        expected_counts[f'{lo}_{hi}/{actual["stratum"][1]}/{actual["stratum"][2]}'] += 1
        protected[str(Path(entry['scenario']).resolve())] = registered['scenario_sha256']
        protected[str(Path(entry['metadata']).resolve())] = registered['metadata_sha256']
    bands = {tuple(row['stratum']['distance_bounds_m']) for row in selected}
    if (dict(expected_counts) != document.get('stratum_counts')
            or len(expected_counts) != len(bands) * len(qualification.ROLES) * len(qualification.LANES)
            or set(expected_counts.values()) != {2}):
        raise ValueError('Expected same2/adjacent2 in every registered band/direction')
    # Includes original v152 slot membership, external pins and serialized pose.
    protected.update(native.validate_manifest(deck))
    pins.verify_hashes(protected, 'Far qualification source')
    return registration, document, deck, cases, protected


def check_selection(rows, allowed):
    result, captures, payloads = {}, set(), set()
    for row in rows:
        ident = row.get('source_cell_id')
        capture = str(Path(row['capture_root']).resolve())
        if (ident not in allowed or ident in result or capture in captures
                or row.get('scenario_sha256') in payloads
                or row.get('status') != 'accepted_candidate'
                or row.get('audit', {}).get('review_reasons')):
            raise ValueError('Duplicate/foreign/reviewed native selection')
        result[ident] = row
        captures.add(capture)
        payloads.add(row['scenario_sha256'])
    return result


def verify_completion(root, baseline, manifest, deck, parent_rows, deficits, cases):
    """Reconstruct the terminal selection without calling completion.execute."""
    root, baseline = Path(root), Path(baseline)
    state, status, quota = read(root / 'state.json'), read(root / 'status.json'), read(root / 'quota.json')
    contract = state.get('contract', {})
    if (state.get('stage') not in TERMINAL or status.get('stage') != state['stage']
            or state.get('baseline_selected') != parent_rows
            or state.get('deficits') != deficits
            or state.get('source_order') != list(cases)
            or contract.get('policy') != completion.POLICY
            or contract.get('source_sha256') != sha256(manifest)
            or Path(contract.get('baseline', '')).resolve() != baseline.resolve()
            or type(contract.get('attempts')) is not int or not 1 <= contract['attempts'] <= 5):
        raise ValueError('Terminal fresh completion source/parent/state contract changed')
    expected_parent = ('selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json')
    if contract.get('parent_hashes') != {name: sha256(baseline / name) for name in expected_parent}:
        raise ValueError('Completion parent input pins changed')
    code = contract.get('code_hashes', {})
    if not code or any(Path(name).name != name for name in code):
        raise ValueError('Completion runtime pins are missing or malformed')
    required_code = {'run_base5_fresh_completion.py', 'base5_spatial_diversity.py',
        'run_base5_flexible_completion.py', 'run_base5_adaptive_completion.py',
        'run_base5_capture_optimized.py', 'run_base5_capture_campaign_spatial.py',
        'base5_secondary_far_vehicle.py', 'base5_far_vehicle_native.py'}
    if not required_code <= set(code):
        raise ValueError('Completion runtime pins omit required frozen far scripts')
    pins.verify_hashes({str(Path(__file__).with_name(name)): digest for name, digest in code.items()},
                       'Completion runtime')
    if contract.get('capture_contract') != completion.adaptive.fa.batch.campaign.contract(
            Path(manifest), list(range(len(cases))), 2):
        raise ValueError('Completion capture runtime contract changed')
    pools = completion.pool_map(completion.spatial.gen.context(
        completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
        completion.spatial.gen.geo.DEFAULT_MGEO))
    seen = set()
    for attempt in state.get('attempts', []):
        key = (attempt.get('id'), attempt.get('ordinal'))
        if (key in seen or key[0] not in deficits or type(key[1]) is not int
                or not 1 <= key[1] <= contract['attempts'] or attempt.get('result') is None):
            raise ValueError('Completion has duplicate/foreign/unresolved attempt')
        seen.add(key)
        result = completion.verify(attempt, deck, cases[key[0]]['metadata'], pools)
        if result != attempt['result']:
            raise ValueError('Completion official raw verification differs from journal')
    rows, missing = completion.merged(state)
    if rows != ledger(root / 'selected_ledger.jsonl'):
        raise ValueError('Completion selected ledger differs from verified state')
    if (quota.get('target') != len(cases) or quota.get('accepted') != len(rows)
            or quota.get('deficit') != len(missing) or quota.get('pending') != 0
            or [row['id'] for row in quota.get('cells', [])] != missing
            or status.get('requested') != len(cases)
            or status.get('accepted_candidate') != len(rows)
            or status.get('remaining_deficit') != len(missing)
            or status.get('review') != len(missing)
            or (state['stage'] == 'complete') != (not missing)):
        raise ValueError('Terminal completion counts/status/deficits differ')
    return rows, missing, state


def distribution(values):
    return dict(zip(('min', 'p50', 'p90', 'p95', 'max'),
                    np.percentile(values, (0, 50, 90, 95, 100)).tolist())) if values else None


def target_observation(case, selected):
    """Recheck target-only sensor evidence against the saved unmodified RAW."""
    entry, expected = case['entry'], case['metadata']
    root = Path(selected['capture_root'])
    actual, audit = read(root / 'metadata.json'), read(root / 'audit.json')
    if (audit != selected.get('audit') or audit.get('accepted_candidate') is not True
            or audit.get('review_reasons')
            or sha256(root / 'scenario.json') != selected['scenario_sha256']):
        raise ValueError('Selected native RAW/audit identity differs')
    plan = far.verify_preserved(expected, actual)
    if plan != case['registered']['plan']:
        raise ValueError('Selected target plan changed')
    frozen = read(plan['source']['scenario']['path'])
    far.verify_payload(read(root / 'scenario.json'), frozen, plan, actual)
    reasons, proof = native.review(actual, audit['components'])
    if reasons or proof != audit.get('far_vehicle_native'):
        raise ValueError('Stored allocated-target native evidence differs')
    components = [c for c in audit['components'] if c['parent_uid'] == plan['target_uid']
                  and c.get('training_class') == 'vehicle']
    if len(components) != 1:
        raise ValueError('Allocated target has no unique native Vehicle component')
    component = components[0]
    center = np.asarray(component['center_lidar_m'], dtype=float)
    size = np.asarray(component['size_m'], dtype=float)
    if (center.shape != (3,) or size.shape != (3,) or not np.isfinite(center).all()
            or not np.isfinite(size).all() or (size <= 0).any()):
        raise ValueError('Malformed native target geometry')
    calibration = np.asarray(actual.get('lidar_to_ego'), dtype=float)
    if (calibration.shape != (4, 4) or not np.isfinite(calibration).all()
            or not np.allclose(calibration[3], [0., 0., 0., 1.], atol=1e-8)):
        raise ValueError('Calibrated native LiDAR-to-Ego geometry required')
    calibrated = (calibration @ np.r_[center, 1.])[:3]
    if (not np.allclose(calibrated, component['center_ego_m'], rtol=0, atol=1e-5)
            or not math.isclose(math.hypot(*calibrated[:2]), component['range_ego_xy_m'],
                                rel_tol=0, abs_tol=1e-4)):
        raise ValueError('Recorded native Ego distance disagrees with LiDAR calibration')
    paths = list(root.glob('LIDAR_*/*_Intensity.bin'))
    if len(paths) != 1:
        raise ValueError('One synchronous LiDAR RAW required')
    points = np.fromfile(paths[0], dtype=np.float32)
    if not points.size or points.size % 4 or not np.isfinite(points).all():
        raise ValueError('Malformed XYZI target evidence')
    points = points.reshape(-1, 4)
    points = points[(points[:, :3] != 0).any(axis=1)]
    local = (points[:, :3] - center) @ rotation_matrix(component['rotation_rpy'])
    inside = (np.abs(local) <= size / 2 + 1e-5).all(axis=1)
    native_points = int(inside.sum())
    model_roi = ((points[:, :3] > np.asarray(ROI[:3]))
                 & (points[:, :3] < np.asarray(ROI[3:]))).all(axis=1)
    filtered_points = int((inside & model_roi).sum())
    if native_points != component['lidar_points']:
        raise ValueError('Saved native point support differs from XYZI/OBB')
    cameras = camera_evidence(root, plan['target_uid'], component['raw_class'], model512=True)
    if cameras != component['cameras']:
        raise ValueError('Saved camera support differs from native masks/annotations')
    views = [dict(camera=c['camera'], native_pixels=c['native_instance_pixels'],
                  model512_pixels=c['crop_512x1056_instance_pixels'],
                  model512_extent_px=c['model512_extent_px'],
                  model512_gate=c['crop_512x1056_instance_pixels'] >= 16
                  and min(c['model512_extent_px']) >= 4) for c in cameras]
    best = max(views, key=lambda c: c['model512_pixels'], default=None)
    actor = actual['runtime_actors'][plan['target_ordinal']]
    source = read(plan['source']['metadata']['path'])['runtime_actors'][plan['target_ordinal']]
    current = proof['components'][0]
    lidar_distance = math.hypot(*center[:2])
    ground = audit.get('ground_support') or {}
    target_ground = [a for a in ground.get('actors', []) if a.get('uid') == plan['target_uid']]
    return dict(id=entry['id'], capture_root=str(root.resolve()), capture_id=selected['id'],
        scenario_sha256=selected['scenario_sha256'], target_uid=plan['target_uid'],
        target_ordinal=plan['target_ordinal'], asset=plan['target_name'], data_id=plan['target_data_id'],
        component_id=component['component_id'], raw_class=component['raw_class'],
        training_class=component['training_class'], body_group=case['registered'].get('body_group'),
        distance_bounds_m=plan['distance_bounds_m'], role=plan['role'], lane_relation=plan['lane_relation'],
        original_source_distance_ego_xy_m=source['planned_center_distance_m'],
        planned_distance_ego_xy_m=actor['planned_center_distance_m'],
        actual_distance_ego_xy_m=current['range_ego_xy_m'],
        actual_distance_lidar_xy_m=lidar_distance, center_ego_m=current['center_ego_m'],
        center_lidar_m=center.tolist(), size_m=size.tolist(),
        center_inside_current_model_roi=bool(np.all(center >= ROI[:3]) and np.all(center < ROI[3:])),
        model_roi=list(ROI), native_lidar_points=native_points,
        filtered_model_lidar_points=filtered_points,
        native_points_bucket='0' if native_points == 0 else '1_4' if native_points < 5 else '5plus',
        camera512_gate=current['camera512_gate'], camera_views=views,
        best_model512_pixels=best['model512_pixels'] if best else None,
        best_model512_extent_px=best['model512_extent_px'] if best else None,
        camera_only=native_points == 0 and current['camera512_gate'],
        frozen_primary_and_all_non_target_preserved=True,
        ground_target=target_ground, ground_warnings=audit.get('ground_warnings', []),
        capture_stationary_support=actual.get('capture_safety', {}).get('support'),
        capture_collision=actual.get('capture_safety', {}).get('collision'),
        native_support_only_not_detector_accuracy=True)


def summarize(observations, cases):
    by_id = {row['id']: row for row in observations}
    if len(by_id) != len(observations) or not set(by_id) <= set(cases):
        raise ValueError('Duplicate or foreign target observation')
    strata = {}
    bands = sorted({tuple(case['registered']['stratum']['distance_bounds_m']) for case in cases.values()})
    for band in bands:
        for role in qualification.ROLES:
            for lane in qualification.LANES:
                key = f'{band[0]}_{band[1]}/{role}/{lane}'
                expected = [ident for ident, case in cases.items()
                            if case['registered']['stratum'] == dict(
                                distance_bounds_m=list(band), role=role, lane_relation=lane)]
                values = [by_id[ident] for ident in expected if ident in by_id]
                strata[key] = dict(requested=len(expected), accepted=len(values),
                    remaining=[ident for ident in expected if ident not in by_id],
                    native_lidar_buckets=dict(Counter(r['native_points_bucket'] for r in values)),
                    filtered_model_lidar_buckets=dict(Counter(
                        '0' if r['filtered_model_lidar_points'] == 0 else '1_4'
                        if r['filtered_model_lidar_points'] < 5 else '5plus' for r in values)),
                    camera512_gate=sum(r['camera512_gate'] for r in values),
                    camera_only=sum(r['camera_only'] for r in values),
                    model_roi_center_retained=sum(r['center_inside_current_model_roi'] for r in values),
                    native_lidar_points=distribution([r['native_lidar_points'] for r in values]),
                    camera512_pixels=distribution([r['best_model512_pixels'] for r in values
                                                   if r['best_model512_pixels'] is not None]),
                    bbox_short_side_px=distribution([min(r['best_model512_extent_px']) for r in values
                                                    if r['best_model512_extent_px']]),
                    actual_ego_distance_m=distribution([r['actual_distance_ego_xy_m'] for r in values]),
                    actual_lidar_distance_m=distribution([r['actual_distance_lidar_xy_m'] for r in values]),
                    asset_counts=dict(Counter(r['asset'] for r in values)))
    return strata


def report(registration, manifest, baseline, output, completion_root=None):
    output, manifest, baseline = Path(output).resolve(), Path(manifest).resolve(), Path(baseline).resolve()
    if output.exists():
        raise FileExistsError('A new JSON report path is required')
    roots = [baseline] + ([Path(completion_root).resolve()] if completion_root else [])
    if any(root == output or root in output.parents for root in roots):
        raise ValueError('Report output must be outside preserved capture roots')
    registration_path, registered, deck, cases, source_pins = source_inputs(registration, manifest)
    if Path(registered['source_candidate']).resolve().parent in output.parents:
        raise ValueError('Report must not write into the frozen full candidate')
    if read(baseline / 'status.json').get('stage') not in TERMINAL:
        raise ValueError('Only a terminal native parent may be reported')
    # Imported parent selections may reference RAW outside this campaign. Keep
    # those physical observations under the same before/after preservation gate.
    for root in list(roots):
        for row in ledger(root / 'selected_ledger.jsonl'):
            capture = Path(row['capture_root']).resolve()
            if not any(parent == capture or parent in capture.parents for parent in roots):
                roots.append(capture)
    if any(root == output or root in output.parents for root in roots):
        raise ValueError('Report output must be outside every referenced RAW root')
    before = {str(root): snapshot(root) for root in roots}
    parent_rows, parent_missing, parent_elapsed = completion.verify_parent(manifest, baseline)
    if parent_rows != ledger(baseline / 'selected_ledger.jsonl'):
        raise ValueError('Native parent ledger differs from official reconstruction')
    parent_selected = check_selection(parent_rows, cases)
    if set(parent_missing) != set(cases) - set(parent_selected):
        raise ValueError('Parent native deficits do not match the registered cohort')
    selected, missing, state = parent_rows, parent_missing, None
    if completion_root:
        selected, missing, state = verify_completion(Path(completion_root), baseline, manifest,
            deck, parent_rows, parent_missing, cases)
    final = check_selection(selected, cases)
    if set(missing) != set(cases) - set(final):
        raise ValueError('Final native deficits do not match registered slots')
    if any(final.get(ident) != row for ident, row in parent_selected.items()):
        raise ValueError('Completion changed an existing parent approval')
    observations = [target_observation(cases[ident], final[ident]) for ident in cases if ident in final]
    after = {str(root): snapshot(root) for root in roots}
    if before != after:
        raise ValueError('Offline report verification changed capture RAW/journal/status files')
    pins.verify_hashes(source_pins, 'Far qualification source after verification')
    result = dict(schema=SCHEMA, created_kst=datetime.now(timezone(timedelta(hours=9))).isoformat(),
        report_script_sha256=sha256(__file__), registration=str(registration_path.resolve()),
        registration_sha256=sha256(registration_path), manifest=str(manifest),
        subset_manifest_sha256=sha256(manifest), source_input_sha256=source_pins,
        baseline=str(baseline), completion=str(Path(completion_root).resolve()) if completion_root else None,
        requested=len(cases), parent_accepted=len(parent_rows), parent_remaining=parent_missing,
        final_accepted=len(selected), remaining=missing,
        accepted_from_completion=len(selected) - len(parent_rows),
        parent_elapsed_sec=parent_elapsed, completion_attempts=len(state['attempts']) if state else 0,
        primary_and_all_non_target_preserved=True, unique_selected_raw_observations=True,
        immutable_evidence=dict(before=before, after_equal=True,
            raw_rejected_and_accepted_preserved=True, parent_ledger_and_status_sha_unchanged=True),
        strata=summarize(observations, cases), observations=observations,
        native_support_qualified_all_registered_targets=not missing,
        detector_ap_improvement_claim=False, training_started=False, bulk_started=False,
        limitations=[
            'Native sensor/geometry support only; not detector recall, AP, heading stability or temporal tracking accuracy.',
            'Camera foreground and bbox extent are simulator annotations after deterministic512 input; not learned evidence.',
            'Native bbox points and current model XY/Z-filtered bbox points have different denominators and are both reported.',
            'Allocated far Vehicle GT-center inclusion in the current LiDAR ROI is an explicit native acceptance gate; support counts are reported separately and no GT is rewritten. Legacy actor gates are unchanged.',
            'Registered bands use planned Ego-center distance; calibrated native LiDAR-center distance is different. A 95-99m Ego cohort alone does not certify actual LiDAR 95+ m coverage.',
            'Each band/direction/lane cell has very few targets; quantiles describe those observations and are not accuracy, reliability or generalization estimates.',
            'Measured ground, warnings and stationary Z proxies do not certify mesh contact at every far target.',
            'Parent selections are retained exactly; rejected RAW is retained and never added as an approval.',
            'The registered targets are a diagnostic cohort, not broad all-weather or final5K qualification.'])
    atomic_json(output, result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('registration', 'manifest', 'baseline', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--completion', type=Path)
    args = parser.parse_args()
    result = report(args.registration, args.manifest, args.baseline, args.output, args.completion)
    print(json.dumps(dict(accepted=result['final_accepted'], requested=result['requested'],
                          remaining=result['remaining'], output=str(args.output)), ensure_ascii=False))
