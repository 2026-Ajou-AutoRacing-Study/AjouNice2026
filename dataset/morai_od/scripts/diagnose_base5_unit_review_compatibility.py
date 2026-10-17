#!/usr/bin/env python3
"""Replay an explicit unit-size review policy on immutable historical captures.

The policy tag is attached to RAM metadata only. No metadata sidecar, source
status, raw, GT, acceptance decision, or legacy QA is rewritten or upgraded.
Canonical selected 5K and independent RedBarrel observations stay separate.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import time

from audit_base5_geometry_reference import LEDGER, Sources, classify, require, unique
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json


REPO = Path(__file__).resolve().parents[3]
RED_EVIDENCE = REPO / 'artifacts/evaluation/base5_mission_scale_261016/redbarrel_native_variation_v155l.json'
SCHEMA = 'base5_historical_unit_review_compatibility_v1'


def review_in_memory(metadata, audit, reference):
    """Never change the caller's metadata, audit, or existing policy tag."""
    if unit.KEY in metadata and metadata[unit.KEY] != reference:
        raise unit.UnitGeometryContractError('Do not replace an existing historical unit policy')
    tagged = copy.deepcopy(metadata)
    tagged[unit.KEY] = copy.deepcopy(reference)
    return unit.review(tagged, copy.deepcopy(audit))


def pin_policy(reference, sources):
    policy = unit.load(reference)
    sources.data(reference['path'])
    require(sources.pins[str(Path(reference['path']).absolute())]['sha256'] == reference['sha256'],
            'Unit policy changed during source registration')
    for pin in [policy['catalog_reference']] + policy['sources']:
        sources.data(pin['path'])
        require(sources.pins[str(Path(pin['path']).absolute())]['sha256'] == pin['sha256'],
                'Unit catalog/native reference changed during source registration')
    return policy


def actor_classifications(metadata, audit, scenario, policy):
    actors = unique(metadata['runtime_actors'], lambda row: int(row['unique_id']), 'runtime parent UID')
    payloads = unique(scenario.get('objectList', []), lambda row: int(row['UNIQUEID']), 'scenario object UID')
    counts = Counter(int(row['parent_uid']) for row in audit['components'])
    result = {}
    for uid, actor in actors.items():
        if actor['name'] not in policy['assets']:
            continue
        require(uid in payloads and int(payloads[uid]['DataID']) == actor['data_id'],
                'Historical unit actor/scenario identity differs')
        factors = (actor.get('asset_scale') or {}).get('scale_xyz', [1., 1., 1.])
        probe = metadata.get('scale_probe', {})
        if probe.get('parent_uid') == uid:
            factors = probe['requested_scale']
        require(list(factors) == [payloads[uid]['scale'][axis] for axis in 'xyz'],
                'Historical actor metadata/scenario scale differs')
        result[uid] = classify(actor, payloads[uid], counts[uid])
    return result


def summarize_reviews(records, names):
    by_asset = {name: Counter(compared=0, matched=0, mismatched=0, component_contract_changed=0,
                              skipped_nonunit=0, skipped_moving_unit=0) for name in names}
    scenes_with_review, reasons = set(), Counter()
    for record in records:
        if record['review']['review_reasons']:
            scenes_with_review.add(record['scene_id'])
        reasons.update(record['review']['review_reasons'])
        compared_uids = set()
        for row in record['review']['actors']:
            stats = by_asset[row['name']]
            compared_uids.add(row['parent_uid'])
            if row['compared']:
                stats['compared'] += 1
                stats['matched' if row['matched'] else 'mismatched'] += 1
            else:
                stats['component_contract_changed'] += 1
        for actor in record['registered_actors']:
            kind = record['classification'][actor['unique_id']]
            if actor['unique_id'] not in compared_uids:
                if kind == 'nonunit_single':
                    by_asset[actor['name']]['skipped_nonunit'] += 1
                elif kind == 'moving_unit_single':
                    by_asset[actor['name']]['skipped_moving_unit'] += 1
    totals = Counter()
    for counts in by_asset.values():
        totals.update(counts)
    return dict(scenes=len(records), scenes_with_unit_review=len(scenes_with_review),
                reason_counts=dict(reasons), totals=dict(totals),
                by_asset={name: dict(counts) for name, counts in by_asset.items()})


def capture_record(scene_id, root, metadata, audit, scenario, policy, reference):
    classification = actor_classifications(metadata, audit, scenario, policy)
    return dict(scene_id=scene_id, capture_root=str(root),
                classification=classification,
                registered_actors=[dict(name=actor['name'], unique_id=actor['unique_id'])
                    for actor in metadata['runtime_actors'] if actor['name'] in policy['assets']],
                review=review_in_memory(metadata, audit, reference))


def diagnose(ledger, reference, red_evidence, output, *, expected_frames=5000,
             expected_red_unit_mismatches=2):
    output = Path(output).absolute()
    require(not output.exists(), 'Fresh diagnostic output required; never overwrite evidence')
    started, cpu_started = time.perf_counter(), time.process_time()
    sources = Sources()
    for path in (__file__, unit.__file__, REPO / 'dataset/morai_od/scripts/audit_base5_geometry_reference.py',
                 REPO / 'dataset/morai_od/scripts/base5_capture_safety.py'):
        sources.data(path)
    policy = pin_policy(reference, sources)
    selected = unique((json.loads(line) for line in sources.data(ledger).splitlines() if line.strip()),
                      lambda row: row['id'], 'canonical selected scene ID')
    require(len(selected) == expected_frames, 'Canonical selected frame count differs')
    canonical, splits, roots = [], Counter(), set()
    for scene_id, row in sorted(selected.items()):
        require(row['status'] == 'accepted_candidate', 'Canonical ledger must contain accepted captures only')
        root = Path(row['capture_root']).absolute()
        require(root not in roots and root != output and root not in output.parents,
                'Duplicate canonical root or diagnostic output inside raw')
        roots.add(root)
        metadata = sources.json(root / 'metadata.json')
        audit = sources.json(root / 'audit.json')
        scenario = sources.json(root / 'scenario.json')
        require(audit == row['audit'], 'Canonical saved audit differs from selected ledger')
        require(sources.pins[str(root / 'scenario.json')]['sha256'] == row['scenario_sha256'],
                'Canonical scenario hash differs')
        splits[metadata['split']] += 1
        canonical.append(capture_record(scene_id, root, metadata, audit, scenario, policy, reference))
    canonical_sec = time.perf_counter() - started
    red_started = time.perf_counter()
    evidence = sources.json(red_evidence)
    require(evidence.get('passed') is True and evidence['asset']['name'] == 'RedBarrel',
            'Explicit validated independent RedBarrel evidence required')
    for path, digest in evidence['source_files'].items():
        sources.data(path)
        require(sources.pins[str(Path(path).absolute())]['sha256'] == digest,
                'Independent RedBarrel source changed')
    observed = unique(evidence['observations'],
        lambda row: (str(Path(row['capture_root']).absolute()), int(row['parent_uid'])),
        'independent RedBarrel observation')
    red_roots = sorted({Path(key[0]) for key in observed})
    red, red_observations = [], []
    for root in red_roots:
        require(root != output and root not in output.parents, 'Never write diagnostic output inside raw')
        metadata = sources.json(root / 'metadata.json')
        audit = sources.json(root / 'audit.json')
        scenario = sources.json(root / 'scenario.json')
        # Filter only the in-memory diagnostic view, not any source metadata.
        filtered = copy.deepcopy(metadata)
        filtered['runtime_actors'] = [actor for actor in filtered['runtime_actors'] if actor['name'] == 'RedBarrel']
        uids = {int(actor['unique_id']) for actor in filtered['runtime_actors']}
        require(uids == {uid for path, uid in observed if path == str(root)},
                'Independent RedBarrel actor inventory differs from evidence')
        filtered_audit = dict(components=[component for component in audit['components']
                                         if int(component['parent_uid']) in uids])
        record = capture_record(str(root), root, filtered, filtered_audit, scenario, policy, reference)
        red.append(record)
        compared = {row['parent_uid']: row for row in record['review']['actors']}
        for uid in sorted(uids):
            original = observed[(str(root), uid)]
            result = compared.get(uid)
            red_observations.append(dict(capture_root=str(root), case=original['case'], stage=original['stage'],
                parent_uid=uid, classification=record['classification'][uid],
                scenario_scale_xyz=original['scenario_scale_xyz'],
                compared=result is not None and result['compared'],
                matched=result.get('matched') if result else None,
                native_summary_size_m=original['native_summary_size_m'],
                independent_observed_summary_state=original['observed_summary_state'], review=result))
    red_sec = time.perf_counter() - red_started
    canonical_summary = summarize_reviews(canonical, sorted(policy['assets']))
    red_summary = summarize_reviews(red, ['RedBarrel'])
    red_summary['observations'] = len(red_observations)
    red_summary['classification'] = dict(Counter(row['classification'] for row in red_observations))
    checks = dict(canonical_unit_mismatches_zero=canonical_summary['totals']['mismatched'] == 0,
        canonical_component_contract_changes_zero=canonical_summary['totals']['component_contract_changed'] == 0,
        historical_red_unit_mismatch_count_expected=red_summary['totals']['mismatched'] == expected_red_unit_mismatches,
        historical_red_nonunit_never_compared=all(not row['compared'] for row in red_observations
                                                  if row['classification'] == 'nonunit_single'))
    verify_started = time.perf_counter()
    sources.verify()
    report = dict(schema=SCHEMA, diagnostic_completed=True, verification_passed=all(checks.values()),
        reference=reference, selected_ledger=str(Path(ledger).absolute()),
        selected_ledger_sha256=sources.pins[str(Path(ledger).absolute())]['sha256'],
        canonical=canonical_summary, canonical_split_counts=dict(splits),
        historical_red=red_summary, expected_red_unit_mismatches=expected_red_unit_mismatches,
        checks=checks, source_files_verified_unchanged=len(sources.pins),
        timing_sec=dict(canonical=canonical_sec, independent_red=red_sec,
            final_source_verification=time.perf_counter()-verify_started,
            total_wall_before_report_write=time.perf_counter()-started,
            total_cpu_before_report_write=time.process_time()-cpu_started),
        metadata_tag_attached_in_memory_only=True, metadata_sidecars_created=False,
        original_status_or_acceptance_changed=False, raw_or_gt_modified=False,
        simulator_ros_gpu_used=False, legacy_qa_upgraded=False,
        limitations=['Policy compatibility replay only; historical scene QA and acceptance are unchanged.',
            'The detector compares saved native local summary size; it does not certify actual mesh/collider geometry.',
            'Independent historical RedBarrel observations are not merged into the canonical 5K.',
            'Nonunit actors remain subject to their distinct scale contract; this diagnostic does not approve recovery or bulk.'])
    output.mkdir(parents=True)
    atomic_json(output / 'report.json', report)
    atomic_json(output / 'canonical_reviews.json', canonical)
    atomic_json(output / 'historical_red_observations.json', red_observations)
    atomic_json(output / 'source_pins.json', sources.pins)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--reference-sha256', required=True)
    parser.add_argument('--ledger', type=Path, default=LEDGER)
    parser.add_argument('--red-evidence', type=Path, default=RED_EVIDENCE)
    parser.add_argument('--expected-frames', type=int, default=5000)
    parser.add_argument('--expected-red-unit-mismatches', type=int, default=2)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = diagnose(args.ledger, dict(path=str(args.reference.absolute()), sha256=args.reference_sha256),
        args.red_evidence, args.output, expected_frames=args.expected_frames,
        expected_red_unit_mismatches=args.expected_red_unit_mismatches)
    print(json.dumps({key: result[key] for key in ('diagnostic_completed', 'verification_passed',
        'canonical', 'historical_red', 'checks', 'source_files_verified_unchanged', 'timing_sec')}, indent=2))
    if not result['verification_passed']:
        raise SystemExit(1)
