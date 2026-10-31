#!/usr/bin/env python3
"""Derive a stricter scale policy from pinned actual native mismatch evidence.

Only RedBarrel variants are deferred. Original actors remain available, all
other qualified profiles stay exact, and no annotation or QA threshold changes.
"""
import argparse
import copy
import json
from pathlib import Path

import base5_asset_scale as scale
from base5_capture_safety import atomic_json, sha256
from report_base5_compound_evidence import _validate_native_geometry

ASSET = 'RedBarrel'
REASON = 'mixed_scene_native_size_mismatch_with_approximately_1p25_instance_size_factor'
SCHEMA = 'base5_native_scale_asset_deferral_v1'


def read(path):
    return json.loads(Path(path).read_text())


def derive(source, cases, evidence):
    """Strict subset only; never change retained profile weights or geometry."""
    scale.validate_policy(source)
    if ASSET not in source['assets'] or len(cases) != 2:
        raise scale.ScaleContractError('RedBarrel and two independently captured mismatch scenes required')
    if (len({row['id'] for row in cases}) != 2 or
            {row['profile_id'] for row in cases} != set(source['assets'][ASSET]['profiles'])):
        raise scale.ScaleContractError('Both declared RedBarrel profiles need distinct native mismatch evidence')
    result = copy.deepcopy(source)
    removed = result['assets'].pop(ASSET)
    if len(result['assets']) != 7 or sum(len(a['profiles']) for a in result['assets'].values()) != 14:
        raise scale.ScaleContractError('Expected exactly seven retained assets and fourteen profiles')
    result['qualification'] = dict(schema=SCHEMA, source_policy=evidence['source_policy'],
        deferral_report=evidence['deferral_report'], retained_profiles_changed=False,
        deferred_assets={ASSET: dict(reason=REASON, data_id=removed['data_id'],
            profiles=sorted(removed['profiles']), original_scale_preserved=True,
            asset_removed_from_population=False, native_failure_observations=cases)},
        native_qa_unchanged=True, raw_files_modified=False, training_gt_modified=False,
        bulk_started=False, new_native_regression_required=True)
    scale.validate_policy(result)
    return result


def prepare(source_policy, source_manifest, ledgers, output, report):
    source_policy, source_manifest, output, report = map(lambda p: Path(p).resolve(),
        (source_policy, source_manifest, output, report))
    if output.exists() or report.exists():
        raise FileExistsError('New deferral policy and evidence report paths required')
    source = read(source_policy)
    scale.validate_policy(source)
    deck = read(source_manifest)
    if deck.get('count') != 5000 or len(deck.get('scenes', [])) != 5000:
        raise scale.ScaleContractError('Actual mismatch must bind to its full frozen source')
    entries = {row['id']: row for row in deck['scenes']}
    pins = {str(source_policy): sha256(source_policy), str(source_manifest): sha256(source_manifest)}
    cases = []
    for ledger in map(lambda p: Path(p).resolve(), ledgers):
        pins[str(ledger)] = sha256(ledger)
        exit_path = ledger.parent / 'shutdown_safety.json'
        if read(exit_path).get('park_verified') is not True:
            raise scale.ScaleContractError('Native mismatch session did not confirm terminal Park')
        pins[str(exit_path)] = sha256(exit_path)
        for row in [json.loads(line) for line in ledger.read_text().splitlines() if line]:
            if row.get('status') != 'review':
                continue
            root = Path(row['capture_root']).resolve()
            audit, metadata = read(root / 'audit.json'), read(root / 'metadata.json')
            if (row.get('audit') != audit or audit.get('raw_complete') is not True
                    or audit.get('accepted_candidate') is not False
                    or audit.get('review_reasons') != ['scaled_native_bbox_size_mismatch']):
                raise scale.ScaleContractError('Only actual isolated native-size review evidence may defer this policy')
            entry = entries.get(row['id'])
            if (entry is None or sha256(root / 'scenario.json') != row['scenario_sha256']
                    or row['scenario_sha256'] != entry['scenario_sha256']
                    or metadata['spec'] != read(entry['metadata'])['spec']):
                raise scale.ScaleContractError('Mismatch raw is not the frozen source scene')
            _validate_native_geometry(root, audit['components'])
            actors = {actor['unique_id']: actor for actor in metadata['runtime_actors']}
            mismatches = [item for item in audit['asset_scale_review']['actors'] if item.get('matched') is False]
            if len(mismatches) != 1 or actors[mismatches[0]['parent_uid']]['name'] != ASSET:
                raise scale.ScaleContractError('Deferral cannot conceal another asset native mismatch')
            item = mismatches[0]
            actor = actors[item['parent_uid']]
            assignment = actor['asset_scale']
            if (assignment['policy_sha256'] != scale.digest(source)
                    or assignment['profile_status'] != 'qualified' or assignment['candidate_only'] is not False):
                raise scale.ScaleContractError('Mismatch actor uses another source policy')
            ratios = [a / b for a, b in zip(item['native_size_m'], item['expected_placement_size_m'])]
            if any(abs(value - 1.25) > .001 for value in ratios):
                raise scale.ScaleContractError('Observed mismatch is not the declared per-instance 1.25 size factor')
            cases.append(dict(id=row['id'], capture_root=str(root), asset=ASSET,
                parent_uid=item['parent_uid'], profile_id=assignment['profile_id'],
                scenario_sha256=row['scenario_sha256'], native_size_m=item['native_size_m'],
                expected_placement_size_m=item['expected_placement_size_m'], observed_size_ratios=ratios,
                review_reasons=audit['review_reasons'], native_geometry_from_verified_raw_corners=True))
            for path in root.rglob('*'):
                if path.is_symlink():
                    raise scale.ScaleContractError('Deferral raw evidence must be ordinary immutable files')
                if path.is_file():
                    pins[str(path.resolve())] = sha256(path)
            for key in ('scenario', 'metadata'):
                pins[str(Path(entry[key]).resolve())] = sha256(entry[key])
    cases.sort(key=lambda row: row['id'])
    report_data = dict(schema=SCHEMA, source_policy=dict(path=str(source_policy), sha256=sha256(source_policy)),
        source_manifest=dict(path=str(source_manifest), sha256=sha256(source_manifest)),
        reason=REASON, cases=cases, source_files=pins, retained_profiles_changed=False,
        deferred_variant_asset=ASSET, original_scale_preserved=True,
        native_qa_unchanged=True, raw_files_modified=False, training_gt_modified=False, bulk_started=False)
    # Validate every requirement before writing either new immutable artifact.
    placeholder = dict(source_policy=report_data['source_policy'],
                       deferral_report=dict(path=str(report), sha256='0' * 64))
    derive(source, cases, placeholder)
    scale._pins([dict(path=path, sha256=expected) for path, expected in pins.items()])
    atomic_json(report, report_data)
    policy = derive(source, cases, dict(source_policy=report_data['source_policy'],
        deferral_report=dict(path=str(report), sha256=sha256(report))))
    atomic_json(output, policy)
    scale._pins([dict(path=path, sha256=expected) for path, expected in pins.items()])
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-policy', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--review-ledger', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.source_policy, args.manifest, args.review_ledger, args.output, args.report))
