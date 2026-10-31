#!/usr/bin/env python3
"""Prepare at most two ordinary visibility retries inside frozen mission strata.

This is a finite pre-freeze experiment, not a bulk recollector or QA waiver.
Only the observed Dog2/concrete-barrier pilot slots may be regenerated. Their
specs, assets, phase, split, yaw/gap policy and all other 4,998 entries remain
unchanged. Captures and all failed evidence are read-only.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path

import audit_base5_scenarios
import base5_front_release_inputs as pins
import base5_spatial_diversity as spatial
import base5_static_mission_extent as extent
from base5_capture_safety import atomic_json, sha256
from prepare_base5_far_vehicle_qualification import source_audit_pins
from prepare_base5_front_support_qualification import validate_source
from run_base5_failure_aware_replenishment_fast import prewarm


POLICY = 'static-extent-bounded-primary-visibility-recovery-v155'
TARGET_IDS = frozenset(('sunny13_sunny11_b5v2_04060', 'sunny13_sunny11_b5v2_03149'))
MAX_ATTEMPTS = 3
MAX_PROPOSALS = 64


class RecoveryContractError(RuntimeError):
    """A provenance/safety/frozen quota mismatch, never an ordinary retry."""


def read(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise RecoveryContractError(message)


def _snapshot(root):
    root = Path(root).resolve(strict=True)
    require(all(not path.is_symlink() for path in root.rglob('*')), 'Recovery evidence cannot contain symlinks')
    return {str(path.resolve()): sha256(path) for path in root.rglob('*') if path.is_file()}


def review_rows(ledger, entries, metadata):
    """Only ordinary visibility reviews after verified collision-clear/Park end."""
    ledger = Path(ledger).resolve()
    rows = [json.loads(line) for line in ledger.read_text().splitlines() if line.strip()]
    require(rows and len(rows) <= 16 and len({row['id'] for row in rows}) == len(rows),
            'A bounded, unique pilot ledger is required')
    park_path = ledger.parent/'shutdown_safety.json'
    require(park_path.is_file() and read(park_path).get('park_verified') is True,
            'Verified terminal Park evidence required before offline recovery')
    inputs = {str(ledger): sha256(ledger), str(park_path): sha256(park_path)}
    failures = {}
    for row in rows:
        ident = row['id']
        require(ident in entries and row['scenario_sha256'] == entries[ident]['scenario_sha256'],
                'Pilot row is foreign or refers to another source payload')
        require(row['status'] in ('accepted_candidate', 'review'),
                'Safety/communication/failed capture cannot be visibility-recovered')
        root = Path(row['capture_root']).resolve(strict=True)
        audit = read(root/'audit.json')
        require(audit == row.get('audit') and audit.get('raw_complete') is True,
                'Actual raw audit must match the pilot ledger')
        collision = audit.get('capture_safety', {}).get('collision', {})
        require(collision.get('observed') is True and collision.get('no_collision_verified') is True
                and not collision.get('non_ego_uids'), 'Collision-clear receive evidence is mandatory')
        require(audit.get('capture_safety', {}).get('support', {}).get('stationary_support_observed') is True,
                'Observed stationary support is mandatory')
        if row['status'] == 'accepted_candidate':
            require(audit.get('accepted_candidate') is True and not audit.get('review_reasons'),
                    'Accepted pilot audit is inconsistent')
            continue
        require(ident in TARGET_IDS and audit.get('accepted_candidate') is False
                and audit.get('review_reasons') == ['primary_no_sensor_evidence']
                and not audit.get('missing_uids') and not audit.get('runtime_missing_uids'),
                'Only the two declared primary-no-sensor-evidence reviews may recover')
        before = metadata[ident]
        captured = read(root/'metadata.json')
        require(captured.get('spec') == before['spec']
                and extent.identity(captured.get('runtime_actors', [])) == extent.identity(before['runtime_actors'])
                and sha256(root/'scenario.json') == entries[ident]['scenario_sha256'],
                'Failure raw/spec/actor identity differs from the frozen source')
        require(extent.eligible(before['spec']) and extent.PLAN_KEY in before['spec'],
                'Visibility recovery requires an existing stationary-mission extent plan')
        inputs.update(_snapshot(root))
        failures[ident] = dict(row=copy.deepcopy(row), capture_root=str(root), audit_sha256=sha256(root/'audit.json'))
    require(1 <= len(failures) <= 2, 'One or two explicit ordinary pilot reviews required')
    return failures, inputs


def verify_transition(before, after):
    require(after['spec'] == before['spec'] and after['split'] == before['split']
            and extent.identity(after['runtime_actors']) == extent.identity(before['runtime_actors']),
            'Recovery changed frozen spec/split/asset/motion identity')
    require(type(after.get('retry')) is int and after['retry'] > before['retry'],
            'Recovery did not advance the deterministic geometry retry ordinal')
    require(after.get('static_mission_extent', {}).get('stratum')
            == before.get('static_mission_extent', {}).get('stratum')
            and after.get('static_mission_extent', {}).get('primary_center_not_ego_used_for_coverage') is True,
            'Recovery changed extent coverage or stopped checking primary-center location')
    return True


def next_attempt(deck, requested):
    require(type(requested) is int and 1 <= requested <= MAX_ATTEMPTS,
            'Explicit finite recovery attempt 1..3 required')
    prior = deck.get('provenance', {}).get('static_extent_visibility_recovery')
    previous = prior.get('attempt') if prior else 0
    require(type(previous) is int and requested == previous + 1,
            'Use the immediately preceding full candidate; recovery budget cannot be reset')
    return requested


def first_candidate(before, config, ctx, known_hashes, maximum=MAX_PROPOSALS, build=None):
    require(type(maximum) is int and 1 <= maximum <= MAX_PROPOSALS,
            'At most 64 finite CPU proposals per declared visibility review')
    build = build or spatial.build
    rejected = Counter()
    first = before['retry'] + 1
    for retry in range(first, first + maximum):
        try:
            scene, meta = build(copy.deepcopy(before['spec']), config, ctx, retry)
        except ValueError as error:
            rejected[str(error)] += 1
            continue
        verify_transition(before, meta)
        payload = (json.dumps(scene, ensure_ascii=False, indent=2) + '\n').encode()
        import hashlib
        checksum = hashlib.sha256(payload).hexdigest()
        if checksum in known_hashes:
            rejected['duplicate_source_or_candidate_payload'] += 1
            continue
        return scene, meta, dict(first_retry=first, selected_retry=retry,
            maximum_proposals=maximum, proposals_tried=retry-first+1,
            rejected=dict(rejected), scenario_sha256=checksum,
            selection='first geometrically valid distinct payload; no visibility/outcome ranking')
    raise RecoveryContractError('Finite primary visibility recovery geometry exhausted')


def prepare(source, failed_ledger, output, attempt=1, maximum_proposals=MAX_PROPOSALS):
    source, failed_ledger, output = map(lambda path: Path(path).resolve(), (source, failed_ledger, output))
    require(not output.exists(), 'New recovery output path required')
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    require(output != runtime and runtime not in output.parents, 'Offline recovery cannot write simulator runtime')
    require(output != source.parent and source.parent not in output.parents,
            'Recovery cannot write inside the preserved full source candidate')
    deck = read(source)
    next_attempt(deck, attempt)
    audit_pins = source_audit_pins(source)
    inputs = pins.collect_inputs(source)
    entries, metadata, _ = validate_source(deck)
    failures, failure_pins = review_rows(failed_ledger, entries, metadata)
    inputs.update(failure_pins)
    watched = pins.collect_runtime(entrypoints=pins.ENTRYPOINTS+(Path(__file__).name,))
    current = audit_base5_scenarios.audit(source)
    require(current.get('status') == 'OFFLINE_PASS' and not current.get('errors'),
            'Current complete source no longer passes offline audit')
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR, spatial.gen.geo.DEFAULT_MGEO), deck['config'])
    known = {entry['scenario_sha256'] for entry in deck['scenes']}
    replacements, changes = {}, []
    for ident in sorted(failures):
        before = metadata[ident]
        extent.validate_plan(before['spec'], deck['config'])
        scene, meta, proposal = first_candidate(before, deck['config'], ctx, known, maximum_proposals)
        extent.finish(spatial.gen, ctx, meta, deck['config'])
        for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                    'fresh_environment_provenance', 'static_mission_extent_preparation'):
            if key in before:
                meta[key] = copy.deepcopy(before[key])
        lineage = dict(policy=POLICY, attempt=attempt, source_id=ident,
            source_manifest=str(source), source_manifest_sha256=inputs[str(source)],
            original_scenario_sha256=entries[ident]['scenario_sha256'],
            original_metadata_sha256=inputs[str(Path(entries[ident]['metadata']).resolve())],
            failure_ledger=str(failed_ledger), failure_ledger_sha256=failure_pins[str(failed_ledger)],
            failed_capture_root=failures[ident]['capture_root'], failure_audit_sha256=failures[ident]['audit_sha256'],
            original_retry=before['retry'], frozen_spec_sha256=extent.digest(before['spec']),
            frozen_extent_plan=copy.deepcopy(before['spec'][extent.PLAN_KEY]),
            native_qa_unchanged=True, live_qualified=False, **proposal)
        meta['static_extent_visibility_recovery'] = lineage
        scenario, mpath = output/'scenarios'/(ident+'.json'), output/'scenarios'/(ident+'_metadata.json')
        atomic_json(scenario, scene)
        atomic_json(mpath, meta)
        require(sha256(scenario) == proposal['scenario_sha256'], 'Serialized candidate differs from proposed geometry')
        known.add(proposal['scenario_sha256'])
        updated = copy.deepcopy(entries[ident])
        updated.update(scenario=str(scenario), metadata=str(mpath), scenario_sha256=sha256(scenario))
        replacements[ident] = updated
        changes.append(dict(id=ident, **lineage))
    candidate = copy.deepcopy(deck)
    candidate['scenes'] = [copy.deepcopy(replacements.get(entry['id'], entry)) for entry in deck['scenes']]
    candidate['status'] = 'STATIC_EXTENT_VISIBILITY_RECOVERY_CPU_ONLY; finite native retry pending'
    candidate.setdefault('provenance', {})['static_extent_visibility_recovery'] = dict(policy=POLICY,
        attempt=attempt, maximum_attempts=MAX_ATTEMPTS, source_manifest=str(source),
        source_manifest_sha256=inputs[str(source)], failed_ledger=str(failed_ledger),
        failed_ledger_sha256=failure_pins[str(failed_ledger)])
    manifest = output/'manifest.json'
    atomic_json(manifest, candidate)
    subset = copy.deepcopy(candidate)
    subset.update(count=len(replacements), scenes=[copy.deepcopy(replacements[ident]) for ident in sorted(replacements)],
        status='REGISTERED_STATIC_EXTENT_VISIBILITY_RETRY_ONLY; no bulk capture')
    subset_path = output/'retry_subset'/'manifest.json'
    atomic_json(subset_path, subset)
    errors = []
    for path, expected in {**inputs, **audit_pins, **watched}.items():
        if sha256(path) != expected:
            errors.append([path, 'source/raw/runtime changed during recovery preparation'])
    before_counts, after_counts = Counter(), Counter()
    for old, new in zip(deck['scenes'], candidate['scenes']):
        require(old['id'] == new['id'], 'Recovery reordered source scene IDs')
        before, after = metadata[old['id']], read(new['metadata'])
        if old['id'] in replacements:
            verify_transition(before, after)
            extent.finish(spatial.gen, ctx, after, deck['config'])
        else:
            require(new == old, 'Recovery changed a nontarget entry')
        require(sha256(new['scenario']) == new['scenario_sha256'], 'Prepared payload checksum changed')
        if extent.eligible(before['spec']):
            key = lambda meta: (meta['spec'][extent.PLAN_KEY]['stratum'], meta['split'],
                meta['spec']['avoidance_phase'], meta['runtime_actors'][0]['data_id'])
            before_counts[key(before)] += 1
            after_counts[key(after)] += 1
    require(sum(before_counts.values()) == 600 and before_counts == after_counts,
            'Recovery changed complete 600-scene stratum/split/phase/asset quotas')
    audit = audit_base5_scenarios.audit(manifest)
    atomic_json(output/'scenario_audit.json', audit)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('errors'):
        errors.append(['full5000', 'complete scenario audit failed'])
    report = dict(schema='base5_static_extent_visibility_recovery_v1', policy=POLICY,
        passed=not errors, errors=errors, frames=5000, candidate_manifest_sha256=sha256(manifest),
        subset_manifest_sha256=sha256(subset_path), attempt=attempt, maximum_attempts=MAX_ATTEMPTS,
        changed_frames=len(replacements), unchanged_entry_paths_and_payloads=5000-len(replacements),
        preserved_static_mission_frames=600, same_frozen_specs=True, stratum_split_phase_asset_preserved=True,
        source_manifest=str(source), source_manifest_sha256=inputs[str(source)],
        source_files=inputs, source_audit_files=audit_pins, runtime_files=watched,
        changes=changes, bulk_started=False, live_qualified=False, native_qa_unchanged=True,
        limitation='First valid geometry in the same quota, not guaranteed new sensor evidence. Native retry remains bounded to 3 explicit generations.')
    atomic_json(output/'preservation_audit.json', report)
    if errors:
        raise RecoveryContractError('Recovery preservation/full-deck audit failed')
    return manifest, subset_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--failed-ledger', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--attempt', type=int, default=1)
    parser.add_argument('--maximum-proposals', type=int, default=MAX_PROPOSALS)
    args = parser.parse_args()
    full, subset = prepare(args.manifest, args.failed_ledger, args.output, args.attempt, args.maximum_proposals)
    print(json.dumps(dict(manifest=str(full), retry_manifest=str(subset))))
