#!/usr/bin/env python3
"""Prepare two counterfactual frozen-scale completion layouts, never capture.

This does not claim that either source failed native QA. Deterministic source
selection precedes geometry ranking, reads no capture ledger, and cannot fall
back to another actor/asset/seed/profile when a bounded proposal search fails.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import audit_base5_scenarios
import base5_asset_scale as scale
import base5_front_release_inputs as pins
import base5_scale_completion_adapter as adapter
import base5_spatial_diversity as spatial
import prepare_base5_front_support_qualification as qualification
import run_base5_fresh_completion as fresh
from base5_capture_safety import atomic_json, sha256
from dynamic_capture_metadata import normalized_sensor_calibration
from prepare_base5_scale_extent_smoke import source_checks
from run_base5_failure_aware_replenishment_fast import prewarm


POLICY = 'counterfactual-frozen-scale-completion-probe-v155'
SCHEMA = 'base5_scale_completion_probe_registration_v1'
ORDINAL = 1
SLOTS = ('scaled_primary', 'scaled_auxiliary_multi')


class ProbeContractError(RuntimeError):
    """A provenance or declared two-slot contract error, not a native deficit."""


def read(path):
    return json.loads(Path(path).read_text())


def payload_sha(scene):
    return hashlib.sha256((json.dumps(scene, ensure_ascii=False, indent=2) + '\n').encode()).hexdigest()


def order_key(slot, ident, ordinal):
    return scale.digest([POLICY, slot, ident, ordinal]), ident, ordinal


def scaled_actors(meta):
    found = []
    for ordinal, actor in enumerate(meta.get('runtime_actors', [])):
        assignment = actor.get('asset_scale')
        if not assignment or assignment.get('profile_id') == scale.BASE:
            continue
        if (actor.get('motion') != 'static' or assignment.get('ordinal') != ordinal
                or assignment.get('candidate_only') is not False
                or assignment.get('profile_status') != 'qualified'
                or assignment.get('name') != actor.get('name')
                or assignment.get('data_id') != actor.get('data_id')):
            raise ProbeContractError('Probe source has an unqualified/foreign/moving scale actor')
        found.append(dict(ordinal=ordinal, name=actor['name'], data_id=actor['data_id'],
                          profile_id=assignment['profile_id']))
    return found


def select(metadata):
    """One scaled static-mission primary and a distinct-asset generic auxiliary.

    The complete frozen metadata is the only input. Source IDs are selected
    once; later geometry success/failure never selects a different source.
    """
    candidates = {slot: [] for slot in SLOTS}
    for ident, meta in metadata.items():
        row, actors = meta['spec'], meta.get('runtime_actors', [])
        if any(actor.get('motion') not in ('static', 'standing') for actor in actors):
            continue
        variants = scaled_actors(meta)
        if not variants or scale.PLAN_KEY not in row:
            continue
        for actor in variants:
            if (actor['ordinal'] == 0 and row.get('family') == 'obstacle'
                    and row.get('mission') is True and row.get('static_mission_extent')):
                candidates[SLOTS[0]].append(dict(id=ident, slot=SLOTS[0], **actor))
            if (actor['ordinal'] > 0 and len(actors) > 1 and row.get('mission') is False
                    and row.get('family') != 'shadow'):
                candidates[SLOTS[1]].append(dict(id=ident, slot=SLOTS[1], **actor))
    if not candidates[SLOTS[0]]:
        raise ProbeContractError('Missing qualified scaled static-mission primary probe cell')
    first = min(candidates[SLOTS[0]], key=lambda item: order_key(item['slot'], item['id'], item['ordinal']))
    auxiliaries = [item for item in candidates[SLOTS[1]]
                   if item['id'] != first['id'] and item['data_id'] != first['data_id']]
    if not auxiliaries:
        raise ProbeContractError('Missing distinct-asset qualified scaled auxiliary/multi probe cell')
    second = min(auxiliaries, key=lambda item: order_key(item['slot'], item['id'], item['ordinal']))
    return [first, second]


def propose(entry, before, config, ctx, calibration, known_shas, slot):
    """Use the production scale-only adapter and preserve its ordinal/budget."""
    scene, actual = adapter.choose(before, config, ctx, calibration, ORDINAL, entry['id'],
        known_shas, spatial.build, fresh.frozen_proposal_score, fresh.adaptive.fa.novelty)
    adapter.verify_preserved(before, actual, config)
    proof = actual.get('flexible_completion', {})
    if (proof.get('attempt') != ORDINAL or proof.get('maximum_proposals') != adapter.MAX_PROPOSALS
            or proof.get('ranked_candidate_cap') != adapter.RANKED_CANDIDATES
            or not 1 <= proof.get('proposals_tried', 0) <= adapter.MAX_PROPOSALS
            or not 1 <= proof.get('valid_candidates', 0) <= adapter.RANKED_CANDIDATES
            or proof.get('actor_population_changed') is not False
            or proof.get('scale_profile_fallback') is not False
            or payload_sha(scene) in known_shas):
        raise ProbeContractError('Completion probe escaped the frozen finite production adapter')
    for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                'fresh_environment_provenance'):
        if key in before:
            actual[key] = copy.deepcopy(before[key])
    actual['scale_completion_probe'] = dict(policy=POLICY, slot=slot['slot'], source_id=entry['id'],
        source_scenario_sha256=entry['scenario_sha256'], source_spec_sha256=scale.digest(before['spec']),
        focus_actor=copy.deepcopy(slot), source_native_failure=False,
        counterfactual_recovery_probe_only=True, native_outcomes_used_for_selection=False,
        stored_completion_ordinal=ORDINAL, source_retry=before['retry'],
        native_qa_unchanged=True, training_gt_modified=False, live_qualified=False)
    return scene, actual


def verify_boundaries(inputs, checks, runtime, policy):
    for mapping in (inputs, checks, runtime):
        pins.verify_hashes(mapping)
    scale.verify_frozen_policy(policy)


def prepare(manifest, output):
    manifest, output = Path(manifest).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh counterfactual probe output required')
    runtime_dir = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if (output == manifest.parent or manifest.parent in output.parents
            or output == runtime_dir or runtime_dir in output.parents):
        raise ProbeContractError('Offline probe cannot modify source or MORAI runtime')
    checks = source_checks(manifest)
    inputs = pins.collect_inputs(manifest)
    deck = read(manifest)
    entries, metadata, _ = qualification.validate_source(deck)
    config = scale.bind_config(copy.deepcopy(deck['config']))
    policy = config.get(scale.CONFIG_KEY)
    if type(policy) is not scale.FrozenPolicy:
        raise ProbeContractError('An immutable qualified scale policy in a full 5K is required')
    catalog = {asset['data_id']: asset for asset in read(spatial.gen.catalog.DEFAULT_CATALOG)['assets']}
    for ident, meta in metadata.items():
        scale.verify_metadata(read(entries[ident]['scenario']), meta, config, catalog)
    selected = select(metadata)
    audit = audit_base5_scenarios.audit(manifest)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('errors'):
        raise ProbeContractError('Current full source scenario audit failed')
    runtime = pins.collect_runtime(entrypoints=pins.ENTRYPOINTS + (Path(__file__).name,))
    sensor = Path(config['secondary_far_vehicle_policy']['proposal_visibility']['sensor']['path']).resolve()
    if inputs.get(str(sensor)) != sha256(sensor):
        raise ProbeContractError('Probe calibration must be one of the frozen full-source inputs')
    calibration = normalized_sensor_calibration(sensor)
    verify_boundaries(inputs, checks, runtime, policy)
    output.mkdir(parents=True)
    preregistration = dict(schema=SCHEMA, policy=POLICY, source_manifest=str(manifest),
        source_manifest_sha256=inputs[str(manifest)], source_native_failure=False,
        counterfactual_recovery_probe_only=True, native_outcomes_used_for_selection=False,
        stored_completion_ordinal=ORDINAL, maximum_proposals=adapter.MAX_PROPOSALS,
        ranked_candidate_cap=adapter.RANKED_CANDIDATES, selected=selected,
        selection='Frozen metadata deterministic ID hash, two distinct assets; no capture ledger is read',
        source_input_hashes=inputs, source_audit_hashes=checks, runtime_hashes=runtime,
        scale_policy_sha256=scale.digest(policy), native_qa_unchanged=True,
        bulk_started=False, training_started=False, live_qualified=False)
    # Persist the two selected source cells before their proposals are built.
    atomic_json(output / 'selection.json', preregistration)
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                     spatial.gen.geo.DEFAULT_MGEO), config)
    known = {entry['scenario_sha256'] for entry in deck['scenes']}
    candidates, records = [], []
    for slot in selected:
        ident, before = slot['id'], metadata[slot['id']]
        try:
            scene, actual = propose(entries[ident], before, config, ctx, calibration, known, slot)
        except (ValueError, RuntimeError) as error:
            atomic_json(output / 'preparation_failure.json', dict(source_id=ident, slot=slot['slot'],
                error_type=type(error).__name__, reason=str(error), native_failure=False,
                counterfactual_recovery_probe_only=True, selected_sources_not_replaced=True,
                native_qa_unchanged=True, bulk_started=False))
            raise
        sid = ident + '_scale_probe01'
        scenario = output / 'scenarios' / (sid + '.json')
        mpath = output / 'scenarios' / (sid + '_metadata.json')
        atomic_json(scenario, scene)
        atomic_json(mpath, actual)
        if sha256(scenario) != payload_sha(scene):
            raise ProbeContractError('Serialized proposal changed from the chosen payload')
        known.add(sha256(scenario))
        entry = copy.deepcopy(entries[ident])
        entry.update(id=sid, source_cell_id=ident, scenario=str(scenario), metadata=str(mpath),
                     scenario_sha256=sha256(scenario))
        candidates.append(entry)
        records.append(dict(**slot, source_scenario_sha256=entries[ident]['scenario_sha256'],
            scenario_sha256=entry['scenario_sha256'], metadata_sha256=sha256(mpath),
            generation_retry=actual['retry'], flexible_completion=actual['flexible_completion'],
            frozen_spec_sha256=scale.digest(actual['spec']), source_native_failure=False,
            counterfactual_recovery_probe_only=True))
        print('scale-completion-probe', len(candidates), '/2', ident, 'retry', actual['retry'], flush=True)
    subset = copy.deepcopy(deck)
    subset.update(count=2, scenes=candidates,
        status='COUNTERFACTUAL_SCALE_COMPLETION_PROBE_ONLY; no source native failure claimed; no bulk or training')
    target = output / 'manifest.json'
    verify_boundaries(inputs, checks, runtime, policy)
    atomic_json(target, subset)
    for entry, slot in zip(candidates, selected):
        actual = read(entry['metadata'])
        adapter.verify_preserved(metadata[slot['id']], actual, config)
        scale.verify_metadata(read(entry['scenario']), actual, config, catalog)
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ProbeContractError('Prepared probe payload checksum changed')
    verify_boundaries(inputs, checks, runtime, policy)
    atomic_json(output / 'source_audit.json', audit)
    atomic_json(output / 'registration.json', dict(preregistration, passed=True,
        subset_manifest_sha256=sha256(target), selection_sha256=sha256(output / 'selection.json'),
        frames=2, records=records, training_gt_modified=False,
        limitation='A newly captured proposal can validate the same-contract geometry path, not ordinary-failure handling or AP'))
    verify_boundaries(inputs, checks, runtime, policy)
    return target


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
