#!/usr/bin/env python3
"""Freeze a native-qualified Sunny13 NPC-support release, never start capture."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
from pathlib import Path

from base5_capture_safety import atomic_json, load_ledger, sha256
from finalize_base5_obstacle_support_release import verify as verify_native
from report_base5_prebulk_cost import report as cost_report, unique_storage
import run_base5_capture_campaign as campaign
import run_base5_fresh_completion as completion


def verify_campaign(root, manifest, expected):
    status = verify_native(root, expected)
    with (root / '.batch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        selected, _, _ = completion.verify_parent(manifest, root)
    if len(selected) != status['accepted_candidate']:
        raise ValueError('Qualification count differs from native selection')
    return selected, status


def verify_repaired_cohort(registration, selected, manifest):
    repaired = set(registration['all_changed_slots'])
    by_id = {row['source_cell_id']: row for row in selected}
    source = {row['id']: row for row in json.loads(manifest.read_text())['scenes']}
    if not repaired or not repaired <= set(by_id):
        raise ValueError('Every repaired NPC slot must be natively approved')
    for ident in repaired:
        row = by_id[ident]
        if row['scenario_sha256'] != source[ident]['scenario_sha256']:
            raise ValueError('Repaired NPC was not tested at its final planned payload')
        runtime = row['audit'].get('stationary_vehicle_runtime', {}).get('actors', [])
        if not runtime or any(not actor['verified'] for actor in runtime):
            raise ValueError('Repaired NPC lacks actual stationary witnesses')
    return len(repaired)


def reverify_completion_attempt(attempt, original, deck, pools):
    observed = completion.verify(attempt, deck=deck, source=original, pools=pools)
    if observed != attempt['result']:
        raise ValueError('Native completion selection changed')
    return observed


def freeze(a):
    if a.output.exists():
        raise FileExistsError('New qualified release directory required')
    deck = json.loads(a.manifest.read_text())
    if deck['count'] != 5000 or not deck['config'].get('parked_npc_placement'):
        raise ValueError('Full offline NPC-support 5K plan required')
    audit = json.loads((a.manifest.parent / 'npc_preservation_audit.json').read_text())
    scenario = json.loads((a.manifest.parent / 'scenario_audit.json').read_text())
    if (audit['errors'] or audit['candidate_sha256'] != sha256(a.manifest) or
            scenario['status'] != 'OFFLINE_PASS' or scenario['scenes'] != 5000):
        raise ValueError('Final offline invariant audit failed')
    registration = json.loads((a.registration / 'registration.json').read_text())
    if registration['source_manifest_sha256'] != sha256(a.manifest):
        raise ValueError('Native registration belongs to another final plan')
    static_manifest = a.registration / 'static64/manifest.json'
    static_rows, static = verify_campaign(a.static, static_manifest, 64)
    moving_rows, moving = verify_campaign(a.moving, a.registration / 'animated8/manifest.json', 8)
    if len(moving_rows) != 8:
        raise ValueError('Moving regression must approve all eight slots')
    verify_native(a.bus_probe, 4)
    if len(load_ledger(a.bus_probe / 'selected_ledger.jsonl')) != 4:
        raise ValueError('Long bus proof incomplete')
    completed = json.loads((a.completion / 'status.json').read_text())
    state = json.loads((a.completion / 'state.json').read_text())
    if (completed['stage'] != 'complete' or completed['accepted_candidate'] != 64
            or completed['replenished'] < 1 or state['baseline_selected'] != static_rows):
        raise ValueError('Actual finite native completion proof required')
    index = {entry['id']: entry for entry in json.loads(static_manifest.read_text())['scenes']}
    context = None; pools = None
    for attempt in state['attempts']:
        if not attempt.get('result') or attempt['result']['status'] != 'accepted_candidate':
            continue
        if context is None:
            context = completion.spatial.gen.context(completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                                      completion.spatial.gen.geo.DEFAULT_MGEO)
            pools = completion.pool_map(context)
        original = json.loads(Path(index[attempt['id']]['metadata']).read_text())
        reverify_completion_attempt(attempt, original, json.loads(static_manifest.read_text()), pools)
    rows, missing = completion.merged(state)
    if missing or rows != load_ledger(a.completion / 'selected_ledger.jsonl'):
        raise ValueError('Final completion ledger not native-authoritative')
    repaired = verify_repaired_cohort(registration, rows, a.manifest)
    proof = json.loads(a.resume_proof.read_text())
    if not proof.get('passed') or proof.get('additional_native_captures') != 0:
        raise ValueError('Terminal resume recaptured or changed evidence')
    for path, digest in proof['ledger_sha256'].items():
        if sha256(path) != digest:
            raise ValueError('Terminal resume selection changed after verification')
    partial = json.loads(a.partial_resume.read_text())
    if not partial.get('passed') or partial.get('preserved_selected') != 16:
        raise ValueError('Partial resume preservation missing')
    cost = cost_report(a.manifest, [a.static, a.moving], detailed_groups=True)
    supplement = unique_storage(a.completion)
    cost['finite_completion_evidence'] = dict(root=str(a.completion),
        additional_elapsed_sec=completed['replenishment_elapsed_sec'], unique_storage=supplement,
        note='Native completion cost is also reported separately; staged parent QA already included in group weighting')
    names = set(campaign.contract(a.manifest, [0], 2)['code'])
    names.update(('run_base5_batch_campaign.py', 'run_base5_batch_optimized.py', 'run_base5_capture_optimized.py',
        'run_base5_fresh_batch.py', 'run_base5_fresh_completion.py', 'run_base5_capture_campaign_spatial.py',
        'base5_spatial_diversity.py', 'launch_base5_obstacle_support.py', 'prepare_base5_parked_npc_support.py',
        'audit_base5_parked_npc_plan.py'))
    scripts = Path(__file__).parent
    canonical = Path.home() / 'morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140/selected_ledger.jsonl'
    canonical_sha = sha256(canonical)
    if canonical_sha != '026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43':
        raise ValueError('Original Sunny13 changed')
    evidence = {key: str(getattr(a, key).resolve()) for key in
                ('registration', 'static', 'moving', 'completion', 'bus_probe', 'resume_proof', 'partial_resume')}
    doc = dict(schema='base5_obstacle_support_release_v1', generated_at_utc=datetime.now(timezone.utc).isoformat(),
        policy=deck['config']['obstacle_support_policy'], parked_npc_policy=deck['config']['parked_npc_placement'],
        qualified_environments=['sunny13'], completion_live_qualified=True, bulk_ready=True, bulk_blockers=[],
        runtime_hashes={str(scripts / name): sha256(scripts / name) for name in sorted(names)},
        environments={'sunny13': dict(manifest=str(a.manifest.resolve()), manifest_sha256=sha256(a.manifest),
            capture_output=str((a.output.parent / 'collection_5k_support_v150_sunny13').resolve()))},
        evidence=evidence, evidence_ledger_sha256={str(root / 'selected_ledger.jsonl'): sha256(root / 'selected_ledger.jsonl')
            for root in (a.static, a.moving, a.completion, a.bus_probe)},
        initial_static_accepted=static['accepted_candidate'], initial_moving_accepted=moving['accepted_candidate'],
        completed_static_accepted=64, repaired_slots_natively_approved=repaired,
        original_sunny13_ledger_sha256=canonical_sha, estimated_5k_hours=cost['extrapolated_hours_group_weighted'],
        estimated_5k_gib=cost['extrapolated_gib_including_qa_and_rejected_group_weighted'],
        operational_hours_range=[8, 12], operational_storage_budget_gib=45., bulk_started=False,
        limitation='Sunny13 small live qualification; advisory ground warnings retained. Other weather UI, all meshes and retrained AP are not certified.')
    atomic_json(a.output / 'cost_estimate.json', cost); atomic_json(a.output / 'release.json', doc)
    return a.output / 'release.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'registration', 'static', 'moving', 'completion', 'bus-probe',
                 'resume-proof', 'partial-resume', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args(); print(freeze(args))
