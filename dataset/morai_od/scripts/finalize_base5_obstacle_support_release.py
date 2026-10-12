#!/usr/bin/env python3
"""Freeze Sunny13-qualified generator/runtime evidence, never start collection."""
import argparse
import json
from pathlib import Path
from datetime import datetime,timezone

from base5_capture_safety import atomic_json,load_ledger,sha256
from report_base5_prebulk_cost import report as cost_report
import run_base5_capture_campaign as campaign


def verify(root,expected):
    status=json.loads((root/'status.json').read_text())
    if status['stage'] not in ('complete','complete_with_deficit') or status['requested']!=expected:
        raise ValueError('Completed bounded qualification required '+str(root))
    for path in root.rglob('session_contract.json'):
        shutdown=path.parent/'shutdown_safety.json'
        if not shutdown.is_file() or not json.loads(shutdown.read_text()).get('park_verified'):
            raise ValueError('Qualification contains unsafe/interrupted native session '+str(path.parent))
    selected=load_ledger(root/'selected_ledger.jsonl')
    for row in selected:
        raw=Path(row['capture_root'])
        if row['status']!='accepted_candidate' or row.get('audit',{}).get('review_reasons'):
            raise ValueError('Rejected row in qualification selection')
        if sha256(raw/'scenario.json')!=row['scenario_sha256']:raise ValueError('Selected scenario changed')
        safety=json.loads((raw/'collision_history.json').read_text())['summary']
        if not safety.get('no_collision_verified') or safety.get('non_ego_uids'):
            raise ValueError('Selected collision evidence not verified')
    return status


def freeze(a):
    if a.output.exists():raise FileExistsError('New release directory required')
    static=verify(a.static,64);moving=verify(a.moving,8)
    completion=json.loads((a.completion/'status.json').read_text())
    if completion['stage'] not in ('complete','complete_with_deficit') or completion['requested']!=64:
        raise ValueError('Actual bounded native completion pending')
    if completion['replenished']<1:raise ValueError('Actual successful native completion proof required')
    journal=json.loads((a.completion/'state.json').read_text())
    if not journal.get('baseline_selected') or not journal.get('deficits') or not journal.get('attempts'):
        raise ValueError('Actual deficit and newly captured completion evidence required')
    if not a.resume_proof.is_file():raise ValueError('Completed resume proof pending')
    resume=json.loads(a.resume_proof.read_text())
    if not resume.get('passed') or resume.get('additional_native_captures')!=0:
        raise ValueError('Completed resume changed raw or selection')
    for row in load_ledger(a.completion/'selected_ledger.jsonl'):
        if row.get('audit',{}).get('review_reasons'):raise ValueError('Completion waived native QA')
    cost=cost_report(a.manifest,[a.static,a.moving],detailed_groups=True)
    source=json.loads(a.manifest.read_text())
    preservation=json.loads((a.manifest.parent/'preservation_audit.json').read_text())
    if preservation['errors'] or preservation['frames']!=5000:raise ValueError('Offline preservation failed')
    names=set(campaign.contract(a.manifest,[0],2)['code'])
    names.update(('run_base5_batch_campaign.py','run_base5_batch_optimized.py','run_base5_capture_optimized.py',
        'run_base5_fresh_batch.py','run_base5_fresh_completion.py','run_base5_capture_campaign_spatial.py',
        'base5_spatial_diversity.py','launch_base5_obstacle_support.py',
        'prepare_base5_obstacle_support.py','prepare_base5_fresh_environment.py','base5_population_budget.py'))
    scripts=Path(__file__).parent
    native_ledger=Path('/home/luv/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140/selected_ledger.jsonl')
    original_sha=sha256(native_ledger)
    if original_sha!='026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43':
        raise ValueError('Completed Sunny13 canonical selection changed')
    doc=dict(schema='base5_obstacle_support_release_v1',generated_at_utc=datetime.now(timezone.utc).isoformat(),
        policy=source['config']['obstacle_support_policy'],qualified_environments=['sunny13'],
        completion_live_qualified=True,bulk_ready=False,
        bulk_blockers=['Long parked NPC in shadow: road-aligned 4-scene probe pending after simulator Vulkan SIGSEGV.',
                       'Reconnect simulator and verify fresh topics/Park; halted comparison must not be force-resumed.'],
        runtime_hashes={str(scripts/name):sha256(scripts/name) for name in sorted(names)},
        environments={'sunny13':dict(manifest=str(a.manifest.resolve()),manifest_sha256=sha256(a.manifest),
            capture_output=str((a.output.parent/'collection_5k_support_v149_sunny13').resolve()))},
        evidence=dict(static=str(a.static),moving=str(a.moving),completion=str(a.completion),resume_proof=str(a.resume_proof)),
        initial_static_accepted=static['accepted_candidate'],initial_moving_accepted=moving['accepted_candidate'],
        completed_static_accepted=completion['accepted_candidate'],remaining_pilot_deficit=completion['review'],
        original_sunny13_ledger_sha256=original_sha,
        estimated_5k_hours=cost['extrapolated_hours_group_weighted'],estimated_5k_gib=cost['extrapolated_gib_including_qa_and_rejected_group_weighted'],
        operational_hours_range=[10,13],operational_storage_budget_gib=max(40.,(cost['extrapolated_gib_including_qa_and_rejected_group_weighted'] or 40.)*1.25),bulk_started=False,
        limitation='Sunny13 live qualification only; other five weather/time plans remain offline. New model AP and complete ground contact are not certified.')
    atomic_json(a.output/'cost_estimate.json',cost)
    atomic_json(a.output/'release.json',doc)
    return a.output/'release.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('manifest','static','moving','completion','resume-proof','output'):
        p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();print(freeze(a))
