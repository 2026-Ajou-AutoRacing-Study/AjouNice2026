#!/usr/bin/env python3
"""Publish a new immutable operational lookup only for profiles with live evidence."""
import argparse
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
from base5_intersection_ground import load_lookup


def report(lookup_file,sessions,output,prior_sessions=()):
    if output.exists():raise FileExistsError('New evidence directory required')
    lookup=load_lookup(lookup_file);rows=[];directions={};files=[];elapsed=0.;invalid=set()
    for session in sessions:
        shutdown=json.loads((session/'shutdown_safety.json').read_text())
        if not shutdown.get('park_verified'):raise ValueError('Unsafe session shutdown')
        status=json.loads((session/'status.json').read_text())
        if status['stage']!='complete':raise ValueError('Incomplete session')
        contract=json.loads((session/'session_contract.json').read_text())
        files.extend(dict(path=str(p.resolve()),sha256=sha256(p)) for p in
                     (session/'shutdown_safety.json',session/'ledger.jsonl',session/'session_contract.json',session/'status.json'))
        elapsed+=status['elapsed_sec']
        for line in (session/'ledger.jsonl').read_text().splitlines():
            row=json.loads(line)
            if row['status'] not in ('accepted_candidate','review'):
                raise ValueError('Execution failure cannot publish a lookup')
            root=Path(row['capture_root']);meta=json.loads((root/'metadata.json').read_text())
            audit=json.loads((root/'audit.json').read_text())
            if audit!=row['audit'] or sha256(root/'scenario.json')!=row['scenario_sha256']:
                raise ValueError('Raw capture evidence differs from ledger')
            actor=meta['runtime_actors'][0];proof=actor['motion_path']['ground_path_evidence']
            if proof['lookup_sha256']!=sha256(lookup_file):raise ValueError('Different manual lookup source')
            key=proof['profile_key'];spec=meta['manual_ground_pilot'];vertical=audit['vertical_motion_actors'][0]
            actual=audit['ground_support']['actors']
            gaps=[a.get('minimum_annotation_ground_gap_m') for a in actual if a.get('minimum_annotation_ground_gap_m') is not None]
            valid=(row['status']=='accepted_candidate' and audit['accepted_candidate'] and not audit['review_reasons']
                   and audit['capture_safety']['collision']['no_collision_verified'])
            rows.append(dict(id=row['id'],profile_key=key,direction=spec['direction'],status=row['status'],
                motion=actor['motion'],review_reasons=audit['review_reasons'],
                duration_sec=row['duration_sec'],height_error_m=vertical['maximum_height_error_m'],
                z_step_m=vertical['maximum_observed_z_step_m'],annotation_ground_gaps_m=gaps,
                ground_warnings=audit['ground_support']['manual_policy_decision']['warnings'],
                capture_root=str(root),accepted=valid))
            if valid:directions.setdefault(key,set()).add(spec['direction'])
            else:invalid.add(key)
            files.extend(dict(path=str(p.resolve()),sha256=sha256(p)) for p in
                (root/'metadata.json',root/'audit.json',root/'telemetry.json',root/'collision_history.json',root/'scenario.json'))
    prior=[]
    for session in prior_sessions:
        shutdown=json.loads((session/'shutdown_safety.json').read_text())
        if not shutdown.get('park_verified'):raise ValueError('Prior diagnostic shutdown unsafe')
        status=json.loads((session/'status.json').read_text())
        elapsed+=status['elapsed_sec']
        files.extend(dict(path=str(p.resolve()),sha256=sha256(p)) for p in
                     (session/'shutdown_safety.json',session/'ledger.jsonl',session/'session_contract.json',session/'status.json'))
        for line in (session/'ledger.jsonl').read_text().splitlines():
            item=json.loads(line)
            prior.append(dict(id=item['id'],session=str(session.resolve()),status=item['status'],
                capture_root=item.get('capture_root'),review_reasons=item.get('audit',{}).get('review_reasons',[]),
                used_for_profile_approval=False))
            if item.get('capture_root'):
                root=Path(item['capture_root'])
                files.extend(dict(path=str(p.resolve()),sha256=sha256(p)) for p in
                             (root/'metadata.json',root/'audit.json',root/'telemetry.json',root/'collision_history.json',root/'scenario.json'))
    approved=sorted(k for k,v in directions.items() if v=={-1,1} and k not in invalid)
    output.mkdir(parents=True)
    evidence=dict(status='COMPLETE',source_lookup_sha256=sha256(lookup_file),sessions=[str(p.resolve()) for p in sessions],
        captures=len(rows),accepted=sum(r['accepted'] for r in rows),rows=rows,validated_profiles=approved,
        prior_diagnostic_attempts=prior,total_actual_capture_attempts=len(rows)+len(prior),
        prior_review_captures=sum(r['status']=='review' for r in prior),
        unresolved_profiles=sorted(set(lookup['profiles'])-set(approved)),
        total_session_wall_sec=elapsed,wall_sec_per_capture=elapsed/(len(rows)+len(prior)) if rows or prior else None,
        maximum_path_height_error_m=max((r['height_error_m'] for r in rows if r['height_error_m'] is not None),default=None),
        files=files,terrain_reference_captures=0,static_reference_captures=0,height_recaptures=0,
        validation_scope='Asset-specific forward/reverse operational QA at the tested intersection path. Warning is not physical ground certification.')
    ep=output/'live_report.json';atomic_json(ep,evidence)
    published=copy.deepcopy(lookup)
    published.update(status='LIVE_VALIDATED',live_evidence=dict(path=str(ep.resolve()),sha256=sha256(ep),validated_profiles=approved),
                     validation_scope=evidence['validation_scope'])
    published['profiles']={k:v for k,v in lookup['profiles'].items() if k in approved}
    for profile in published['profiles'].values():
        profile['live_operational_qa_passed']=True
        if 'transfer_reference' in profile:profile['transfer_verified_within_live_scope']=True
    atomic_json(output/'height_lookup_live.json',published)
    return output/'height_lookup_live.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lookup',type=Path,required=True);p.add_argument('--sessions',type=Path,nargs='+',required=True)
    p.add_argument('--prior-sessions',type=Path,nargs='*',default=[],help='Preserved safe diagnostics, not profile approval evidence')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(report(a.lookup,a.sessions,a.output,a.prior_sessions))
