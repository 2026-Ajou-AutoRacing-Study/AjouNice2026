#!/usr/bin/env python3
"""Audit the explicitly revised 5K plan and convert selected native raw, without recapture."""
import argparse
from collections import Counter
import copy
from pathlib import Path
import pickle

import run_base5_flexible_completion as flex
from build_base5_bevfusion_infos import convert
from report_base5_final_coverage import report as coverage


def adopted_deck(source, rows):
    lookup = {e['id']: e for e in source['scenes']}
    used = set()
    entries = []
    for row in rows:
        ident = row['source_cell_id']
        if ident not in lookup or ident in used or row['status'] != 'accepted_candidate':
            raise ValueError('Unknown, duplicate or unapproved source slot')
        used.add(ident)
        root = Path(row['capture_root'])
        meta = flex.ac.fa.read(root/'metadata.json')
        old = flex.ac.fa.read(lookup[ident]['metadata'])
        if (meta['spec']['family'], bool(meta['spec']['mission']), meta['split']) != (
                old['spec']['family'], bool(old['spec']['mission']), old['split']):
            raise ValueError('Family, mission or split changed')
        if meta.get('scene_group_id', ident) != ident:
            raise ValueError('Selected scene group differs from logical slot')
        audit = flex.ac.fa.read(Path(row.get('audit_path', root/'audit.json')))
        if audit.get('review_reasons') or meta.get('calibration_only'):
            raise ValueError('Review/calibration cannot become training data')
        scenario = root/'scenario.json'
        if flex.ac.fa.sha256(scenario) != row['scenario_sha256']:
            raise ValueError('Selected native scenario checksum changed')
        entries.append(dict(id=ident, scenario=str(scenario), metadata=str(root/'metadata.json'),
            scenario_sha256=row['scenario_sha256'], split=meta['split'], scene_group_id=ident,
            family=meta['spec']['family'], mission=meta['spec']['mission'],
            distance_bounds=meta['spec'].get('distance_bounds', []), role=meta['spec'].get('role'),
            subfamily=meta['spec'].get('subfamily'), motion=meta['runtime_actors'][0]['motion'] if meta['runtime_actors'] else 'static',
            replacement_policy=row.get('flexible_completion')))
    return flex.ac.fa.batch.campaign.subset_deck(source, entries)


def execute(root, output):
    if flex.execute(root, output, resume=True):
        raise ValueError('Frozen flexible campaign verification failed')
    state = flex.ac.fa.read(output/'state.json')
    if state['stage'] != 'complete':
        raise ValueError('Final 5K approval requires complete, not partial quota')
    rows = flex.ac.fa.load_ledger(output/'selected_ledger.jsonl')
    source_path = root/'environment_suite/sunny13/manifest.json'
    source = flex.ac.fa.read(source_path)
    if len(rows) != len(source['scenes']) or len(rows) != 5000:
        raise ValueError('Exactly 5000 unique selections required')
    indexed = {r['source_cell_id']: r for r in rows}
    if not all(indexed.get(r['source_cell_id']) == r for r in state['baseline_selected']):
        raise ValueError('Original valid selection changed')
    deck = adopted_deck(source, rows)
    deck['status'] = 'LIVE_SELECTED; explicitly adopted flexible replacements, original plan preserved'
    deck['provenance'] = dict(policy=flex.POLICY, original_manifest=str(source_path),
        original_manifest_sha256=flex.ac.fa.sha256(source_path),
        selection_sha256=flex.ac.fa.sha256(output/'selected_ledger.jsonl'),
        original_valid_scenes=len(state['baseline_selected']), replacement_scenes=len(rows)-len(state['baseline_selected']),
        limitation='Adopted selection plan, not the unchanged original per-cell asset/distance quota')
    plan = output/'adopted_manifest.json'
    flex.ac.fa.atomic_json(plan, deck)
    result = coverage(plan, output/'selected_ledger.jsonl')
    flex.ac.fa.atomic_json(output/'coverage_final.json', result)
    if result['errors'] or result['unknown_ledger_sources'] or result['effective_coverage_status'] != 'PASS':
        raise ValueError('Adopted-plan coverage failed')
    conversion = convert(output/'selected_ledger.jsonl', output/'cpu_infos_final', flex.ac.fa.SENSOR,
                         environment_tag='sunny13')
    if conversion['excluded_review_scenes'] or conversion['excluded_failed_scenes']:
        raise ValueError('Selected scenes excluded during conversion')
    tokens=[];groups={};files=[];counts=Counter();split_counts={}
    for split in ('train','val','test'):
        data=pickle.load((output/f'cpu_infos_final/morai_base5_infos_{split}.pkl').open('rb'))['data_list']
        split_counts[split]=len(data)
        for info in data:
            tokens.append(info['token'])
            groups.setdefault(info['scene_group_id'],set()).add(split)
            files.append(info['lidar_points']['lidar_path'])
            files.extend(im['img_path'] for im in info['images'].values())
            counts.update(i['bbox_label_3d'] for i in info['instances'])
    if len(tokens)!=5000 or len(set(tokens))!=5000 or any(len(g)>1 for g in groups.values()):
        raise ValueError('Conversion token/group uniqueness failed')
    if len(files)!=25000 or not all(Path(p).is_file() for p in files):
        raise ValueError('Sensor input missing')
    proof=dict(stage='complete',selected=5000,deficit=0,baseline_preserved=True,
        baseline_scenes=len(state['baseline_selected']),new_scenes=5000-len(state['baseline_selected']),
        capture_attempts=len(state['attempts']),parent_hashes_unchanged=all(
            flex.ac.fa.sha256(Path(state['contract']['parent'])/n)==sha
            for n,sha in state['contract']['parent_hashes'].items()),
        source_sha256_unchanged=flex.ac.fa.sha256(source_path)==state['contract']['source_sha256'],
        adopted_manifest_sha256=flex.ac.fa.sha256(plan),selection_sha256=flex.ac.fa.sha256(output/'selected_ledger.jsonl'),
        coverage_status=result['effective_coverage_status'],expected_assets=result['expected_assets'],
        effective_primary_deficits=result['effective_primary_deficits'],splits=split_counts,gt=sum(counts.values()),
        conversion=conversion,input_files_checked=len(files),input_files_exist=True,
        unique_tokens=len(set(tokens)),group_split_overlap=0,
        limitations=['CPU infos use provisional crop/visibility; rebuild after final crop choice.',
            'Adopted replacements change original per-cell identities/distances, explicitly authorized and logged.',
            'Sensor existence and capture provenance checked, not a historical bitwise baseline of every image.',
            'Dataset QA completion is not detector accuracy or real-driving certification.'])
    flex.ac.fa.atomic_json(output/'validation_report.json',proof)
    return proof


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=flex.ac.DEFAULT_ROOT)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    proof=execute(a.root,a.output or a.root/'collection_5k_sunny13_flexible_v140')
    print({k:v for k,v in proof.items() if k not in ('conversion','limitations')})
