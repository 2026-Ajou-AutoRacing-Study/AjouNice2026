#!/usr/bin/env python3
"""Read-only planned vs accepted-visible coverage; never treats placed actors as GT."""
import argparse
from collections import Counter
import json
from pathlib import Path
import generate_base5_independent as gen
from base5_capture_safety import atomic_json
from build_base5_bevfusion_infos import in_roi


def report(manifest_path, ledger=None):
    manifest=gen.geo.load_json(Path(manifest_path));config=manifest['config']
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    expected={a['data_id']:a['name'] for key in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider') for a in ctx['pools'][key]}
    primary=Counter();all_actors=Counter();directions=Counter();phases=Counter();ids=set();errors=[];expected_primary={}
    for entry in manifest['scenes']:
        if entry['id'] in ids:errors.append('duplicate scene '+entry['id'])
        ids.add(entry['id'])
        if gen.geo.sha256(Path(entry['scenario']))!=entry['scenario_sha256']:errors.append('payload checksum '+entry['id'])
        meta=gen.geo.load_json(Path(entry['metadata']))
        actors=meta['runtime_actors'];row=meta['spec']
        if actors:
            primary[actors[0]['data_id']]+=1
            expected_primary[entry['id']]=actors[0]['data_id']
        for actor in actors:
            all_actors[actor['data_id']]+=1
            directions[str((row['family'],actor['actual_ego_role']))]+=1
        if row.get('vehicle_close_phase'):phases[str((row['family'],row['vehicle_close_phase']))]+=1
    floor=config.get('coverage_policy',{}).get('minimum_primary_scenes_per_asset',1)
    deficits={expected[k]:max(0,floor-primary[k]) for k in expected if primary[k]<floor}
    complete=len(ids)==config['target_scenes']
    if complete and deficits:errors.append('primary asset quota deficit')
    accepted=Counter();visible=Counter();seen=set();encountered=set();unknown_sources=[];actual_population=Counter()
    if ledger:
        for line in Path(ledger).read_text().splitlines():
            row=json.loads(line)
            if row['status']!='accepted_candidate':continue
            source=row.get('source_cell_id',row['id'])
            if source not in ids:unknown_sources.append(source);continue
            if source in encountered:continue
            encountered.add(source);root=Path(row['capture_root'])
            meta=gen.geo.load_json(root/'metadata.json')
            if meta.get('calibration_only'):continue
            audit=gen.geo.load_json(Path(row.get('audit_path',root/'audit.json')))
            if audit['review_reasons']:errors.append('accepted capture has review '+source);continue
            seen.add(source)
            actual_population[str((meta.get('population_budget',{}).get('mode','legacy'),
                len(meta['runtime_actors']),audit.get('population_visibility',{}).get('visible_positive_actors')))]+=1
            if not meta['runtime_actors']:continue
            actor=meta['runtime_actors'][0]
            if actor['data_id']!=expected_primary.get(source):
                errors.append('replacement primary differs from source quota '+source);continue
            accepted[actor['data_id']]+=1
            if any(c['parent_uid']==actor['unique_id'] and c['positive_lidar1_camera0'] and in_roi(c['center_lidar_m'],config['training_roi_candidate']) for c in audit['components']):
                visible[actor['data_id']]+=1
            else:errors.append('accepted primary has no positive ROI evidence '+source)
    effective_floor=config.get('coverage_policy',{}).get('minimum_accepted_visible_primary_per_asset',floor)
    effective_deficits={expected[k]:max(0,effective_floor-visible[k]) for k in expected if visible[k]<effective_floor}
    return dict(status='FAIL' if errors else 'PLANNED_PASS' if complete else 'PARTIAL_PLAN',
        requested_scenes=config['target_scenes'],manifest_scenes=len(ids),expected_assets=len(expected),
        primary_assets=len(primary),minimum_planned_primary=min((primary[k] for k in expected),default=0),
        primary_deficits=deficits,planned_actor_directions=dict(directions),vehicle_close_phases=dict(phases),
        accepted_source_scenes=len(seen),effective_coverage_status='NOT_CAPTURED' if not ledger else 'PARTIAL' if len(seen)<len(ids) else 'PASS' if not effective_deficits and not errors else 'FAIL',
        effective_primary_deficits=effective_deficits if ledger else None,unknown_ledger_sources=unknown_sources,
        actual_population_histogram=dict(actual_population),
        assets=[dict(data_id=k,name=expected[k],planned_primary=primary[k],planned_all_actors=all_actors[k],
                     accepted_primary=accepted[k],visible_positive_primary=visible[k]) for k in sorted(expected)],
        errors=errors,limitations=['Planned coverage is not effective sensor coverage or model accuracy.',
            'Four primary views is a coverage floor, not proof of sufficient asset learning.',
            'Secondary exposure does not replace primary quota. Raw components remain unchanged.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--ledger',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--require-effective-complete',action='store_true',help='Final collection gate, not a planned-manifest check')
    a=p.parse_args()
    result=report(a.manifest,a.ledger);atomic_json(a.output,result)
    print(json.dumps({k:v for k,v in result.items() if k!='assets'},ensure_ascii=False,indent=2))
    if result['errors']:raise SystemExit(1)
    if a.require_effective_complete and (result['effective_coverage_status']!='PASS' or result['unknown_ledger_sources']):raise SystemExit(1)
