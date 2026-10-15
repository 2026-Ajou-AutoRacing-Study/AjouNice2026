#!/usr/bin/env python3
"""Freeze five fresh geometry environments and audit quotas, identity and XY diversity."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import math
import numpy as np
from scipy.spatial import cKDTree

from base5_capture_safety import atomic_json,sha256
from base5_region_contract import category,validate_quotas
from report_base5_final_coverage import report as coverage
from prepare_base5_fresh_environment import generate

TAGS=('sunny11','sunny15','foggy11','foggy13','foggy15')


def quota_key(meta):
    row=meta['spec'];actors=meta['runtime_actors']
    return json.dumps([category(row),bool(row['mission']),row['role'],row.get('distance_bounds'),
        actors[0]['data_id'] if actors else None,row.get('population_mode'),
        row.get('avoidance_phase'),row.get('vehicle_close_phase'),row.get('roundabout_phase'),
        row.get('dynamic_observation_phase'),row.get('crossing_direction_sign'),
        row.get('shadow_observation_stage'),meta['split']],sort_keys=True)


def diversity(metas,source_metas):
    result={}
    for family in ('obstacle','pedestrian_related','background','highway','roundabout','shadow'):
        rows=[m for m in metas if category(m['spec'])==family and
            (not m['spec']['mission'] or family=='roundabout')]
        old=[m for m in source_metas if category(m['spec'])==family and
            (not m['spec']['mission'] or family=='roundabout')]
        xy=np.asarray([[m['base']['x'],m['base']['y']] for m in rows]);oldxy=np.asarray([[m['base']['x'],m['base']['y']] for m in old])
        if not len(xy):continue
        rounded={tuple(v) for v in np.round(xy,3)};old_rounded={tuple(v) for v in np.round(oldxy,3)}
        nn=cKDTree(xy).query(xy,k=2)[0][:,1] if len(xy)>1 else [0.]
        result[family]=dict(frames=len(rows),old_unique_base_3dp=len(old_rounded),
            new_unique_base_3dp=len(rounded),old_base_overlap_3dp=len(rounded&old_rounded),
            nearest_xy_distance_p50_m=float(np.median(nn)),nearest_xy_distance_p95_m=float(np.quantile(nn,.95)),
            retry_window_levels=dict(Counter(m.get('continuous_ego_sampling',{}).get('retry_window_level','legacy_mission') for m in rows)))
    return result


def prepare(source,sunny13,output,seed=261010):
    if output.exists():raise FileExistsError('Fresh suite required')
    source_deck=json.loads(source.read_text());source_metas=[json.loads(Path(e['metadata']).read_text()) for e in source_deck['scenes']]
    expected=Counter(quota_key(m) for m in source_metas)
    output.mkdir(parents=True);catalog=[];all_payloads=set();groups={};tokens=set();proofs={};final_links=defaultdict(set)
    for tag in TAGS:
        path=generate(source,output/tag,tag,seed)
        deck=json.loads(path.read_text());metas=[json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']]
        if Counter(quota_key(m) for m in metas)!=expected:raise ValueError('Quota/primary/split drift '+tag)
        validate_quotas(metas)
        report=coverage(path);atomic_json(output/tag/'planned_coverage.json',report)
        if report['errors'] or report['primary_deficits']:raise ValueError('Primary coverage drift '+tag)
        for e,m in zip(deck['scenes'],metas):
            if e['scenario_sha256'] in all_payloads:raise ValueError('Duplicate payload between fresh environments')
            all_payloads.add(e['scenario_sha256']);group=m['scene_group_id'];token=tag+':'+group
            if token in tokens:raise ValueError('Duplicate token')
            tokens.add(token)
            if group in groups and groups[group]!=m['split']:raise ValueError('Group split conflict')
            groups[group]=m['split']
            if m['split_group'] in groups and groups[m['split_group']]!=m['split']:raise ValueError('Physical anchor link split conflict')
            groups[m['split_group']]=m['split']
            final_link=m.get('ego_ground_initializer',{}).get('center',{}).get('link_id')
            if final_link:final_links[final_link].add(m['split'])
        proofs[tag]=diversity(metas,source_metas)
        catalog.append(dict(environment=tag,weather=tag[:-2],hour=int(tag[-2:]),manifest=str(path.resolve()),
            manifest_sha256=sha256(path),capture_output=str((output.parent/('collection_5k_fresh_v141_'+tag)).resolve()),expected_frames=5000))
    catalog.append(dict(environment='sunny13',weather='sunny',hour=13,manifest=str(source.resolve()),
        manifest_sha256=sha256(source),capture_output=str(sunny13.resolve()),expected_frames=5000))
    atomic_json(output/'suite.json',dict(schema=2,total_expected_frames=30000,environments=catalog,
        same_scene_geometry_and_split=False,source_manifest=str(source.resolve()),source_manifest_sha256=sha256(source),
        dataset_warning='Sunny13 preserved; other environments have independent geometry seeds, same quotas. Same map/background and adjacent 100m views are not geographically independent.',
        bulk_started=False,live_qualified=False,operator_UI_confirmation_required=True))
    atomic_json(output/'suite_audit.json',dict(fresh_scenes=len(all_payloads),unique_tokens=len(tokens),
        source_primary_quota_preserved=True,logical_slot_split_preserved=True,
        physical_anchor_link_split_conflicts=0,diversity=proofs,
        final_support_links_shared_between_train_val=sorted(k for k,v in final_links.items() if len(v)>1),
        final_ROI_ground_policy='Native collector QA required; this is offline map screening',
        full_geographical_disjointness=False,bulk_started=False,live_qualified=False))
    return output/'suite.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--sunny13-collection',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=int,default=261010);a=p.parse_args();print(prepare(a.manifest,a.sunny13_collection,a.output,a.seed))
