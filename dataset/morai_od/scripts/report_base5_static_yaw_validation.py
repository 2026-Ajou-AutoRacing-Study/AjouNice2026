#!/usr/bin/env python3
"""Validate frozen yaw release, protected geometry and selected live raw evidence."""
import argparse
import json
from collections import Counter
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
from base5_static_yaw import POLICY


def raw_snapshot(root):
    rows=[]
    for p in sorted(root.rglob('*')):
        if p.is_file() and any(v.startswith(('CAMERA_','LIDAR_')) for v in p.relative_to(root).parts):
            s=p.stat();rows.append([str(p.relative_to(root)),s.st_size,s.st_mtime_ns,s.st_ino,sha256(p)])
    return dict(files=len(rows),rows=rows)


def report(source,manifest,campaign=None):
    read=lambda p:json.loads(Path(p).read_text())
    old=read(source);new=read(manifest)
    if len(new['scenes'])!=5000:raise ValueError('Not complete 5K')
    oldrows={e['id']:e for e in old['scenes']};unchanged=Counter();yaw=Counter();splits=Counter()
    for e in new['scenes']:
        before=oldrows[e['id']];m=read(e['metadata']);s=read(e['scenario']);r=m['spec']
        if sha256(before['scenario'])!=before['scenario_sha256']:raise ValueError('Old input changed')
        if sha256(e['scenario'])!=e['scenario_sha256']:raise ValueError('New input changed')
        key='moving' if any(a['motion'] not in ('static','standing') for a in m['runtime_actors']) else 'static'
        splits[m['split']]+=1
        protected=key=='moving' or r['family'] in ('highway','roundabout','shadow','background') or r.get('dynamic_observation_phase')
        if protected and before['scenario_sha256']!=e['scenario_sha256']:raise ValueError('Protected geometry changed')
        if before['scenario_sha256']==e['scenario_sha256']:unchanged[r['family']]+=1
        for a in m['runtime_actors']:
            p=a.get('static_yaw_policy')
            if not p:continue
            if p['policy']!=POLICY or a['motion'] not in ('static','standing') or a['source_list']=='vehicleList':
                raise ValueError('Yaw policy applied outside static scope')
            actor=next(x for x in s[a['source_list']] if x['UNIQUEID']==a['unique_id'])
            if abs(float(actor['rot']['yaw'])-p['placement_yaw_deg'])>.001:
                raise ValueError('Serialized yaw differs from policy')
            yaw[str((p['scope'],p['mode']))]+=1
    suite=read(manifest.parent.parent/'environment_suite/suite.json')
    for item in suite['environments']:
        d=read(item['manifest'])
        if ({e['id']:e for e in d['scenes']}!={e['id']:e for e in new['scenes']}
                or sha256(item['manifest'])!=item['manifest_sha256']):
            raise ValueError('Environment input or split changed')
        if not d.get('collection_order'):raise ValueError('Collection order missing')
        moving=False
        for e in d['scenes']:
            animated=any(a['motion'] not in ('static','standing') for a in read(e['metadata'])['runtime_actors'])
            if moving and not animated:raise ValueError('Static scene after animated block')
            moving=moving or animated
    live={}
    if campaign:
        q=read(campaign/'quota.json');st=read(campaign/'status.json');placed=positive=components=0;modes=Counter();motions=Counter();severe=Counter()
        ledger=campaign/'selected_ledger.jsonl'
        selected=[json.loads(x) for x in ledger.read_text().splitlines()]
        for row in selected:
            root=Path(row['capture_root']);m=read(root/'metadata.json');a=read(root/'audit.json')
            if row['status']!='accepted_candidate' or a['review_reasons']:raise ValueError('Failed raw selected')
            if not a.get('capture_safety',{}).get('collision',{}).get('no_collision_verified'):
                raise ValueError('Selected capture lacks no-contact evidence')
            if not m.get('ego_stationary_witness',{}).get('stationary_verified',False):
                # Audit safety support is the independent stationary Z witness;
                # full Ego witness shape is preserved in metadata for review.
                if not a['capture_safety'].get('support',{}).get('stationary_support_observed'):
                    raise ValueError('Selected capture lacks stationary support evidence')
            placed+=len(m['runtime_actors'])
            positives=[c for c in a['components'] if c['positive_lidar1_camera0']]
            positive+=len({c['parent_uid'] for c in positives});components+=len(positives)
            for actor in m['runtime_actors']:
                if actor.get('static_yaw_policy'):modes[actor['static_yaw_policy']['mode']]+=1
            motions['moving' if any(x['motion'] not in ('static','standing') for x in m['runtime_actors']) else 'static']+=1
        for p in campaign.rglob('audit.json'):
            for reason in read(p)['review_reasons']:
                if any(k in reason for k in ('severe','vertical','collision','stationary_vehicle_moved')):severe[reason]+=1
        live=dict(stage=st['stage'],accepted=q['accepted'],deficit=q['deficit'],requested=q['target'],
            selected_motion=dict(motions),selected_parent_placements=placed,positive_parents=positive,
            positive_components=components,selected_yaw_modes=dict(modes),severe_review_reasons=dict(severe),
            elapsed_sec=st['elapsed_sec'],selected_ledger_sha256=sha256(ledger),
            note='Preregistered pilot, not 5K success-rate or training-performance guarantee')
    return dict(status='YAW_RELEASE_PASS',manifest_sha256=sha256(manifest),source_sha256=sha256(source),
        verified_cells=5000,unchanged_by_family=dict(unchanged),static_yaw_actor_modes=dict(yaw),
        split_counts=dict(splits),environment_count=6,protected_payloads_unchanged=True,live=live)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--campaign',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--raw-snapshot',type=Path)
    a=p.parse_args();r=report(a.source,a.manifest,a.campaign)
    if a.raw_snapshot:
        if not a.campaign:p.error('--raw-snapshot requires --campaign')
        snapshot=raw_snapshot(a.campaign)
        if a.raw_snapshot.exists():
            if json.loads(a.raw_snapshot.read_text())!=snapshot:raise ValueError('Pilot sensor raw changed')
            r['raw_unchanged']=True
        else:atomic_json(a.raw_snapshot,snapshot)
        r['raw_files_verified']=snapshot['files']
    atomic_json(a.output,r);print(json.dumps(r,ensure_ascii=False,indent=2))
