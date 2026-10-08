#!/usr/bin/env python3
"""Versioned final release audit: quotas, real yaw, envelopes, weather and live evidence."""
import argparse
import json
import math
from collections import Counter,defaultdict
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
from base5_static_yaw import FINAL_POLICY,sample
from base5_placement_geometry import configure
from report_base5_static_yaw_validation import raw_snapshot
import generate_base5_independent as gen


def read(path):return json.loads(Path(path).read_text())


def release_status(live):
    if not live:return 'FINAL_APPEARANCE_OFFLINE_PASS'
    if (live['stage']!='complete' or live['deficit'] or live['accepted']!=live['requested']):
        return 'FINAL_APPEARANCE_LIVE_WITH_DEFICIT'
    return 'FINAL_APPEARANCE_RELEASE_PASS'


def report(source,manifest,campaign=None):
    old=read(source);deck=read(manifest);oldrows={e['id']:e for e in old['scenes']}
    if len(deck['scenes'])!=5000:raise ValueError('Exactly 5K required')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    assets={a['data_id']:a for pool in ctx['pools'].values() for a in pool}
    primary=Counter();oldprimary=Counter();population=Counter();exposure=Counter();motions=Counter();protected=Counter()
    modes=defaultdict(Counter);sectors=defaultdict(Counter);allsectors=Counter();directions=defaultdict(Counter)
    clearance=defaultdict(list);unknown=Counter();actual_scope=Counter();components=Counter();envelopes=Counter()
    for e in deck['scenes']:
        before=oldrows[e['id']];prior=read(before['metadata']);m=read(e['metadata']);r=m['spec'];s=read(e['scenario'])
        if sha256(e['scenario'])!=e['scenario_sha256'] or sha256(before['scenario'])!=before['scenario_sha256']:
            raise ValueError('Frozen payload changed')
        if prior['runtime_actors']:oldprimary[prior['runtime_actors'][0]['data_id']]+=1
        if m['runtime_actors']:primary[m['runtime_actors'][0]['data_id']]+=1
        protected_row=r['family'] in ('highway','roundabout','background') or r.get('dynamic_observation_phase')
        if protected_row:
            if before['scenario_sha256']!=e['scenario_sha256']:raise ValueError('Protected traffic/crossing/background changed')
            protected[r['family']]+=1
        moving=any(a['motion'] not in ('static','standing') for a in m['runtime_actors'])
        local=configure(ctx,deck['config'],r)
        motions['moving' if moving else 'static']+=1
        population[str((r['family'],r['mission']))]+=1
        for i,a in enumerate(m['runtime_actors']):
            if a['motion'] in ('static','standing') and a['name'] in local['geometry']:
                size,off,known=gen.geometry(assets[a['data_id']],local)
                if size!=a['estimated_size']:
                    raise ValueError('Stationary adapter missed measured geometry: '+a['name'])
            if a.get('static_placement_geometry_reference'):envelopes[a['name']]+=1
            exposure['obstacle' if a['population_pool'].startswith('obstacle_') else a['population_pool']]+=1
            policy=a.get('static_yaw_policy')
            if policy and policy['policy']==FINAL_POLICY:
                expected=sample(assets[a['data_id']],r,a['motion'],i,policy['reference_road_yaw_deg'],m['retry'])
                if policy!=expected:raise ValueError('Yaw metadata diverges from frozen sector/retry')
                raw=next(x for x in s[a['source_list']] if x['UNIQUEID']==a['unique_id'])
                raw=raw.get('initPosition',raw)
                if abs(float(raw['rot']['yaw'])-policy['placement_yaw_deg'])>.001:
                    raise ValueError('Serialized yaw diverges from metadata')
                actual_scope[policy['scope']]+=1
                if policy['mode']=='uniform':allsectors[policy['sector']]+=1
                if i==0:
                    key=str((policy['scope'],a['population_pool']));modes[key][policy['mode']]+=1
                    if policy['mode']=='uniform':sectors[key][policy['sector']]+=1
            if r.get('avoidance_phase')=='alongside' and i==0:
                clearance[a['population_pool']].append(a['planned_surface_clearance_m'])
                if not a['geometry_reference_known']:unknown[a['name']]+=1
                if a.get('static_placement_geometry_reference'):
                    local=configure(ctx,deck['config'],r);size,off,known=gen.geometry(assets[a['data_id']],local)
                    if not known or size!=a['estimated_size']:raise ValueError('Measured static envelope missing')
                    components[a['name']]+=1
        if r.get('shadow_moving'):
            direction=r['shadow_relative_motion'];a=m['runtime_actors'][0]
            rad=math.radians(a['placement_yaw_deg']);dx,dy=a['geometric_center_estimate'][0]-m['ego']['x'],a['geometric_center_estimate'][1]-m['ego']['y']
            dot=dx*math.cos(rad)+dy*math.sin(rad)
            if (dot<0)!=(direction=='approaching'):raise ValueError('Shadow actual movement sign differs')
            directions[r['motion']][direction]+=1
    if primary!=oldprimary:raise ValueError('Primary identity counts changed')
    if motions!=Counter(static=4822,moving=178):raise ValueError('Motion quota changed')
    for key,hist in sectors.items():
        counts=[hist[i] for i in range(12)]
        if max(counts)-min(counts)>1:raise ValueError('Primary free-sector quota imbalance '+key)
    if dict(directions)!=dict(walking=Counter(approaching=6,receding=6),running=Counter(approaching=6,receding=6)):
        raise ValueError('Shadow approach/recede walk/run quotas changed')
    suite=read(manifest.parent.parent/'environment_suite/suite.json')
    for env in suite['environments']:
        d=read(env['manifest'])
        if ({e['id']:e for e in d['scenes']}!={e['id']:e for e in deck['scenes']}
                or sha256(env['manifest'])!=env['manifest_sha256']):raise ValueError('Weather changed geometry/group/split')
        moving=False
        for e in d['scenes']:
            animated=any(a['motion'] not in ('static','standing') for a in read(e['metadata'])['runtime_actors'])
            if moving and not animated:raise ValueError('Stationary-first order broken')
            moving|=animated
    live={}
    if campaign:
        q=read(campaign/'quota.json');status=read(campaign/'status.json');gaps=defaultdict(list);positive=0;placed=0;parents=0
        selected=[json.loads(line) for line in (campaign/'selected_ledger.jsonl').read_text().splitlines()]
        for entry in selected:
            root=Path(entry['capture_root']);m=read(root/'metadata.json');audit=read(root/'audit.json')
            if entry['status']!='accepted_candidate' or audit['review_reasons']:
                raise ValueError('Unapproved raw selected')
            if not audit.get('capture_safety',{}).get('collision',{}).get('no_collision_verified'):
                raise ValueError('No collision evidence')
            placed+=len(m['runtime_actors']);cs=[c for c in audit['components'] if c['positive_lidar1_camera0']]
            positive+=len(cs);parents+=len({c['parent_uid'] for c in cs})
            if m['spec'].get('avoidance_phase')=='alongside':
                uid=m['runtime_actors'][0]['unique_id'];pool=m['runtime_actors'][0]['population_pool']
                gaps[pool].extend(c['annotation_surface_gap_m'] for c in audit.get('actual_annotation_clearances',[]) if c['parent_uid']==uid)
        shutdown=[read(p)['park_verified'] for p in campaign.rglob('shutdown_safety.json')]
        if not shutdown or not all(shutdown):raise ValueError('Pilot final Park unverified')
        live=dict(stage=status['stage'],requested=q['target'],accepted=q['accepted'],deficit=q['deficit'],
            elapsed_sec=status['elapsed_sec'],placed_parents=placed,visible_positive_parents=parents,positive_components=positive,
            annotation_clearance_m={k:dict(min=min(v),max=max(v),count=len(v)) for k,v in gaps.items()},
            shutdown_sessions=len(shutdown),park_verified=all(shutdown),
            limitation='Representative pilot; not 5K success rate, all-asset collider validation or learned-model accuracy')
    return dict(status=release_status(live),manifest_sha256=sha256(manifest),
        protected_payload_counts=dict(protected),motion_counts=dict(motions),family_mission_counts=dict(population),
        planned_parent_exposure_5k=dict(exposure),planned_parent_exposure_30k={k:v*6 for k,v in exposure.items()},
        planned_parent_total=sum(exposure.values()),primary_assets=len(primary),primary_floor=min(primary.values()),
        primary_modes={k:dict(v) for k,v in modes.items()},primary_sectors={k:dict(v) for k,v in sectors.items()},
        static_yaw_actor_scopes=dict(actual_scope),all_free_actor_sectors=dict(allsectors),
        shadow_relative_motion={k:dict(v) for k,v in directions.items()},
        planned_near_clearance_m={k:dict(min=min(v),max=max(v),count=len(v)) for k,v in clearance.items()},
        unknown_near_assets=dict(unknown),measured_parent_envelope_near_cells=sum(components.values()),
        measured_parent_envelope_actor_counts=dict(envelopes),
        environment_count=6,live=live,bulk_started=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--campaign',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--raw-snapshot',type=Path)
    a=p.parse_args();r=report(a.source,a.manifest,a.campaign)
    if a.raw_snapshot:
        if not a.campaign:raise ValueError('Raw snapshot needs campaign')
        snapshot=raw_snapshot(a.campaign)
        if a.raw_snapshot.exists():
            if read(a.raw_snapshot)!=snapshot:raise ValueError('Pilot sensor raw changed')
            r['raw_snapshot_unchanged']=True
        else:atomic_json(a.raw_snapshot,snapshot)
        r['sensor_files']=snapshot['files']
    atomic_json(a.output,r)
    print(json.dumps({k:v for k,v in r.items() if k not in
        ('primary_modes','primary_sectors','all_free_actor_sectors','measured_parent_envelope_actor_counts')},ensure_ascii=False))
