#!/usr/bin/env python3
"""Read-only v122 geometry/preservation and preregistered native sensor validation."""
import argparse
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_shadow_observation import validate_overlay, validate_approach, visibility_review
from report_base5_shadow_validation import verified_entries


def report(manifest, source, ledgers=(), required=None):
    deck=gen.geo.load_json(manifest);old=gen.geo.load_json(source)
    originals={e['id']:e for e in old['scenes']};errors=[]
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    metas=[gen.geo.load_json(Path(e['metadata'])) for e in deck['scenes']]
    coverage=validate_overlay(deck,metas);geometry=[];preserved=0;preserved_moving=0;lanes=Counter();bands=Counter()
    for e,m in zip(deck['scenes'],metas):
        if m['spec'].get('shadow_observation_stage'):
            try:geometry.append(validate_approach(gen,ctx,m))
            except ValueError as exc:errors.append(e['id']+': '+str(exc))
            if m['runtime_actors']:
                lanes[str(m['spec']['shadow_primary_lane_offset_m'])]+=1
                bands[str(m['spec']['shadow_target_fraction_bounds'])]+=1
        else:
            if e!=originals[e['id']]:errors.append('Preserved source changed: '+e['id'])
            preserved+=1;preserved_moving+=int(bool(m['spec'].get('shadow_moving')))
    if (preserved,preserved_moving)!=(4524,24):errors.append('Preservation count mismatch')
    checked=verified_entries(deck,ledgers) if ledgers else {}
    expected={e['id'] for e in gen.geo.load_json(required)['scenes']} if required else set()
    missing=sorted(expected-set(checked))
    if missing:errors.append('Preregistered pilot missing: '+','.join(missing))
    live=[]
    for uid,row in checked.items():
        root=Path(row['capture_root']);meta=gen.geo.load_json(root/'metadata.json');audit=gen.geo.load_json(root/'audit.json')
        reasons,observation=visibility_review(meta,audit['components'])
        if reasons:errors.extend(uid+': '+r for r in reasons)
        if observation and audit.get('shadow_observation_visibility')!=observation:
            errors.append(uid+': sensor quota evidence differs from audit')
        geometry_result=validate_approach(gen,ctx,meta)
        live.append(dict(id=uid,root=str(root),observation=observation,geometry=geometry_result,
            classes=dict(Counter(c['training_class'] for c in audit['components'] if c['positive_lidar1_camera0'])),
            collision_verified=audit['capture_safety']['collision']['no_collision_verified']))
    outside=[m for m in metas if m['spec'].get('shadow_observation_stage','').startswith('approach') and m['runtime_actors']]
    static=[m for m in metas if m['spec'].get('shadow_observation_stage') and m['runtime_actors']]
    outside_live=[r['observation'] for r in live if r['observation'] and
                  r['observation']['stage'].startswith('approach') and not r['observation'].get('empty')]
    stages=Counter(p['stage'] for p in geometry)
    # Explicit quotas must be audited, not inferred from plan headers.
    stages['interior']+=24
    if stages!=Counter(deck['config']['shadow_observation']['stage_counts']):errors.append('Actual stage counts differ')
    if lanes!=Counter({'0.0':365,'3.5':91}):errors.append('Static primary lane quotas differ')
    return dict(status='PASS' if not errors else 'FAIL',errors=errors,
        planned=dict(coverage=coverage,preserved_entries=preserved,preserved_movers=preserved_moving,
            stage_counts=dict(stages),static_primary_lane_offsets=dict(lanes),static_primary_inside_fraction_bands=dict(bands),
            outside_primary_pools=dict(Counter(m['runtime_actors'][0]['population_pool'] for m in outside)),
            static_primary_inside_20_80_count=sum(.2<=m['shadow_approach_geometry']['actual_actor_inside_fractions'][0]<=.8 for m in static),
            staggered_scenes=sum(m['spec'].get('shadow_layout')=='staggered' for m in metas),
            outside_positive_scenes=sum(p['stage'].startswith('approach') and bool(p['actual_actor_inside_fractions']) for p in geometry)),
        live=dict(accepted=len(live),required=len(expected),missing=missing,evidence=live,
            outside_positive=len(outside_live),outside_forward=sum(v['forward_primary'] for v in outside_live),
            outside_front_camera=sum(v['front_camera_primary'] for v in outside_live)),
        manifest_sha256=sha256(manifest),source_sha256=sha256(source),
        limitations=['Placement/native GT QA, not detector or planning success.',
                    'Preserved v121 mover paths remain independently ground-prepared at capture; no new all-mover certification.',
                    'Right/left refer to supported route-relative lane strips, not official spawning probabilities.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--ledger',type=Path,action='append',default=[]);p.add_argument('--required-manifest',type=Path)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=report(a.manifest,a.source_manifest,a.ledger,a.required_manifest);atomic_json(a.output,r)
    print({k:v for k,v in r.items() if k not in ('planned','live')});print(r['planned'])
    print({k:v for k,v in r['live'].items() if k!='evidence'})
    if r['errors']:raise SystemExit(1)
