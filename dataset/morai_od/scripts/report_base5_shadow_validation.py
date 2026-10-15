#!/usr/bin/env python3
"""Read-only planned preservation and actual tunnel sensor/QA evidence."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path

import numpy as np
from PIL import Image

import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256
from base5_shadow_observation import validate_overlay
from build_base5_bevfusion_infos import in_roi
from base5_motion_ground import validate_motion_ground


def verified_entries(deck, ledgers):
    original={e['id']:e for e in deck['scenes']};result={}
    for ledger in ledgers:
        for line in Path(ledger).read_text().splitlines():
            entry=json.loads(line)
            if entry['status']!='accepted_candidate':continue
            key=entry.get('source_cell_id',entry['id'])
            if key not in original or key in result:raise ValueError('Unknown/duplicate accepted source cell')
            root=Path(entry['capture_root']);child=root.parent.parent
            scene=gen.geo.load_json(root/'scenario.json');meta=gen.geo.load_json(root/'metadata.json')
            audit=gen.geo.load_json(root/'audit.json');session=gen.geo.load_json(child/'session_contract.json')
            if sha256(root/'scenario.json')!=entry['scenario_sha256']:raise ValueError('Captured payload changed')
            if (not gen.geo.load_json(child/'shutdown_safety.json').get('park_verified') or
                audit['review_reasons'] or not audit['capture_safety']['collision'].get('no_collision_verified')):
                raise ValueError('Unsafe selected capture')
            if session.get('runner_options',{}).get('shared_ros_batch') and not entry.get('between_scene_park',{}).get('park_verified'):
                raise ValueError('Batch capture lacks boundary Park')
            if entry['scenario_sha256']!=original[key]['scenario_sha256']:
                # Ground-prepared paths are expected to differ from proposals.
                # Certify the stored independent proof, source spec/identities
                # and target XY; never silently accept an unrelated replacement.
                prior=gen.geo.load_json(Path(original[key]['metadata']))
                replacement=meta.get('replacement',{})
                same_cell_replacement=(replacement.get('source_scenario_sha256')==original[key]['scenario_sha256']
                    and replacement.get('source_cell_id')==key and meta['split']==prior['split'])
                if meta['spec']!=prior['spec']:
                    raise ValueError('Selection changed frozen source quota')
                if same_cell_replacement:
                    if meta['runtime_actors'][0]['data_id']!=prior['runtime_actors'][0]['data_id']:
                        raise ValueError('Replacement changed primary identity')
                    if any(a.get('motion_path') for a in meta['runtime_actors']):
                        validate_motion_ground(meta,scene)
                    result[key]=entry
                    continue
                if not any(a.get('motion_path') for a in meta['runtime_actors']):
                    raise ValueError('Selection is not a witnessed ground-prepared source cell')
                if len(meta['runtime_actors'])!=len(prior['runtime_actors']):
                    raise ValueError('Ground preparation changed actor count')
                for left,right in zip(prior['runtime_actors'],meta['runtime_actors']):
                    if (left['data_id'],left['unique_id'])!=(right['data_id'],right['unique_id']):
                        raise ValueError('Ground preparation changed actor identity')
                    for axis in ('x','y'):
                        if abs(left['motion_path']['target_position'][axis]-right['motion_path']['target_position'][axis])>1e-7:
                            raise ValueError('Ground preparation changed target XY')
                validate_motion_ground(meta,scene)
            elif any(a.get('motion_path') for a in meta['runtime_actors']):
                validate_motion_ground(meta,scene)
            result[key]=entry
    return result


def report(manifest,source,ledgers,required_manifests=()):
    deck=gen.geo.load_json(manifest);original=gen.geo.load_json(source)
    metas=[gen.geo.load_json(Path(e['metadata'])) for e in deck['scenes']]
    planned=validate_overlay(deck,metas)
    old={e['id']:e for e in original['scenes']};errors=[]
    unchanged=[e==old[e['id']] for e,m in zip(deck['scenes'],metas) if not m['spec'].get('shadow_phase')]
    if len(unchanged)!=4500 or not all(unchanged):errors.append('Non-shadow source cell changed')
    planned.update(preserved_source_entries=sum(unchanged),
        positive_distance_bins=dict(Counter(str(m['spec']['distance_bounds']) for m in metas
            if m['spec'].get('shadow_phase') and m['runtime_actors'])),
        actor_count_histogram=dict(Counter(len(m['runtime_actors']) for m in metas if m['spec'].get('shadow_phase'))),
        primary_pool_histogram=dict(Counter(m['runtime_actors'][0]['population_pool'] for m in metas
            if m['spec'].get('shadow_phase') and m['runtime_actors'])),
        protected_missions_preserved=sum(e==old[e['id']] for e in deck['scenes'] if e['mission']))
    checked=verified_entries(deck,ledgers) if ledgers else {}
    required={e.get('source_cell_id',e['id']) for path in required_manifests
              for e in gen.geo.load_json(path)['scenes']}
    missing=sorted(required-set(checked))
    if missing:errors.append('Required pilot cells not accepted: '+','.join(missing))
    actual=[];brightness=defaultdict(list);gt=Counter();ground=[];moving=[]
    for uid,entry in checked.items():
        root=Path(entry['capture_root']);meta=gen.geo.load_json(root/'metadata.json');audit=gen.geo.load_json(root/'audit.json')
        phase=meta['spec'].get('shadow_phase')
        if not phase:errors.append('Non-shadow capture in shadow ledger');continue
        if audit['review_reasons']:errors.append('Accepted capture has QA rejection')
        images=list(root.glob('CAMERA_1/*_Intensity.png'))
        with Image.open(images[0]) as image:
            rgb=np.asarray(image.convert('RGB'),dtype=float);h,w=rgb.shape[:2]
            luma=rgb[int(h*.35):int(h*.8),int(w*.2):int(w*.8)]@np.array([.2126,.7152,.0722])
            photometry=dict(road_roi_mean_luma=float(luma.mean()),road_roi_p50_luma=float(np.median(luma)),
                fraction_below_40=float((luma<40).mean()),image=str(images[0]))
        brightness[phase].append(photometry['road_roi_mean_luma'])
        positives=[c for c in audit['components'] if c['positive_lidar1_camera0'] and in_roi(c['center_lidar_m'])]
        gt.update(c['training_class'] for c in positives)
        for a in audit.get('ground_support',{}).get('actors',[]):
            gap=a.get('minimum_annotation_ground_gap_m')
            if gap is not None:ground.append(float(gap))
        for a in audit.get('motion_capture_actors',[]):
            if a.get('required'):moving.append(a)
        actual.append(dict(id=uid,phase=phase,photometry=photometry,positive_gt=len(positives),
            raw_components=len(audit['components']),camera_only_gt=sum(c['lidar_points']==0 for c in positives),
            asset_names=[a['name'] for a in meta['runtime_actors']],
            moving=any(a['motion'] not in ('static','standing') for a in meta['runtime_actors']),
            collision_verified=audit['capture_safety']['collision'].get('no_collision_verified'),
            park_verified=gen.geo.load_json(root.parent.parent/'shutdown_safety.json').get('park_verified'),
            motion_capture=audit.get('motion_capture_actors'),vertical_motion=audit.get('vertical_motion_actors')))
    return dict(status='PASS' if not errors else 'FAIL',errors=errors,planned=planned,
        live=dict(accepted_scenes=len(actual),required_scenes=len(required),missing_required_cells=missing,
            phase_counts=dict(Counter(a['phase'] for a in actual)),
            moving_scenes=sum(a['moving'] for a in actual),positive_gt=dict(gt),
            camera_road_roi_mean_luma_by_phase={p:dict(count=len(v),median=float(np.median(v)),
                min=float(min(v)),max=float(max(v))) for p,v in brightness.items()},
            measured_ground_gap_m=dict(count=len(ground),min=min(ground),max=max(ground)) if ground else {},
            evidence=actual),
        manifest_sha256=sha256(manifest),
        limitations=['Sensor/GT acquisition QA, not trained-model detection or driving success.',
            'Dark road pixel statistics do not certify each target has identical exposure.',
            'Static selections require exact payload or witnessed same-primary/quota/split replacement; movers require independent ground proof.',
            'Unmeasured/sparse static ground stays warning; not all-asset collider certification.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--ledger',type=Path,action='append',default=[]);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--required-manifest',type=Path,action='append',default=[],
                   help='Fail if any preregistered pilot source cell is absent from accepted ledgers')
    a=p.parse_args();result=report(a.manifest,a.source_manifest,a.ledger,a.required_manifest);atomic_json(a.output,result)
    print({k:v for k,v in result.items() if k!='live'});print({k:v for k,v in result['live'].items() if k!='evidence'})
    if result['errors']:raise SystemExit(1)
