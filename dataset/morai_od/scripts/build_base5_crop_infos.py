#!/usr/bin/env python3
"""Versioned crop visibility / infos, preserve canonical ledger and raw 3D GT.

Uses full instance-mask measurements bound by canonical ledger SHA. Counts and
extents use the current native policy (>=16 mask pixels, >=4px minimum extent).
No camera support is invented for ambiguous composite components.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from build_base5_bevfusion_infos import convert,in_roi


def update_component(component,measurement,profile):
    new=deepcopy(component)
    # Ambiguous components never obtain a synthetic per-component camera match.
    visible=False if component.get('camera_component_ambiguous') else bool(
        measurement and measurement['candidates'][profile]['camera_gate'])
    new['camera_visible_candidate']=visible
    new['positive_lidar1_camera0']=new['lidar_points']>=1 or visible
    new['ignore_reason']=None if new['positive_lidar1_camera0'] else 'no_visible_sensor_evidence'
    return new


def build(ledger,analysis,output,sensor,profile):
    if output.exists():raise ValueError('Never overwrite previous infos/evidence')
    payload=ledger.read_bytes();report=json.loads((analysis/'report.json').read_text())
    digest=hashlib.sha256(payload).hexdigest()
    if not report['full_snapshot'] or report['source_sha256']!=digest or report['accepted_scenes']!=5000:
        raise ValueError('Full canonical 5K analysis/source mismatch')
    if profile not in {p['id'] for p in report['profiles']}:raise ValueError('Unknown profile')
    if profile not in ('bottom320','bottom384','bottom448','bottom512w1056'):
        raise ValueError('Only runtime-verified common bottom-crop profiles are supported')
    measurements=json.loads((analysis/'component_unions.json').read_text())
    lookup={(m['scene'],m['component_id']):m for m in measurements}
    entries=[json.loads(line) for line in payload.splitlines()]
    output.mkdir(parents=True);audits=output/'derived_audits';audits.mkdir()
    rows=[];gained=Counter();lost=Counter();changes=[]
    for entry in entries:
        row=deepcopy(entry);root=Path(row['capture_root'])
        original=json.loads(Path(row.get('audit_path',root/'audit.json')).read_text())
        updated=deepcopy(original)
        updated['components']=[]
        for c in original['components']:
            m=lookup.get((row['id'],c['component_id']))
            measurable=any(e['native_instance_pixels']>0 and e['native_rect'][3]>e['native_rect'][1]
                           and e['native_rect'][2]>e['native_rect'][0] for e in c['cameras'])
            if measurable and not c.get('camera_component_ambiguous') and m is None:
                raise ValueError('Missing measured camera component: '+row['id']+'/'+c['component_id'])
            n=update_component(c,m,profile);updated['components'].append(n)
            if in_roi(c['center_lidar_m']) and c['positive_lidar1_camera0']!=n['positive_lidar1_camera0']:
                (gained if n['positive_lidar1_camera0'] else lost)[c['training_class']]+=1
                changes.append(dict(scene=row['id'],component=c['component_id'],asset=c['asset'],
                                    gained=n['positive_lidar1_camera0']))
            assert all(c[k]==n[k] for k in ('center_lidar_m','size_m','rotation_rpy','lidar_points'))
        updated['derived_visibility']=dict(profile=profile,source_ledger_sha256=digest,
            raw_preserved=True,original_audit_sha256=hashlib.sha256(
                Path(row.get('audit_path',root/'audit.json')).read_bytes()).hexdigest())
        target=audits/(hashlib.sha256(row['id'].encode()).hexdigest()+'.json')
        target.write_text(json.dumps(updated,separators=(',',':'))+'\n')
        row['audit_path']=str(target.resolve());row['audit']=updated;rows.append(row)
    derived=output/'derived_selected_ledger.jsonl'
    derived.write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in rows))
    result=convert(derived,output/'infos',sensor)
    result.update(profile=profile,canonical_ledger_sha256=digest,gained_gt=dict(gained),lost_gt=dict(lost),
        changed_components=changes,raw_3d_geometry_unchanged=True,
        limitations=['Visibility resize uses nearest binary mask, not RGB interpolation.',
          '320 reference recomputation has historical Pillow/OpenCV integer-boundary differences.',
          'Collection acceptance remains original; derived audits are only training label selection.'])
    (output/'crop_conversion.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='changed_components'},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--analysis',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sensor',type=Path,default=Path('configs/sensors/Sensor.json'))
    p.add_argument('--profile',required=True);a=p.parse_args()
    build(a.ledger,a.analysis,a.output,a.sensor,a.profile)
