#!/usr/bin/env python3
"""Small-pilot 512 mask label derivation; preserve native raw and approval QA."""
import argparse
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
from build_base5_bevfusion_infos import convert


def build(ledger,measurement,output,sensor):
    if output.exists():raise FileExistsError('New derived pilot output required')
    report=json.loads(measurement.read_text())
    if report['ledger_sha256']!=sha256(ledger) or report['errors']:
        raise ValueError('Measured native mask/source mismatch')
    lookup={f['id']:f for f in report['captures']}
    output.mkdir(parents=True);rows=[]
    for line in ledger.read_text().splitlines():
        row=json.loads(line)
        if row['status']!='accepted_candidate':continue
        root=Path(row['capture_root']);original=Path(row.get('audit_path',root/'audit.json'))
        audit=json.loads(original.read_text());new=copy.deepcopy(audit)
        measured={c['component_id']:c for c in lookup[row['id']]['components']}
        for c in new['components']:
            cc=measured[c['component_id']]
            c['camera_visible_candidate']=cc['camera_gate512']
            c['positive_lidar1_camera0']=c['lidar_points']>=1 or cc['camera_gate512']
            c['ignore_reason']=None if c['positive_lidar1_camera0'] else 'no_visible_sensor_evidence'
        new['derived_visibility']=dict(profile='512x1056',measurement_sha256=sha256(measurement),
            original_audit_sha256=sha256(original),raw_preserved=True,pilot_not_training=True)
        path=output/'derived_audits'/(row['id']+'.json');atomic_json(path,new)
        row['audit_path']=str(path.resolve());row['audit']=new;rows.append(row)
    derived=output/'derived_selected_ledger.jsonl'
    # The generated ledger is a program output, never an edit to original raw.
    with derived.open('x') as stream:
        for row in rows:stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    # Portable packing resolves each info token against the capture ledger ID;
    # group identity stays separate and is preserved in scene_group_id.
    result=convert(derived,output/'infos',sensor)
    result.update(source_ledger_sha256=sha256(ledger),measurement_sha256=sha256(measurement),
                  profile='512x1056',pilot_only=True,raw_preserved=True)
    atomic_json(output/'conversion_report.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('ledger','measurement','output','sensor'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();print(json.dumps(build(a.ledger,a.measurement,a.output,a.sensor),indent=2))
