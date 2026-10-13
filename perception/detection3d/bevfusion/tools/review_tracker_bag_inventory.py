#!/usr/bin/env python3
"""Read bag indexes and recorded model receipts; never replay or reindex sources."""
import argparse
from collections import Counter
import json
from pathlib import Path

TYPES=('autoware_perception_msgs/DetectedObjects','autoware_perception_msgs/TrackedObjects',
       'autoware_perception_msgs/PredictedObjects')


def inspect(path):
    import rosbag
    row=dict(path=str(path.resolve()),bytes=path.stat().st_size)
    try:
        with rosbag.Bag(str(path),'r') as bag:
            topics=bag.get_type_and_topic_info().topics
            selected={k:dict(type=v.msg_type,count=v.message_count) for k,v in topics.items()
                if v.msg_type in TYPES or k in ('/Object_topic','/Ego_topic','/tf','/tf_static',
                    '/localization/kinematic_state','/planning/trajectory','/CollisionData',
                    '/perception/bevfusion/status','/sensor/lidar1') or k.endswith('/compressed')}
            row.update(duration_sec=bag.get_end_time()-bag.get_start_time(),topics=selected,
                perception=any(v.msg_type in TYPES for v in topics.values()))
        sidecar=path.with_suffix('.json')
        if sidecar.is_file():
            data=json.loads(sidecar.read_text())
            model=data.get('model') or {}
            row['recorded_model']={k:model[k] for k in ('preset','checkpoint_sha256','checkpoint_verified',
                'model_config_sha256','thresholds') if k in model}
            row['scenario']=data.get('scenario')
            row['recorded_status']=data.get('status')
        row['provenance']='recorded_model_receipt' if row.get('recorded_model') else 'not_identified_by_filename'
    except Exception as e:row.update(error_type=type(e).__name__,error=str(e))
    return row


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--roots',nargs='+',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Fresh inventory required')
    paths=sorted({s.resolve() for root in a.roots for s in root.rglob('*.bag')})
    rows=[inspect(s) for s in paths]
    output=dict(scope='Indexes and saved model receipts only; detector provenance is not guessed from bag names.',
        bags=len(rows),perception_bags=sum(r.get('perception',False) for r in rows),
        errors=sum('error' in r for r in rows),rows=rows)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(output,indent=2,ensure_ascii=False)+'\n')
    print({k:v for k,v in output.items() if k!='rows'})
    print('Recorded models',dict(Counter((r.get('recorded_model') or {}).get('preset','unidentified')
        for r in rows if r.get('perception'))))
