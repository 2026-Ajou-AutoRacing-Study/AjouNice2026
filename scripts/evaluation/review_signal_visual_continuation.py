#!/usr/bin/env python3
"""Render every NEW visual-continuation acceptance for manual identity review.

Never draws a ground-truth label or turns CNN known into correctness. Paired
trusted/current original crops and full current frame are retained for review.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]


def render(replay_path,output):
    import cv2
    import numpy as np
    replay=json.loads(replay_path.read_text());input_path=Path(replay['source_inputs'])
    inputs=json.loads(input_path.read_text());by_stamp={r['source_stamp_ns']:r for r in inputs['records']}
    decisions={r['source_stamp_ns']:r for r in replay['records']}
    output=output.resolve();output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh visual review output required')
    output.mkdir(parents=True);groups=defaultdict(list);index=[]
    for row in replay['records']:
        current=row['outcomes'].get('current',{});decision=current.get('decision',{})
        visual=decision.get('visual_continuation',{})
        if visual.get('status')!='matched':continue
        source=by_stamp[row['source_stamp_ns']];anchor=by_stamp[visual['trusted_source_stamp_ns']]
        def image(record):
            payload=(input_path.parent/record['image_path']).read_bytes()
            if hashlib.sha256(payload).hexdigest()!=record['image_sha256']:raise ValueError('Original JPEG changed')
            return cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
        frame=image(source);previous=image(anchor)
        bounds=current['classifier']['crop_xyxy']
        previous_bounds=decisions[anchor['source_stamp_ns']]['outcomes']['current']['classifier']['crop_xyxy']
        def crop(img,b):
            a,y,c,d=b
            return cv2.resize(img[y:d,a:c],(288,96),interpolation=cv2.INTER_NEAREST)
        cell=np.full((240,640,3),245,np.uint8)
        display=frame.copy();a,b,c,d=current['classifier']['detector_bbox_xyxy']
        cv2.rectangle(display,(int(a),int(b)),(int(c),int(d)),(255,0,255),3)
        cell[42:222,0:320]=cv2.resize(display,(320,180))
        cell[34:130,342:630]=crop(frame,bounds)
        cell[138:234,342:630]=crop(previous,previous_bounds)
        lamps='+'.join(x[0]+('_L' if x[1]=='arrow' else '') for x in decision['final_tuples'])
        text='%s %.3fs %s %s'%(row['section'],row['t'],visual['head_id'],lamps)
        cv2.putText(cell,text,(5,20),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,0),1,cv2.LINE_AA)
        cv2.putText(cell,'CURRENT',(342,31),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,0,0),1)
        cv2.putText(cell,'PRIOR NORMAL MAP',(342,136),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,0,0),1)
        groups[row['section']].append(cell)
        index.append(dict(t=row['t'],section=row['section'],head_id=visual['head_id'],
            current_source_stamp_ns=row['source_stamp_ns'],trusted_source_stamp_ns=visual['trusted_source_stamp_ns'],
            current_image_path=str(input_path.parent/source['image_path']),
            trusted_image_path=str(input_path.parent/anchor['image_path']),
            current_crop_xyxy=bounds,trusted_crop_xyxy=previous_bounds,
            current_lamps=decision['final_tuples'],source_preserved_fresh=
                current['output_source_stamp_ns']==row['source_stamp_ns']))
    for section,cells in groups.items():
        sheet=np.full((240*math.ceil(len(cells)/2),1280,3),255,np.uint8)
        for i,cell in enumerate(cells):sheet[(i//2)*240:(i//2+1)*240,(i%2)*640:(i%2+1)*640]=cell
        cv2.imwrite(str(output/(section+'_all_new_acceptances.jpg')),sheet,[cv2.IMWRITE_JPEG_QUALITY,95])
    (output/'index.json').write_text(json.dumps(dict(replay=str(replay_path),new_acceptances=index,
        limitations=['Panels are for manual visual review, not independently annotated GT.',
            'Prior/current crop visual agreement does not independently prove global regulatory ID.']),indent=2)+'\n')
    print(json.dumps(dict(output=str(output),counts={k:len(v) for k,v in groups.items()})))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--replay',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();render(a.replay,a.output)
