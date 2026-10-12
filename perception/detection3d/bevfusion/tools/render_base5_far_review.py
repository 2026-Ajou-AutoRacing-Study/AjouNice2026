#!/usr/bin/env python3
"""Original-camera overlays for selected unmatched ROS-track diagnostics.

Projection is for manual review, not visibility-GT filtering. Red=track,
cyan=known catalog actors. Selected duration-ranked examples are not random.
"""
import argparse
import json
from pathlib import Path
import pickle

import cv2
import numpy as np


def corners(box,bottom=True):
    box=np.asarray(box,float);center=box[:3].copy()
    if bottom:center[2]+=box[5]/2
    signs=np.array([[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],
                    [1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]])
    points=signs*box[3:6]/2
    c,s=np.cos(box[6]),np.sin(box[6]);points[:,:2]=points[:,:2]@np.array([[c,s],[-s,c]])
    return points+center


def draw_box(image,box,projection,color,bottom=True):
    xyz=corners(box,bottom);p=np.asarray(projection)@np.c_[xyz,np.ones(8)].T
    valid=p[2]>.1;uv=(p[:2]/np.maximum(p[2],.1)).T
    for i in range(8):
        for j in range(i+1,8):
            if bin(i^j).count('1')!=1 or not valid[i] or not valid[j]:continue
            a=tuple(np.clip(uv[i],-100000,100000).astype(int));b=tuple(np.clip(uv[j],-100000,100000).astype(int))
            visible,start,end=cv2.clipLine((0,0,image.shape[1],image.shape[0]),a,b)
            if visible:cv2.line(image,start,end,color,2,cv2.LINE_AA)


def render(args):
    args.output.mkdir(parents=True,exist_ok=False)
    report=json.loads(args.report.read_text())
    sources={r['token']:r for r in pickle.loads((args.inputs/'infos_val.pkl').read_bytes())['data_list']}
    replay_name = getattr(args, 'replay_name', 'ros_near_zero_coast_v1')
    mode = getattr(args, 'mode', 'off')
    cases=report['tracking'][replay_name][mode]['cases']
    examples=[]
    for case,classes in cases.items():
        for name,value in classes.items():
            persistence=value['unmatched_persistence']
            selected=persistence if getattr(args, 'diagnostic', 'no-actor') == 'all' else persistence['no_catalog_actor_runs']
            for run in selected['worst'][:2]:
                examples.append((case,name,run))
    entries=[]
    for n,(case,name,run) in enumerate(examples):
        ex=run['example'];source=sources[ex['token']];tiles=[]
        for camera,meta in source['images'].items():
            original=cv2.imread(meta['img_path'])
            if original is None:raise ValueError('Original JPEG unavailable')
            for gt in source['instances']:draw_box(original,gt['bbox_3d'],meta['lidar2img'],(255,255,0),bottom=False)
            draw_box(original,ex['box'],meta['lidar2img'],(40,40,255))
            tile=cv2.resize(original,(480,270));cv2.putText(tile,camera,(12,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
            tiles.append(tile)
        header=np.zeros((80,1920,3),dtype=np.uint8)
        text=f"{case} {name} track={run['track_key']} unmatched={run['seconds']:.2f}s {ex['token']}"
        cv2.putText(header,text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
        text=f"RED: tracked box; CYAN: catalog GT; local XY=({ex['box'][0]:.1f},{ex['box'][1]:.1f}) score={ex['score']:.2f}"
        cv2.putText(header,text,(12,60),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
        output=np.vstack([header,np.hstack(tiles)])
        path=args.output/f'{n:02d}_{case}_{name}.jpg'
        if not cv2.imwrite(str(path),output):raise ValueError('Overlay save failed')
        entries.append(dict(image=str(path),case=case,cls=name,run=run))
    (args.output/'index.json').write_text(json.dumps(entries,indent=2)+'\n')
    print('Saved',len(entries),'original-camera review overlays')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('inputs','report','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--replay-name', default='ros_near_zero_coast_v1')
    p.add_argument('--mode', choices=('off','on','off_repeat'), default='off')
    p.add_argument('--diagnostic', choices=('no-actor','all'), default='no-actor',
                   help='All unmatched includes duplicate/geometry/class-confusion near real actors; never semantic FP truth.')
    render(p.parse_args())
