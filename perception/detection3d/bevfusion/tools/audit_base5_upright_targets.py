#!/usr/bin/env python3
"""CPU actual dataset-adapter audit: native corners, BEV augmentation and support.

Checks a fresh derived portable dataset against its immutable predecessor.
No RGB inference, simulator, model training or accuracy measurement.
"""
import argparse
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path
import pickle

import numpy as np
import torch
from mmengine.config import Config
from mmengine.dataset import Compose
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import DATASETS
from mmdet3d.structures import LiDARPoints
from mmdet3d.utils import register_all_modules

from prepare_base5_portable_dataset import sha256, validate
from prepare_base5_upright_portable import native_corners, rotation
from dataset.morai_od.scripts.base5_upright_boxes import BOX_GEOMETRY_POLICY


def local_points(points, boxes):
    relative = points - boxes[:,None,:3]
    yaw = boxes[:,6]; c=np.cos(yaw)[:,None];s=np.sin(yaw)[:,None]
    return np.stack((relative[:,:,0]*c+relative[:,:,1]*s,
                     -relative[:,:,0]*s+relative[:,:,1]*c,relative[:,:,2]),axis=-1)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--data',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Use a fresh audit output')
    torch.set_num_threads(1);register_all_modules(init_default_scope=True)
    cfg=Config.fromfile(str(a.config));import_modules_from_strings(**cfg.custom_imports)
    manifest=validate(a.data)
    assert manifest['box_geometry_policy']==BOX_GEOMETRY_POLICY
    assert manifest['source_pre_upright_manifest_sha256']==sha256(a.source/'manifest.json')
    geometry=Compose([deepcopy(t) for t in cfg.train_dataloader.dataset.pipeline if t['type']=='BEVFusionGlobalRotScaleTrans'])
    counts=Counter();size_changes=[];examples=[]
    for split in ('train','val'):
        path=f'infos/512x1056/morai_base5_infos_{split}.pkl'
        old=pickle.loads((a.source/path).read_bytes());new=pickle.loads((a.data/path).read_bytes())
        opts=deepcopy(cfg.train_dataloader.dataset if split=='train' else cfg.val_dataloader.dataset)
        opts.update(ann_file=str((a.data/path).resolve()),data_root=str(a.data.resolve()),pipeline=[],test_mode=False,filter_empty_gt=False)
        dataset=DATASETS.build(opts);assert len(dataset)==len(old['data_list'])
        for idx,(before,row) in enumerate(zip(old['data_list'],new['data_list'])):
            compare=deepcopy(row)
            assert len(before['instances'])==len(row['instances'])
            for x,y in zip(before['instances'],compare['instances']):y['bbox_3d']=x['bbox_3d']
            assert compare==before
            cap=a.data/'data'/row['token'];components={c['component_id']:c for c in json.loads((cap/'audit.json').read_text())['components']}
            # Independent text-to-component comparison, not just roundtrip.
            native={};ordinals=Counter();files=list(cap.glob('LIDAR_*/*_Instance.txt'));assert len(files)==1
            for line in files[0].read_text().splitlines():
                if not line.strip():continue
                fields=line.split();assert len(fields)>=15
                key=(int(fields[14]),fields[0]);n=ordinals[key];ordinals[key]+=1
                native[f'{key[0]}:{key[1]}:{n}']=np.asarray([float(v) for v in fields[1:10]])
            boxes=dataset.get_data_info(idx)['ann_info']['gt_bboxes_3d']
            center_boxes=np.asarray([i['bbox_3d'] for i in row['instances']],dtype=np.float32).reshape(-1,7)
            np.testing.assert_allclose(boxes.gravity_center.numpy(),center_boxes[:,:3],rtol=0,atol=2e-6)
            corners=[];tilted=[]
            for x,y in zip(before['instances'],row['instances']):
                c=components[y['component_id']];text=native[y['component_id']]
                np.testing.assert_allclose(text,np.r_[c['center_lidar_m'],c['rotation_rpy'],c['size_m']],rtol=1e-7,atol=1e-6)
                corners.append(native_corners(c));counts['native_gt']+=1
                size_changes.append(np.abs(np.asarray(y['bbox_3d'][3:6])-x['bbox_3d'][3:6]))
                tilt=np.degrees(np.arccos(np.clip(abs(rotation(c['rotation_rpy'])[2,2]),0,1)))
                if tilt>5:tilted.append((x,y,c));counts['tilt_above_5_deg']+=1
                if c['lidar_points']==0:counts['camera_only']+=1
                if c.get('camera_component_ambiguous'):counts['ambiguous_components']+=1
            counts['frames']+=1
            if not corners:continue
            corners=np.asarray(corners,dtype=np.float32)
            repeats=2 if split=='train' else 0
            for repeat in range(-1,repeats):
                target=deepcopy(boxes);pts=LiDARPoints(torch.from_numpy(corners.reshape(-1,3).copy()),points_dim=3)
                if repeat>=0:
                    np.random.seed(idx+261011+repeat*5000)
                    transformed=geometry(dict(points=pts,gt_bboxes_3d=target,gt_labels_3d=np.asarray([i['bbox_label_3d'] for i in row['instances']])))
                    target=transformed['gt_bboxes_3d'];pts=transformed['points']
                metric=target.tensor.numpy().copy();metric[:,:3]=target.gravity_center.numpy()
                local=local_points(pts.tensor.numpy().reshape(-1,8,3),metric)
                assert (np.abs(local)<=metric[:,None,3:6]/2+5e-5).all(),(row['token'],repeat)
                counts['corner_enclosure_gt_checks']+=len(metric)
            if tilted:
                points=np.fromfile(a.data/row['lidar_points']['lidar_path'],np.float32).reshape(-1,4)[:,:3]
                points=points[(points!=0).any(axis=1)]
                for x,y,c in tilted:
                    full=(np.abs((points-c['center_lidar_m'])@rotation(c['rotation_rpy']))<=np.asarray(c['size_m'])/2+1e-5).all(1)
                    assert int(full.sum())==c['lidar_points']
                    for label,inst in (('before',x),('after',y)):
                        bbox=np.asarray(inst['bbox_3d'])[None]
                        inside=(np.abs(local_points(points[None],bbox)[0])<=bbox[0,3:6]/2+1e-5).all(1)
                        missed=int((full&~inside).sum());counts[label+'_native_support_outside']+=missed
                        if label=='after':assert missed==0
                    counts['tilted_native_points']+=int(full.sum())
                    if row['token']=='b5v2_04541' and c['asset']=='PE_Drum':examples.append((c,x,y))
    changes=np.asarray(size_changes)
    report=dict(valid=True,scope='Actual adapter + native 8 corners + actual BEV transforms; not AP',
        counts=dict(counts),dimension_abs_change_m=dict(p50=np.percentile(changes,50,axis=0).tolist(),
        p95=np.percentile(changes,95,axis=0).tolist(),maximum=changes.max(0).tolist()),
        source_manifest_sha256=sha256(a.source/'manifest.json'),derived_manifest_sha256=sha256(a.data/'manifest.json'))
    a.output.mkdir(parents=True)
    if examples:draw_example(examples[0],a.output/'sideways_barrel_boxes.png')
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


def draw_example(example,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    c,before,after=example
    fig=plt.figure(figsize=(7,5));ax=fig.add_subplot(111,projection='3d')
    for label,color,component in [('Native full RPY','blue',c),('Before: yaw only','red',dict(center_lidar_m=c['center_lidar_m'],size_m=before['bbox_3d'][3:6],rotation_rpy=[0,0,c['rotation_rpy'][2]])),('After: enclosure','green',dict(center_lidar_m=c['center_lidar_m'],size_m=after['bbox_3d'][3:6],rotation_rpy=[0,0,c['rotation_rpy'][2]]))]:
        vertices=native_corners(component)-c['center_lidar_m']
        for i in range(8):
            for j in range(i+1,8):
                if bin(i^j).count('1')==1:ax.plot(*vertices[[i,j]].T,color=color,alpha=.7,label=label if i==0 and j==1 else None)
    ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_zlabel('Z (m)');ax.set_box_aspect([1,1,1]);ax.legend()
    ax.set_title('Actual PE_Drum: b5v2_04541 | center-relative boxes')
    fig.savefig(path,dpi=150);plt.close(fig)


if __name__=='__main__':main()
