#!/usr/bin/env python3
"""Actual 5K geometry / point quantization / proposal competition audit.

No AP, sensor rewrite or training. Raw native-RPY support is used for point
statistics, not enlarged-box background. Feature-cell collisions are risks,
not proof of missed detection: the head predicts continuous center offsets.
"""
import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import itertools
import json
from pathlib import Path
import pickle
import time

import numpy as np
import torch
from mmengine.config import Config
from mmengine.structures import InstanceData
from mmengine.utils import import_modules_from_strings
from mmdet3d.structures import LiDARInstance3DBoxes
from projects.BEVFusion.bevfusion.utils import TransFusionBBoxCoder
from base5_grid_candidates import SPATIAL_CASES, candidate_config, geometry
from prepare_base5_portable_dataset import sha256, validate
from prepare_base5_upright_portable import rotation, native_corners
from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import MoraiRectangularTransFusionHead

CLASSES=('vehicle','pedestrian','obstacle','animal')


def quantiles(values):
    if not values:return None
    values=np.asarray(values,float)
    return dict(count=len(values),mean=float(values.mean()),
                p10=float(np.percentile(values,10)),p50=float(np.percentile(values,50)),
                p95=float(np.percentile(values,95)),sum=float(values.sum()))


def distance_bucket(xy):
    distance=float(np.linalg.norm(xy))
    for limit in (10,30,60):
        if distance<limit:return f'under{limit}'
    return '60plus'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('Use a fresh output')
    a.output.mkdir(parents=True);torch.set_num_threads(1)
    manifest=validate(a.data)
    if manifest.get('box_geometry_policy')!='native_rpy_fixed_yaw_enclosure_v1':raise ValueError('Upright data required')
    base=Config.fromfile(str(a.config));import_modules_from_strings(**base.custom_imports)
    configs={case:candidate_config(base,case) for case in SPATIAL_CASES}
    for case,cfg in configs.items():cfg.dump(str(a.output/(case+'.py')))
    heads={};coders={};positions={};totals={case:Counter() for case in SPATIAL_CASES}
    sizes={case:defaultdict(list) for case in SPATIAL_CASES}
    voxel_stats={case:defaultdict(list) for case in SPATIAL_CASES}
    before=sha256(a.data/'manifest.json');native_counts=Counter();objects=[];depth_observations=[]
    for case,cfg in configs.items():
        head=object.__new__(MoraiRectangularTransFusionHead);torch.nn.Module.__init__(head)
        head.train_cfg=deepcopy(cfg.model.bbox_head.train_cfg);head.num_classes=4;heads[case]=head
        g=geometry(case)
        positions[case]=head.create_2D_grid(g['fusion_shape_yx'][1],g['fusion_shape_yx'][0])
        coders[case]=TransFusionBBoxCoder(**{k:v for k,v in cfg.model.bbox_head.bbox_coder.items() if k!='type'})
    start=time.monotonic()
    for split in ('train','val'):
        records=pickle.loads((a.data/f'infos/512x1056/morai_base5_infos_{split}.pkl').read_bytes())['data_list']
        for idx,row in enumerate(records):
            raw=np.fromfile(a.data/row['lidar_points']['lidar_path'],np.float32).reshape(-1,4)
            raw=raw[np.isfinite(raw).all(1)&(raw[:,:3]!=0).any(1),:3]
            xyz=raw[((raw>=[-100,-30,-3])&(raw<[100,30,2])).all(1)]
            audit=json.loads((a.data/'data'/row['token']/'audit.json').read_text())
            components={c['component_id']:c for c in audit['components']}
            boxes=torch.tensor([i['bbox_3d'] for i in row['instances']],dtype=torch.float32).reshape(-1,7)
            labels=torch.tensor([i['bbox_label_3d'] for i in row['instances']],dtype=torch.long)
            gt=InstanceData(bboxes_3d=LiDARInstance3DBoxes(boxes,origin=(.5,.5,.5)),labels_3d=labels)
            for case,head in heads.items():
                g=geometry(case);centers=((boxes[:,:2]-torch.tensor(g['origin']))/g['fusion_pitch_m']).int()
                target=head.build_dense_heatmap_targets(gt)
                if len(boxes):
                    assert (target[0,labels,centers[:,1],centers[:,0]]==1).all()
                    pos=positions[case]
                    torch.testing.assert_close(pos[0,centers[:,1]*g['fusion_shape_yx'][1]+centers[:,0]],centers.float()+.5,rtol=0,atol=0)
                    coder=coders[case];encoded=coder.encode(gt.bboxes_3d.tensor)
                    score=torch.zeros(1,4,len(boxes));score[0,labels,torch.arange(len(boxes))]=1
                    decoded=coder.decode(score,encoded[:,6:8].T[None],encoded[:,3:6].T[None],encoded[:,:2].T[None].clone(),encoded[:,2:3].T[None],None)[0]['bboxes']
                    torch.testing.assert_close(decoded[:,:6],gt.bboxes_3d.tensor[:,:6],atol=5e-5,rtol=0)
                totals[case]['frames']+=1;totals[case]['aligned_gt_peaks']+=len(boxes)
                occupied=Counter((int(l),int(y),int(x)) for l,(x,y) in zip(labels,centers))
                totals[case]['same_class_same_cell_excess']+=sum(n-1 for n in occupied.values())
                for (i,j) in itertools.combinations(range(len(boxes)),2):
                    if labels[i]==labels[j] and int((centers[i]-centers[j]).abs().max())<=1:
                        totals[case]['same_class_3x3_neighbor_pairs']+=1
                for inst in row['instances']:
                    cls=CLASSES[inst['bbox_label_3d']]
                    size=min(inst['bbox_3d'][3:5])/g['fusion_pitch_m']
                    sizes[case][cls].append(size)
            for inst in row['instances']:
                component=components[inst['component_id']];cls=CLASSES[inst['bbox_label_3d']]
                native_counts[cls]+=1
                center=np.asarray(component['center_lidar_m']);size=np.asarray(component['size_m'])
                rot=rotation(component['rotation_rpy'])
                support=xyz[(np.abs((xyz-center)@rot)<=size/2+1e-5).all(1)]
                obj=dict(token=row['token'],component_id=inst['component_id'],asset=inst['asset'],cls=cls,
                    distance_m=float(np.linalg.norm(center[:2])),native_points=component['lidar_points'],
                    model_roi_native_points=len(support),voxels={})
                for case in SPATIAL_CASES:
                    g=geometry(case)
                    coors=np.floor((support-[*g['origin'],-5.])/g['voxel']).astype(np.int64)
                    unique=len(np.unique(coors,axis=0))
                    obj['voxels'][case]=unique
                    for key in ('all',cls,cls+':'+distance_bucket(center[:2])):
                        voxel_stats[case][key].append(dict(points=len(support),voxels=unique))
                objects.append(obj)
                corners=native_corners(component)
                for camera_name,cam in row['images'].items():
                    matrix=np.asarray(cam['lidar2cam']);camcenter=(matrix@np.r_[center,1])[:3]
                    if not 1<=camcenter[2]<100:continue
                    projected=np.asarray(cam['cam2img'])[:3,:3]@camcenter;pixel=projected[:2]/projected[2]
                    # Current deterministic bottom crop, native side resolution included.
                    scale=.825*1280/cam['width'];new_h=int(cam['height']*scale)
                    uv=pixel*scale-[0,new_h-512]
                    if not ((uv>=[0,0])&(uv<[1056,512])).all():continue
                    depth=(corners@matrix[:3,:3].T+matrix[:3,3])[:,2]
                    depth_observations.append(dict(cls=cls,depth_m=float(camcenter[2]),
                        depth_span_m=float(depth.max()-depth.min()),
                        note='geometrically projectable center, not visibility or depth prediction accuracy'))
            if (idx+1)%500==0:print('AUDIT '+split+' '+str(idx+1)+'/'+str(len(records)),flush=True)
    report=dict(valid=True,scope='All 5K nominal geometry / target / coder / native-supported voxel information; not AP or complete physical projection certification',
        source_manifest_sha256=before,frames=5000,gt=sum(native_counts.values()),classes=dict(native_counts),
        cases={},depth=dict(projectable_center_camera_observations=len(depth_observations),
            note='Depth is camera optical Z; projected center does not establish visibility. Spacing is not learned depth error.',
            center_depth_m=quantiles([o['depth_m'] for o in depth_observations]),
            native_bbox_depth_span_m=quantiles([o['depth_span_m'] for o in depth_observations])),
        elapsed_sec=time.monotonic()-start)
    for case in SPATIAL_CASES:
        g=geometry(case)
        report['cases'][case]=dict(geometry=g,targets=dict(totals[case]),
            minimum_gaussian_radius_m=base.model.bbox_head.train_cfg.min_radius*g['fusion_pitch_m'],
            local_max_neighbor_pitch_m=g['fusion_pitch_m'],
            nominal_rf_note='Convolution stencil size fixed: metric receptive field changes with pitch; cannot infer AP.',
            short_axis_feature_cells={k:dict(distribution=quantiles(v),below1=sum(x<1 for x in v),below2=sum(x<2 for x in v)) for k,v in sizes[case].items()},
            native_supported_voxels={k:dict(objects=len(v),zero_usable_points=sum(x['points']==0 for x in v),
                points=quantiles([x['points'] for x in v]),voxels=quantiles([x['voxels'] for x in v]),
                merged_points=sum(x['points']-x['voxels'] for x in v)) for k,v in voxel_stats[case].items()})
    assert before==sha256(a.data/'manifest.json')
    (a.output/'objects.json').write_text(json.dumps(objects,indent=2)+'\n')
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2),flush=True)


if __name__=='__main__':main()
