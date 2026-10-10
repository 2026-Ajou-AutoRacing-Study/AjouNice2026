#!/usr/bin/env python3
"""Actual dataset loader/pack/GPU normalize versus raw online path, read-only."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import torch
from mmdet3d.apis import init_model
from mmdet3d.registry import DATASETS
from app.online_preprocessor import Base3OnlinePreprocessor,preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from benchmark_base5_input_optimization import config_for,load_samples


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path('/workspace/AjouNice2026'))
    p.add_argument('--samples',type=Path,required=True);p.add_argument('--infos',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Do not overwrite evidence')
    manifest,samples=load_samples(a.samples);result=dict(source=manifest,cases=[])
    calibration=load_sensor4_calibration(a.root/'configs/sensors/Sensor.json')
    torch.set_num_threads(4)
    for shape in ('448x896','512x1056'):
        cfg=config_for(a.root,tuple(map(int,shape.split('x'))))
        model=init_model(deepcopy(cfg),None,device='cuda:0')
        pre=Base3OnlinePreprocessor(calibration,camera_workers=4,**preprocessing_options_from_config(cfg))
        items={}
        for split in ('train','val'):
            dc=deepcopy(cfg.val_dataloader.dataset);dc.ann_file=str(a.infos/('morai_base5_infos_'+split+'.pkl'))
            ds=DATASETS.build(dc)
            lookup={ds.get_data_info(i)['scene_id']:i for i in range(len(ds))}
            for s in samples:
                if s['meta']['id'] in lookup:items[s['meta']['id']]=ds[lookup[s['meta']['id']]]
        for s in samples:
            offline=items[s['meta']['id']]
            online=pre.prepare(s['points'],s['images'])
            torch.testing.assert_close(offline['inputs']['img'].float(),online['inputs']['img'][0].float(),atol=0,rtol=0)
            torch.testing.assert_close(offline['inputs']['points'],online['inputs']['points'][0],atol=0,rtol=0)
            for k in ('cam2img','lidar2cam','cam2lidar','lidar2img','img_aug_matrix','lidar_aug_matrix'):
                # Test pipeline has no global augmentation; BEVFusion itself
                # defaults a missing lidar_aug_matrix to identity.
                value=offline['data_samples'].metainfo.get(k,np.eye(4) if k=='lidar_aug_matrix' else None)
                np.testing.assert_allclose(value,online['data_samples'][0].metainfo[k],atol=1e-5)
            batch=dict(inputs={k:[v] for k,v in offline['inputs'].items()},data_samples=[offline['data_samples']])
            with torch.no_grad():
                old=model.data_preprocessor(batch,training=False)
                new=model.data_preprocessor(online,training=False)
            torch.testing.assert_close(old['inputs']['imgs'],new['inputs']['imgs'],atol=0,rtol=0)
        result['cases'].append(dict(shape=shape,samples=len(samples),actual_dataset_loader=True,
            raw_pixels_exact=True,calibration_close=True,normalized_pixels_exact=True,points_exact=True))
        pre.close();del model,pre,ds;torch.cuda.empty_cache()
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['cases'],indent=2))


if __name__=='__main__':main()
