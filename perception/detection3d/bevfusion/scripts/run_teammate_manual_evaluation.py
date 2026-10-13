#!/usr/bin/env python3
"""Run E25 and Q1-P7 on received manual bags with explicit GT provenance."""
import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
E25 = ROOT / 'artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth'
P7 = ROOT / ('artifacts/detection3d/bevfusion/team_training/sw/'
    'bf-sw-base4-q1-p3-robust-motion-gate90-hardcase-v2-w1-head5x-e4-rtx5060ti-260923-115103/epoch_1.pth')


def run(command, env, log=None):
    print('RUN', ' '.join(str(v) for v in command), flush=True)
    if log:
        with log.open('w') as handle:
            subprocess.run(command, cwd=ROOT, env=env, stdout=handle,
                           stderr=subprocess.STDOUT, check=True)
    else:
        subprocess.run(command, cwd=ROOT, env=env, check=True)


def prepare_map(meta, output):
    scenario = meta['run']['scenario']
    content = subprocess.check_output(['git', 'show', meta['git_sha'] +
        ':configs/scenario/human_driving_evaluation/' + scenario], cwd=ROOT)
    assert hashlib.sha256(content).hexdigest() == meta['scenario_sha256'], scenario
    s = json.loads(content)
    catalog = json.loads((ROOT/'dataset/morai_od/configs/morai_asset_catalog_260807.json').read_text())
    entries = {int(v['data_id']):v['name'] for values in catalog.values()
               if isinstance(values, list) for v in values if isinstance(v,dict) and 'data_id' in v}
    names = {}
    for cls, field in [('pedestrian','pedestrianList'),('obstacle','objectList')]:
        for obj in s.get(field,[]):
            scale = obj.get('scale',{})
            assert all(abs(float(scale.get(a,1))-1)<1e-6 for a in ['x','y','z']), 'nonunit scale'
            names[f"{cls}:{obj['UNIQUEID']}"] = entries[int(obj['DataID'])]
    path = output/'asset_name_map.json'
    output.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(dict(asset_names=names, scenario=scenario,
        scenario_sha256=meta['scenario_sha256'], source_git_sha=meta['git_sha']),indent=2)+'\n')
    return path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags',type=Path,default=ROOT/'rosbags/eval/perception/260923_teammate_manual')
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/evaluation/bevfusion/teammate_manual_261004')
    parser.add_argument('--models',nargs='+',choices=['e25','p7'],default=['e25','p7'])
    parser.add_argument('--phase',choices=['all','detection','pipeline'],default='all')
    parser.add_argument('--wait-for-detections',action='store_true')
    args=parser.parse_args()
    base=os.environ.copy()
    base.update(AJOUNICE_ROOT=str(ROOT),BEVFUSION_IMAGE='ajounice/bevfusion:team-cu128-v1',
        BEVFUSION_EVAL_GT_REFERENCE='asset-catalog-bbox-center-v1',
        BEVFUSION_EVAL_MORAI_HEADING_CORRECTION_DEG='-1.347',
        BEVFUSION_PIPELINE_EVAL_MAXIMUM_TRACKING_AGE_SEC='0.5')
    compose=['docker','compose','-f',str(ROOT/'docker/bevfusion/compose.yaml')]
    for model in args.models:
        env=base.copy()
        env['BEVFUSION_CHECKPOINT']=str(E25 if model=='e25' else P7)
        env['BEVFUSION_MODEL_CONFIG']='/workspace/AjouNice2026/perception/detection3d/bevfusion/configs/'+(
            'bevfusion_base3_b0_4cam_lidar.py' if model=='e25' else
            'bevfusion_base4_q1_p7_free_intercept_4cam_lidar_velocity.py')
        env['BEVFUSION_EVAL_QUERY_MEMORY_CONTEXT']='true' if model=='p7' else 'false'
        env['BEVFUSION_EVAL_PUBLISH_DETECTOR_VELOCITY']='true' if model=='p7' else 'false'
        env['BEVFUSION_EVAL_DETECTOR_VELOCITY_NOISE_STD_MPS']='0.5'
        env['BEVFUSION_EVAL_LIDAR_DESKEW_MODE']='off'
        assert Path(env['BEVFUSION_CHECKPOINT']).is_file(),env['BEVFUSION_CHECKPOINT']
        if args.phase in ['all','detection']:
            run(compose+['up','-d','--force-recreate','--pull','never','inference'],env)
            health=env.copy();health['PYTHONPATH']=str(ROOT/'docker/bevfusion')
            for attempt in range(120):
                check=subprocess.run(['/usr/bin/python3',str(ROOT/'docker/bevfusion/scripts/healthcheck.py')],
                    cwd=ROOT,env=health,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                if check.returncode==0:break
                time.sleep(1)
            else:raise RuntimeError('inference server not ready')
        for bag in sorted(args.bags.rglob('*.bag')):
            mission=bag.parent.parent.name
            if model=='p7' and mission!='highway':continue
            metadata=list(bag.parent.glob('*metadata.json'))
            assert len(metadata)==1,bag
            meta=json.loads(metadata[0].read_text())
            label=meta['run']['id']+'_'+bag.name.split('_')[1]
            output=args.output/model/label;output.mkdir(parents=True,exist_ok=True)
            name_map=prepare_map(meta,output)
            case_env=env.copy();case_env['BEVFUSION_EVAL_ASSET_NAME_MAP']=str(name_map.resolve())
            case_env['BEVFUSION_EVAL_CLASS_NAME']={'highway':'vehicle','roundabout':'vehicle',
                'pedestrian':'pedestrian','obstacle':'obstacle'}[mission]
            if args.phase=='pipeline' and args.wait_for_detections:
                deadline=time.monotonic()+7200
                while not (output/'summary.json').exists():
                    if time.monotonic()>deadline:raise TimeoutError(str(output))
                    time.sleep(5)
            if args.phase in ['all','detection'] and not (output/'summary.json').exists():
                run(['bash',str(ROOT/'perception/detection3d/bevfusion/scripts/run_rosbag_detection_evaluation.sh'),
                     str(bag.resolve()),str(output.resolve()),label,'0.5'],case_env,output/'detection_runner.log')
            if args.phase in ['all','pipeline'] and not (output/'pipeline/summary_cv_stable.json').exists():
                run(['bash',str(ROOT/'perception/detection3d/bevfusion/scripts/run_recorded_detections_pipeline_evaluation.sh'),
                     str(bag.resolve()),str((output/'detections.bag').resolve()),
                     str((output/'pipeline').resolve()),label,'1.0'],case_env,output/'pipeline_runner.log')
            (output/'case_contract.json').write_text(json.dumps(dict(model=model,checkpoint=env['BEVFUSION_CHECKPOINT'],
                config=env['BEVFUSION_MODEL_CONFIG'],mission=mission,role=meta['run']['role'],
                asset_name_map=str(name_map),raw_bag=str(bag.resolve()),detection_playback_rate=.5,
                pipeline_playback_rate=1.,gt_reference='asset-catalog-bbox-center-v1',
                heading_correction_deg=-1.347,tracking_maximum_age_sec=.5,
                tracker_velocity_fusion_mode=0,query_memory=model=='p7',lidar_deskew='off'),indent=2)+'\n')
            print('COMPLETE',model,label,flush=True)
        if args.phase in ['all','detection']:
            run(compose+['stop','inference'],env)


if __name__=='__main__':
    main()
