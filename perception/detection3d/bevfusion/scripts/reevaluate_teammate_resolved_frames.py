#!/usr/bin/env python3
"""Re-score saved outputs on conservative, fully resolved GT frames (no inference)."""
import argparse
import json
import time
from pathlib import Path
from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
from tracking_prediction_evaluation.detection_evaluator import (
    read_detection_frames as read_boxes, build_evaluation_frames, evaluate_detection_frames)
from tracking_prediction_evaluation.pipeline_evaluator import (
    read_ego_points, read_output_frames, evaluate_pipeline)
from tracking_prediction_evaluation.static_transform import load_morai_to_map_transform
from tracking_prediction_evaluation.trajectory import read_morai_trajectories, transform_segments_planar
from tracking_prediction_evaluation.unresolved_frame_policy import resolved_frame_mask

ROOT=Path(__file__).resolve().parents[4]


def process(case, catalog, suffix):
    contract=json.loads((case/'case_contract.json').read_text())
    base=json.loads((case/'summary.json').read_text())
    old_pipeline=json.loads((case/'pipeline/summary.json').read_text())
    cls=old_pipeline['evaluation']['class']
    heading=contract.get('heading_correction_deg', -1.347)
    names=json.loads((case/'asset_name_map.json').read_text())['asset_names']
    raw=read_morai_trajectories(Path(contract['raw_bag']), include_classes=(cls,),
        heading_correction_deg=heading,asset_name_map=names)
    calibrated,calibration=transform_segments_to_asset_catalog_bbox_center(raw, catalog)
    translation,yaw=load_morai_to_map_transform(ROOT/'configs/tf/morai_lanelet_static_tf.yaml')
    raw_map=transform_segments_planar(raw,translation,yaw)
    segments=transform_segments_planar(calibrated,translation,yaw)
    ego=read_ego_points(Path(contract['raw_bag']),'/Ego_topic',translation,yaw,heading)
    frames,counts=build_evaluation_frames(segments,ego,read_boxes(case/'detections.bag',
        '/perception/object_recognition/detection/objects'))
    mask,policy=resolved_frame_mask(raw_map,segments,ego,[f.timestamp for f in frames])
    frames=tuple(f for f in frames if mask[f.timestamp])
    base['metrics']=evaluate_detection_frames(frames,class_names=(cls,),production_score_threshold=.1)
    base['resolved_frame_policy']=policy
    base['gt_reference_calibration']=calibration
    base['protocol']['note']='Secondary, whole-frame GT-resolution subset; published raw result preserved separately.'
    base['source']['asset_bbox_catalog']=str(catalog.resolve())
    base['runtime_coverage']['resolved_evaluated_frames']=len(frames)
    (case/f'summary{suffix}.json').write_text(json.dumps(base,indent=2)+'\n')
    tracks,predictions=read_output_frames(case/'pipeline/pipeline_outputs.bag',
        '/perception/object_recognition/tracking/objects','/perception/object_recognition/objects',class_name=cls)
    mask,policy=resolved_frame_mask(raw_map,segments,ego,set(tracks)|set(predictions))
    tracks={t:v for t,v in tracks.items() if mask[t]}
    predictions={t:v for t,v in predictions.items() if mask[t]}
    for motion,filename in [('all',f'summary{suffix}.json'),('cv-stable',f'summary_cv_stable{suffix}.json')]:
        result=evaluate_pipeline(segments,ego,tracks,predictions,prediction_motion_slice=motion,
            gt_reference='asset-catalog-bbox-center-v1')
        result['evaluation']['gt_reference_calibration']=calibration
        result['evaluation']['resolved_frame_policy']=policy
        result['source']=old_pipeline['source']
        result['source']['note']='Saved runtime outputs re-scored; no fresh inference/tracking.'
        result['source']['asset_bbox_catalog']=str(catalog.resolve())
        (case/'pipeline'/filename).write_text(json.dumps(result,indent=2)+'\n')
    print('RESOLVED',case.parent.name,case.name,policy,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT/'artifacts/evaluation/bevfusion/teammate_manual_261004')
    parser.add_argument('--wait-for-count',type=int,default=0)
    parser.add_argument('--catalog',type=Path,default=ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json')
    parser.add_argument('--suffix',default='_resolved_frames')
    parser.add_argument('--missions',nargs='+',choices=['highway','roundabout','pedestrian','obstacle'])
    args=parser.parse_args()
    deadline=time.monotonic()+10800
    while True:
        completed=0
        for contract in sorted(args.root.glob('*/*/case_contract.json')):
            case=contract.parent
            if args.missions and json.loads(contract.read_text())['mission'] not in args.missions:continue
            if not (case/'pipeline/summary_cv_stable.json').is_file():continue
            if not (case/f'pipeline/summary_cv_stable{args.suffix}.json').is_file():process(case,args.catalog,args.suffix)
            completed+=1
        if completed>=args.wait_for_count:break
        if time.monotonic()>deadline:raise TimeoutError(completed)
        time.sleep(5)


if __name__=='__main__':main()
