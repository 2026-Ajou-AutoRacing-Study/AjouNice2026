#!/usr/bin/env python3
"""CPU-only explicit box-origin audit against immutable native annotation.

This neither rewrites raw predictions/labels nor enables a ROS change. XY,
class, score, heading, dimensions and XY-matching are frozen. The trial helper
requires named/versioned origins and copies its input; never infer the origin
from Z or asset size, and never convert a box twice without a declared change.
"""
import argparse
from collections import Counter, defaultdict
import gzip
import json
from pathlib import Path
import pickle

import numpy as np

from analyze_base5_proposal_peaks import CLASSES, THRESHOLDS, distribution, eligible_matches, prepared
from experiment_base5_proposal_peaks import ROOT, digest

ORIGINS = ('lidar_bottom_v1', 'gravity_center_v1')


def convert_origin_trial(boxes, source_origin, destination_origin):
    """Versioned research-only origin conversion, not a model/GT coordinate fix."""
    if source_origin not in ORIGINS or destination_origin not in ORIGINS:
        raise ValueError('An explicit supported source and destination origin is required')
    values = np.asarray(boxes, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < 7 or not np.isfinite(values).all():
        raise ValueError('Finite Nx7+ box values required')
    if not (values[:,3:6]>0).all():
        raise ValueError('Positive dimensions required')
    result = values.copy()
    if source_origin != destination_origin:
        result[:,2] += values[:,5] * (.5 if destination_origin=='gravity_center_v1' else -.5)
    return result


def convert_record_origin_trial(record, destination_origin):
    """Copy a tagged research record; repeated conversion stays idempotent.

    A wire/adapter integration must carry this origin marker, not merely pass
    an out-of-band guessed source value for every call. Untagged legacy records
    fail explicitly here; a known legacy origin belongs in a separate audited
    checkpoint/producer adapter, never a Z-value heuristic.
    """
    if 'box_center_origin' not in record or 'boxes' not in record:
        raise ValueError('Tagged boxes required; legacy producer origin is not guessed')
    source_origin = record['box_center_origin']
    result = dict(record)
    result['boxes'] = convert_origin_trial(record['boxes'],source_origin,destination_origin)
    result['box_center_origin'] = destination_origin
    result.setdefault('raw_box_center_origin',source_origin)
    return result


def annotation_components(capture):
    files = list((capture/'LIDAR_5').glob('*_Instance.txt'))
    if len(files)!=1:
        raise ValueError('One original native LiDAR Instance annotation required')
    rows, ordinals = {}, Counter()
    for line in files[0].read_text().splitlines():
        if not line.strip():
            continue
        fields = line.split()
        if len(fields)<15:
            raise ValueError('Native annotation schema differs')
        key=(int(fields[14]),fields[0]); ordinal=ordinals[key]; ordinals[key]+=1
        rows[f'{key[0]}:{key[1]}:{ordinal}'] = np.asarray([float(v) for v in fields[1:10]])
    return rows, files[0]


def source_line_refs():
    refs = [('decoder_bottom_conversion','third_party/mmdetection3d/projects/BEVFusion/bevfusion/utils.py',88),
            ('boxes_default_origin','third_party/mmdetection3d/mmdet3d/structures/bbox_3d/base_box3d.py',48),
            ('runtime_boxes_type','docker/bevfusion/app/online_preprocessor.py',154),
            ('head_boxes_type','third_party/mmdetection3d/projects/BEVFusion/bevfusion/transfusion_head.py',484),
            ('app_raw_tensor','docker/bevfusion/app/bevfusion_model.py',195),
            ('app_export_center','docker/bevfusion/app/bevfusion_model.py',207),
            ('ros_object_pose','base_ws/src/perception_object_pipeline/src/perception_object_pipeline/detected_objects.py',73),
            ('ros_marker_cube','base_ws/src/perception/bevfusion_ros/scripts/bevfusion_ros_bridge.py',811),
            ('audit_export_gt_bottom','scripts/evaluation/experiment_base5_proposal_peaks.py',54),
            ('native_physical_center_validation','perception/detection3d/bevfusion/tools/audit_base5_upright_targets.py',69)]
    return {name:dict(path=path,line=line,sha256=digest(ROOT/path)) for name,path,line in refs}


def audit(args):
    if args.output.exists():
        raise FileExistsError('Fresh origin audit output required')
    payload = json.load(gzip.open(args.predictions,'rt'))
    infos = pickle.loads(args.info.read_bytes())
    if tuple(infos['metainfo']['classes']) != ('vehicle','pedestrian','obstacle','animal'):
        raise ValueError('Original native class definition differs')
    originals = {row['token']:row for row in infos['data_list']}
    native = [row for row in payload['rows'] if row['source']=='native']
    if len(native)!=len(originals) or {r['token'] for r in native}!=set(originals):
        raise ValueError('Complete native validation token identity required')
    measurements, values = [], defaultdict(lambda:defaultdict(list))
    verification = Counter()
    annotation_sha, audit_sha = {}, {}
    for raw in native:
        info = originals[raw['token']]
        capture = args.data/'data'/raw['token']
        components = {c['component_id']:c for c in json.loads((capture/'audit.json').read_text())['components']}
        text, text_path = annotation_components(capture)
        annotation_sha[raw['token']]=digest(text_path)
        audit_sha[raw['token']]=digest(capture/'audit.json')
        for i,instance in enumerate(info['instances']):
            component = components[instance['component_id']]
            written = text[instance['component_id']]
            if component['parent_uid']!=instance['parent_uid'] or component['asset']!=instance['asset']:
                raise ValueError('Native actor/component identity differs')
            np.testing.assert_allclose(written,np.r_[component['center_lidar_m'],component['rotation_rpy'],component['size_m']],rtol=1.e-7,atol=1.e-6)
            np.testing.assert_allclose(instance['bbox_3d'][:3],written[:3],rtol=1.e-7,atol=1.e-6)
            physical=np.asarray(instance['bbox_3d'],dtype=float)
            exported=np.asarray(raw['gt_boxes'][i],dtype=float)
            np.testing.assert_allclose(convert_origin_trial(exported[None,:],'lidar_bottom_v1','gravity_center_v1')[0],physical,rtol=0,atol=1.e-7)
            if raw['gt_assets'][i]!=instance['asset'] or raw['gt_labels'][i]!=(1 if instance['bbox_label_3d']==3 else instance['bbox_label_3d']):
                raise ValueError('Exported merged class/asset identity differs')
            verification['native_gt_center_verified']+=1
        row=prepared(raw,True)
        row['gt_physical_center_boxes']=convert_origin_trial(row['gt_boxes'],'lidar_bottom_v1','gravity_center_v1')
        for label,name in enumerate(CLASSES):
            for g,p in eligible_matches(row,label).items():
                gt,pred=np.asarray(row['gt_physical_center_boxes'][g]),np.asarray(row['pred_boxes'][p])
                gravity=convert_origin_trial(pred[None,:],'lidar_bottom_v1','gravity_center_v1')[0]
                asset=str(row['gt_assets'][g])
                item=dict(token=raw['token'],cls=name,asset=asset,gt_physical_center=gt[:3].tolist(),
                    pred_bottom_center=pred[:3].tolist(),pred_gravity_center_trial=gravity[:3].tolist(),
                    xy_error_m=float(np.linalg.norm(pred[:2]-gt[:2])),gt_upright_height_m=float(gt[5]),pred_height_m=float(pred[5]),
                    bottom_as_physical_z_signed_error_m=float(pred[2]-gt[2]),bottom_as_physical_z_abs_error_m=float(abs(pred[2]-gt[2])),
                    gravity_trial_z_signed_error_m=float(gravity[2]-gt[2]),gravity_trial_z_abs_error_m=float(abs(gravity[2]-gt[2])),
                    height_signed_error_m=float(pred[5]-gt[5]),height_abs_error_m=float(abs(pred[5]-gt[5])),
                    predicted_origin_shift_m=float(pred[5]*.5),gt_points=int(row['gt_points'][g]))
                measurements.append(item)
                for group in ('all',name,name+'/asset/'+asset):
                    for key,value in item.items():
                        if key.endswith('_m') and isinstance(value,(int,float)):
                            values[group][key].append(value)
    report=dict(schema='base5_box_origin_native_annotation_audit_v1',predictions_sha256=digest(args.predictions),
        info_sha256=digest(args.info),native_frames=len(native),verification=dict(verification),matched_gt=len(measurements),
        source_line_refs=source_line_refs(),annotation_sha256=annotation_sha,audit_sha256=audit_sha,
        match_contract=dict(planar_same_class_gate_m=1.,class_thresholds=dict(zip(CLASSES,THRESHOLDS)),floor=.1,top100=True,nms_iou=.5),
        comparison=dict(current='Raw prediction bottom-center erroneously interpreted as physical center',
            candidate='Raw prediction bottom-center plus predicted height/2, output adapter trial only',
            physical_gt='Original native Instance text verified against component audit and upright infos gravity center'),
        scopes=['Native matched GT only; matching is fixed by XY/score/classes, not Z.',
            'Upright height is the native RPY enclosure used by training; raw local asset height is not substituted.',
            'No raw export/GT/weights/ROI/default/ROS code change. No learning claim or lost/unmatched-object adjudication.',
            'Both raw predictions and author_row GT use bottom center internally; existing XY recall/duplicates remain valid.'],
        distributions={group:{key:distribution(numbers) for key,numbers in metrics.items()} for group,metrics in values.items()},
        per_match=measurements,production_changed=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),native_frames=len(native),matched_gt=len(measurements),
        classes={name:{key:value for key,value in report['distributions'][name].items() if key in ['bottom_as_physical_z_abs_error_m','gravity_trial_z_abs_error_m','height_abs_error_m']} for name in CLASSES}),indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('predictions','info','data','output'):
        p.add_argument('--'+key,type=Path,required=True)
    audit(p.parse_args())
