#!/usr/bin/env python3
"""CPU-only all-GT E12 eager/TRT native-val AP comparison.

Consumes immutable exports from infer_base5_execution_candidate.py. This does
not run inference, change thresholds, prune predictions with GT, apply the
LiDAR>=1 ablation, or compare runtime AMP with historical FP32 exports.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import pickle
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'perception/detection3d/bevfusion/tools'))
from compare_base5_common30 import gt_identity, validate_payload

METRIC_PATH = ROOT/'perception/detection3d/common/evaluation/morai_detection.py'
SPEC = importlib.util.spec_from_file_location('_e12_native_pair_canonical_metric', METRIC_PATH)
METRIC = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = METRIC
SPEC.loader.exec_module(METRIC)
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
E12_SHA256 = 'f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8'
ROI = [-100., -30., -3., 100., 30., 2.]
BANDS = [(i,i+10) for i in range(0,100,10)]


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''):
            h.update(block)
    return h.hexdigest()


def write_new(path, payload):
    with Path(path).open('x') as stream:
        json.dump(payload,stream,indent=2,allow_nan=False)
        stream.write('\n')


def arrays(row):
    return dict(token=row['token'],
        gt_boxes=np.asarray(row['gt_boxes'],dtype=float).reshape(-1,7),
        gt_labels=np.asarray(row['gt_labels'],dtype=np.int64),
        gt_points=np.asarray(row['gt_points'],dtype=np.int64),
        pred_boxes=np.asarray(row['pred_boxes'],dtype=float).reshape(-1,7),
        pred_labels=np.asarray(row['pred_labels'],dtype=np.int64),
        pred_scores=np.asarray(row['pred_scores'],dtype=float))


def filtered(rows, band=None, side=None):
    """Exactly the canonical ROI/radial convention, preserving support fields."""
    result = []
    for row in rows:
        out = dict(row)
        for prefix in ('gt','pred'):
            boxes = row[prefix+'_boxes']
            mask = ((boxes[:,:3] >= ROI[:3]) & (boxes[:,:3] <= ROI[3:])).all(axis=1)
            if band is not None:
                distance = np.linalg.norm(boxes[:,:2],axis=1)
                mask &= (distance >= band[0]) & (distance < band[1])
            if side is not None:
                mask &= boxes[:,0] >= 0 if side == 'front' else boxes[:,0] < 0
            for key in row:
                if key.startswith(prefix+'_'): out[key] = row[key][mask]
        result.append(out)
    return result


def ap(rows):
    values = METRIC._evaluate_center(rows,list(CLASSES),METRIC.DEFAULT_CENTER_THRESHOLDS,
        2.,.1,.1,'center',include_tp=False,include_operating_points=False)
    classes, present = {}, []
    for label,name in enumerate(CLASSES):
        count = sum(int(np.count_nonzero(row['gt_labels'] == label)) for row in rows)
        assert count == int(values['center/'+name+'_GT'])
        points = [point for row in rows for point in row['gt_points'][row['gt_labels'] == label]]
        mean = values['center/'+name+'_mean_AP'] if count else None
        if mean is not None: present.append(mean)
        classes[name] = dict(GT=count, zero_point_GT=sum(int(value == 0) for value in points),
            predictions=sum(int(np.count_nonzero(row['pred_labels'] == label)) for row in rows),
            mean_AP=mean, AP_by_gate_m={str(float(gate)):values['center/%s_AP_%gm'%(name,gate)] if count else None
                for gate in METRIC.DEFAULT_CENTER_THRESHOLDS})
    return dict(frames=len(rows),empty_GT_frames=sum(len(row['gt_labels']) == 0 for row in rows),
        classes=classes, present_classes_mAP=float(np.mean(present)) if present else None,
        configured_three_class_mAP=float(values['center/mAP']))


def score(rows):
    return dict(whole_ROI=ap(filtered(rows)),
        ranges={'%g_%gm'%band:ap(filtered(rows,band)) for band in BANDS},
        directional_far={side:{'%g_%gm'%band:ap(filtered(rows,band,side))
            for band in ((80,90),(90,100),(80,100))} for side in ('front','rear')})


def delta(reference,candidate):
    result = dict(classes={})
    assert reference['frames'] == candidate['frames']
    for name in CLASSES:
        old,new = reference['classes'][name],candidate['classes'][name]
        assert old['GT'] == new['GT'] and old['zero_point_GT'] == new['zero_point_GT']
        result['classes'][name] = dict(GT=old['GT'],zero_point_GT=old['zero_point_GT'],
            prediction_count_change=new['predictions']-old['predictions'],
            mean_AP_delta_pp=100*(new['mean_AP']-old['mean_AP']) if old['mean_AP'] is not None else None,
            AP_by_gate_delta_pp={gate:100*(new['AP_by_gate_m'][gate]-value) if value is not None else None
                for gate,value in old['AP_by_gate_m'].items()})
    result['configured_three_class_mAP_delta_pp'] = 100*(candidate['configured_three_class_mAP']-reference['configured_three_class_mAP'])
    return result


def load(folder,backend,authored,info_sha,pins):
    required = {name:folder/name for name in ('predictions.json.gz','receipt.json','COMPLETE.json','STARTED.json')}
    for path in required.values():
        if not path.is_file(): raise FileNotFoundError('Await complete native export: '+str(path))
        pins[str(path)] = sha(path)
    payload = json.load(gzip.open(required['predictions.json.gz'],'rt'))
    receipt = json.loads(required['receipt.json'].read_text())
    complete = json.loads(required['COMPLETE.json'].read_text())
    started = json.loads(required['STARTED.json'].read_text())
    validate_payload(payload)
    assert receipt['complete'] and complete['complete'] and receipt['source_and_inputs_unchanged']
    assert complete['receipt_sha256'] == pins[str(required['receipt.json'])]
    assert complete['predictions_sha256'] == receipt['predictions_sha256'] == pins[str(required['predictions.json.gz'])]
    assert receipt['backend'] == started['backend'] == backend
    assert payload['checkpoint_sha256'] == receipt['checkpoint_sha256'] == E12_SHA256
    assert payload['frames'] == receipt['frames'] == complete['frames'] == len(authored) == 4614
    assert payload['gt_identity'] == receipt['gt_identity'] == complete['gt_identity']
    assert payload['info_sha256'] == receipt['info_sha256'] == info_sha
    assert payload['strict_state_load'] and payload['point_intensity_scale'] == 1.
    options = payload['inference_optimization']
    assert options['compile_parts'] == [] and options['parallel_camera'] is True
    assert options['backend'] == backend
    assert payload['inference_precision'].startswith('AMP;')
    contract = started['contract']
    assert contract['amp'] and contract['parallel_camera'] and not contract['compile_camera']
    assert contract['raw_proposals'] == 200 and contract['native_intensity_scale'] == 1.
    assert not any(contract[key] for key in ('ros_score_floor','ros_top100','ros_rotated_nms','gt_support_filter'))
    assert contract['empty_gt_frames_preserved'] and not receipt['gt_matching_performed']
    assert [str(row['token']) for row in authored] == [row['token'] for row in payload['rows']]
    for source,row in zip(authored,payload['rows']):
        boxes = np.asarray([item['bbox_3d'] for item in source['instances']],dtype=np.float32).reshape(-1,7)
        boxes[:,2] -= boxes[:,5]/2
        assert np.allclose(boxes,np.asarray(row['gt_boxes']).reshape(-1,7),rtol=0,atol=1e-5)
        assert row['gt_labels'] == [item['bbox_label_3d'] for item in source['instances']]
        assert row['gt_points'] == [item['num_lidar_pts'] for item in source['instances']]
        assert row['gt_assets'] == [item.get('asset','legacy_unknown') for item in source['instances']]
        assert len(row['pred_labels']) <= 200
    rows = [arrays(row) for row in payload['rows']]
    # Independently prove our ROI/radial masks equal the canonical operators.
    roi_rows = filtered(rows)
    canonical_roi = METRIC._filter_center_box_range(rows,ROI)
    for actual,expected in zip(roi_rows,canonical_roi):
        for key,value in expected.items(): assert np.array_equal(actual[key],value)
    for band in BANDS:
        canonical = METRIC._filter_range(canonical_roi,band)
        for actual,expected in zip(filtered(rows,band),canonical):
            for key,value in expected.items(): assert np.array_equal(actual[key],value)
    return rows,payload,dict(backend=backend,receipt=receipt,contract=contract,
        source_sensor_integrity=started['sensor_integrity'],original_empty_GT_frames=sum(not row['instances'] for row in authored),
        canonical_mask_checks=True, all_original_GT_support_and_frames_preserved=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--info',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check-ready',action='store_true',help='Print required-file readiness only; do not load/score/create results')
    a = p.parse_args()
    expected = [a.info,*[folder/name for folder in (a.reference,a.candidate)
        for name in ('predictions.json.gz','receipt.json','COMPLETE.json','STARTED.json')]]
    if a.check_ready:
        missing = [str(path) for path in expected if not path.is_file()]
        print(json.dumps(dict(ready=not missing,missing=missing,CPU_only=True),indent=2))
        return
    if a.output.exists(): p.error('Fresh AP result required')
    pins = {str(path):sha(path) for path in (Path(__file__),METRIC_PATH,
        ROOT/'perception/detection3d/bevfusion/tools/compare_base5_common30.py',a.info)}
    authored_info = pickle.loads(a.info.read_bytes())
    assert tuple(authored_info['metainfo']['classes']) == CLASSES
    assert list(authored_info['metainfo']['point_cloud_range']) == ROI
    authored = authored_info['data_list']; info_sha = pins[str(a.info)]
    began = time.monotonic()
    models, payloads, inputs = {}, {}, {}
    # Load both first: reject mixed precision/config/ordered GT before scoring.
    for name,folder,backend in (('eager',a.reference,'eager'),('camera_trt',a.candidate,'camera_trt')):
        rows,payload,contract = load(folder,backend,authored,info_sha,pins)
        models[name] = dict(contract=contract); payloads[name] = payload; inputs[name] = rows
    for field in ('gt_identity','info_sha256','checkpoint_sha256','config_sha256','script_sha256',
                  'inference_precision','input_pipeline','point_intensity_scale'):
        assert payloads['eager'][field] == payloads['camera_trt'][field], 'Pair contract mismatch: '+field
    assert models['eager']['contract']['contract'] == models['camera_trt']['contract']['contract']
    for name,rows in inputs.items():
        models[name]['evaluation'] = score(rows)
        print(json.dumps(dict(event='full_native_model_scored',model=name,
            mAP=models[name]['evaluation']['whole_ROI']['configured_three_class_mAP'])),flush=True)
    ref,new = models['eager']['evaluation'],models['camera_trt']['evaluation']
    changes = dict(whole_ROI=delta(ref['whole_ROI'],new['whole_ROI']),
        ranges={name:delta(ref['ranges'][name],new['ranges'][name]) for name in ref['ranges']},
        directional_far={side:{name:delta(ref['directional_far'][side][name],new['directional_far'][side][name])
            for name in ref['directional_far'][side]} for side in ('front','rear')})
    assert pins == {path:sha(path) for path in pins}, 'Scorer input/source changed'
    result = dict(schema='base5_e12_runtime_eager_trt_native_AP_pair_v1',complete=True,
        models=models,delta_trt_minus_eager=changes,source_and_input_sha256=pins,
        seconds=time.monotonic()-began,evaluation_contract=dict(ROI=ROI,
            AP_center_gates_m=[.5,1.,2.,4.],min_recall=.1,min_precision=.1,
            matching='Canonical score-ranked greedy XY center matching; all GT including zero-LiDAR-point GT retained',
            range='Half-open radial bins; GT and predictions filtered before matching, original canonical convention',
            directions='front x>=0 / rear x<0 in LiDAR coordinates; radial distance bins',
            empty_GT_frames='All 4614 original frames retained, including 232 authored empty-GT frames',
            absent_class='mean_AP and individual gates null, not zero or perfect; actual prediction count reported independently',
            execution='Same E12 AMP+parallel, same TF32/cuDNN setting, no historical FP32 comparison',
            postprocessing='Raw native decoded <=200 queries; no ROS floor/top100/NMS or GT support pruning'),
        acceptance=dict(numerical_identity_required=False,
            interpretation='Report measured AP deltas/counts with separately measured speed; small deviations are a trade-off, not automatic rejection or automatic production approval',
            AP_does_not_certify='Continuous tracker association, input freshness, 24GB full-stack latency or safety'))
    write_new(a.output,result)
    print(json.dumps(dict(complete=True,output=str(a.output),whole_ROI_delta=changes['whole_ROI'])),flush=True)


if __name__ == '__main__': main()
