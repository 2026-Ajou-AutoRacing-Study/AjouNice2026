#!/usr/bin/env python3
"""CPU-only all-GT E12 execution-pair native-val AP comparison.

Consumes immutable exports from infer_base5_execution_candidate.py. This does
not run inference, change thresholds, prune predictions with GT, apply the
LiDAR>=1 ablation, or compare runtime AMP with historical FP32 exports.
Pairs are the original eager/camera-TRT comparison or a fresh same-camera-TRT
reference versus its explicitly qualified SDPA+position-cache head candidate.
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
PAIRS = {('eager','camera_trt'), ('camera_trt','camera_trt_head_sdpa_position')}
HEAD_FLAGS = dict(proposal_topk=False,attention_sdpa=True,cache_bev_pos=True)
HEAD_FORWARD_AST = 'ff24ebde9a33a958ee25859d58b2b75b58d039a6d5bc7f21d5b0f2569e5c0eef'
HEAD_SOURCE_SHA = '7712b2c65649f2fcd5f9bd852a8f4406e4081af8e9137001a565261d9d3699f1'


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


def _canonical_digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def _source_inventory(pins):
    """Only the one per-export resolved config has a mode-specific path."""
    if not isinstance(pins,dict) or not pins:
        raise ValueError('Nonempty actual source inventory required')
    resolved = [name for name in pins if Path(name).name == 'resolved_config.py']
    if len(resolved) != 1: raise ValueError('Exactly one per-export resolved_config.py source required')
    result = dict(pins)
    result['<export-output>/resolved_config.py'] = result.pop(resolved[0])
    return result


def validate_pair_contract(reference,candidate,reference_contract,candidate_contract,
                           reference_backend='eager',candidate_backend='camera_trt'):
    """Reject mixed execution populations BEFORE doing any metric work.

    Old eager/TRT validation stays compatible. The new camera-TRT/head pair
    additionally requires a fresh SAME exporter/source/engine pair, not an old
    receipt relabelled or edited to bypass a provenance mismatch.
    """
    pair = (reference_backend,candidate_backend)
    if pair not in PAIRS: raise ValueError('Unsupported explicit backend pair')
    for field in ('gt_identity','info_sha256','checkpoint_sha256','config_sha256','script_sha256',
                  'inference_precision','input_pipeline','point_intensity_scale'):
        if reference[field] != candidate[field]:
            raise ValueError('Pair contract mismatch: '+field)
    if reference_contract['contract'] != candidate_contract['contract']:
        raise ValueError('Pair common execution contract differs')
    if pair == ('eager','camera_trt'):
        return dict(pair=list(pair),scope='Existing matched eager/camera TensorRT contract')
    for field in ('classes','gpu','torch_version','torch_cuda','strict_state_load',
                  'point_intensity_adapter','raw_prediction_contract',
                  'inference_timing_scope','inference_timing_warmup_excluded'):
        if reference[field] != candidate[field]:
            raise ValueError('Head pair non-mode runtime field differs: '+field)
    ref_options,new_options = dict(reference['inference_optimization']),dict(candidate['inference_optimization'])
    if ref_options.pop('backend') != reference_backend or new_options.pop('backend') != candidate_backend:
        raise ValueError('Head pair backend provenance differs')
    if ref_options.pop('head_runtime',None) is not None:
        raise ValueError('Head reference must explicitly use the original head')
    head = new_options.pop('head_runtime',None)
    if not isinstance(head,dict) or head.get('schema') != 'base5_head_runtime_optimization_v1':
        raise ValueError('Candidate audited head report missing')
    for name,value in HEAD_FLAGS.items():
        if head.get(name) is not value: raise ValueError('Unexpected head candidate option: '+name)
    if head.get('upstream_forward_ast_sha256') != HEAD_FORWARD_AST or head.get('attention_modules') != 2:
        raise ValueError('Head source/attention module contract differs')
    if ref_options != new_options:
        raise ValueError('Non-head inference optimization options differ')
    off_flags = {name:False for name in HEAD_FLAGS}
    if reference_contract.get('mode_head_runtime') != off_flags:
        raise ValueError('Reference STARTED head mode is not explicitly off')
    if candidate_contract.get('mode_head_runtime') != HEAD_FLAGS:
        raise ValueError('Candidate STARTED head mode differs from the only allowed candidate')
    ref_engine,new_engine = reference_contract.get('engine_identity'),candidate_contract.get('engine_identity')
    if not isinstance(ref_engine,dict) or not ref_engine or ref_engine != new_engine:
        raise ValueError('Head pair must use the same complete camera engine identity')
    binary_sha = ref_engine.get('engine',{}).get('sha256')
    if not isinstance(binary_sha,str) or len(binary_sha) != 64 or any(c not in '0123456789abcdef' for c in binary_sha):
        raise ValueError('Head pair camera engine binary SHA missing/invalid')
    if (ref_engine.get('contract',{}).get('checkpoint_sha256') != E12_SHA256 or
            ref_engine.get('contract',{}).get('config_sha256') != reference['config_sha256']):
        raise ValueError('Camera engine does not bind the selected E12/config')
    ref_source = _source_inventory(reference_contract['receipt']['actual_source_sha256'])
    new_source = _source_inventory(candidate_contract['receipt']['actual_source_sha256'])
    if ref_source != new_source:
        raise ValueError('Head pair actual source inventories differ beyond output config path')
    head_pins = [digest for name,digest in ref_source.items() if name.endswith('/bevfusion/transfusion_head.py')]
    if not head_pins or set(head_pins) != {HEAD_SOURCE_SHA}:
        raise ValueError('Audited upstream head source SHA missing/different')
    for contract in (reference_contract,candidate_contract):
        if contract['source_sensor_integrity'].get('mode') != 'sha256':
            raise ValueError('Fresh head pair requires sensor content SHA snapshots')
        if (not contract.get('source_pins_match_start') or not contract.get('control_pins_match_start') or
                not contract.get('sensor_inventory_complete') or not contract.get('sensor_aliases_present')):
            raise ValueError('Head pair STARTED/source/control/sensor provenance incomplete')
    if (reference_contract.get('source_sensor_inventory_sha256') != candidate_contract.get('source_sensor_inventory_sha256') or
            reference_contract.get('sensor_path_aliases_sha256') != candidate_contract.get('sensor_path_aliases_sha256')):
        raise ValueError('Head pair native sensor inventory/aliases differ')
    return dict(pair=list(pair),only_head_delta=HEAD_FLAGS,
        same_camera_engine=True,same_actual_source_except_output_config_path=True,
        same_sensor_content_snapshots_and_aliases=True,numerical_identity_required=False)


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
    initial_sources,actual_sources = started.get('source_sha256',{}),receipt.get('actual_source_sha256',{})
    sensors,aliases = started.get('source_sensor_files'),started.get('sensor_path_aliases')
    return rows,payload,dict(backend=backend,receipt=receipt,contract=contract,
        source_sensor_integrity=started['sensor_integrity'],original_empty_GT_frames=sum(not row['instances'] for row in authored),
        canonical_mask_checks=True, all_original_GT_support_and_frames_preserved=True,
        engine_identity=started.get('engine_identity'),mode_head_runtime=started.get('mode_head_runtime'),
        source_pins_match_start=bool(initial_sources) and all(actual_sources.get(path) == digest for path,digest in initial_sources.items()),
        control_pins_match_start=bool(started.get('control_sha256')) and started['control_sha256'] == receipt.get('control_sha256'),
        sensor_inventory_complete=isinstance(sensors,dict) and bool(sensors) and all(
            isinstance(value,dict) and isinstance(value.get('sha256'),str) and len(value['sha256']) == 64
            for value in sensors.values()),
        sensor_aliases_present=isinstance(aliases,dict) and bool(aliases),
        source_sensor_inventory_sha256=_canonical_digest(sensors),
        sensor_path_aliases_sha256=_canonical_digest(aliases))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--info',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reference-backend',choices=['eager','camera_trt'],default='eager')
    p.add_argument('--candidate-backend',choices=['camera_trt','camera_trt_head_sdpa_position'],default='camera_trt')
    p.add_argument('--check-ready',action='store_true',help='Print required-file readiness only; do not load/score/create results')
    a = p.parse_args()
    pair = (a.reference_backend,a.candidate_backend)
    if pair not in PAIRS: p.error('Only eager/camera_trt or camera_trt/camera_trt_head_sdpa_position is allowed')
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
    for name,folder,backend in ((a.reference_backend,a.reference,a.reference_backend),
                                (a.candidate_backend,a.candidate,a.candidate_backend)):
        rows,payload,contract = load(folder,backend,authored,info_sha,pins)
        models[name] = dict(contract=contract); payloads[name] = payload; inputs[name] = rows
    pair_audit = validate_pair_contract(payloads[a.reference_backend],payloads[a.candidate_backend],
        models[a.reference_backend]['contract'],models[a.candidate_backend]['contract'],*pair)
    for name,rows in inputs.items():
        models[name]['evaluation'] = score(rows)
        print(json.dumps(dict(event='full_native_model_scored',model=name,
            mAP=models[name]['evaluation']['whole_ROI']['configured_three_class_mAP'])),flush=True)
    ref,new = models[a.reference_backend]['evaluation'],models[a.candidate_backend]['evaluation']
    changes = dict(whole_ROI=delta(ref['whole_ROI'],new['whole_ROI']),
        ranges={name:delta(ref['ranges'][name],new['ranges'][name]) for name in ref['ranges']},
        directional_far={side:{name:delta(ref['directional_far'][side][name],new['directional_far'][side][name])
            for name in ref['directional_far'][side]} for side in ('front','rear')})
    assert pins == {path:sha(path) for path in pins}, 'Scorer input/source changed'
    original_pair = pair == ('eager','camera_trt')
    result = dict(schema=('base5_e12_runtime_eager_trt_native_AP_pair_v1' if original_pair else
                          'base5_e12_runtime_camera_head_native_AP_pair_v1'),complete=True,
        models=models,source_and_input_sha256=pins,
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
    result['delta_trt_minus_eager' if original_pair else 'delta_head_minus_camera_trt'] = changes
    result['pair_audit'] = pair_audit
    write_new(a.output,result)
    print(json.dumps(dict(complete=True,output=str(a.output),whole_ROI_delta=changes['whole_ROI'])),flush=True)


if __name__ == '__main__': main()
