"""Small CPU-only canonical-AP population and boundary regressions."""
import importlib.util
from copy import deepcopy
import gzip
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('_e12_pair_scorer_tests',
    ROOT/'perception/detection3d/bevfusion/scripts/score_base5_execution_pair.py')
S = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(S)


def row(token, gt, pred, points=None, pred_labels=None):
    return dict(token=token, gt_boxes=np.asarray(gt,float).reshape(-1,7),
        gt_labels=np.zeros(len(gt),np.int64),gt_points=np.asarray(points if points is not None else [1]*len(gt)),
        pred_boxes=np.asarray(pred,float).reshape(-1,7),
        pred_labels=np.asarray(pred_labels if pred_labels is not None else [0]*len(pred),np.int64),
        pred_scores=np.full(len(pred),.9))


def box(x,y=0.,z=0.): return [x,y,z,1.,1.,1.,0.]


class PairScorerTests(unittest.TestCase):
    def test_zero_point_GT_and_empty_frames_retained(self):
        rows = [row('empty',[],[]),row('point0',[box(10)],[box(10)],[0]),
                row('point1',[box(20)],[box(20)],[1])]
        result = S.ap(S.filtered(rows))
        self.assertEqual(result['frames'],3)
        self.assertEqual(result['empty_GT_frames'],1)
        self.assertEqual(result['classes']['vehicle']['GT'],2)
        self.assertEqual(result['classes']['vehicle']['zero_point_GT'],1)
        self.assertGreater(result['classes']['vehicle']['mean_AP'],.95)

    def test_absent_GT_is_null_but_output_count_is_real(self):
        rows = [row('empty',[],[box(5)],pred_labels=[1])]
        result = S.ap(rows)
        pedestrian = result['classes']['pedestrian']
        self.assertEqual(pedestrian['GT'],0)
        self.assertEqual(pedestrian['predictions'],1)
        self.assertIsNone(pedestrian['mean_AP'])
        self.assertTrue(all(value is None for value in pedestrian['AP_by_gate_m'].values()))
        self.assertIsNone(result['present_classes_mAP'])

    def test_radial_halfopen_and_directional_boundaries(self):
        rows = [row('boundaries',[box(80),box(90),box(100),box(-80),box(84,10)],
                    [box(80),box(90),box(100),box(-80),box(84,10)])]
        self.assertEqual(len(S.filtered(rows,(80,90),'front')[0]['gt_boxes']),2)
        self.assertEqual(len(S.filtered(rows,(90,100),'front')[0]['gt_boxes']),1)
        self.assertEqual(len(S.filtered(rows,(80,90),'rear')[0]['gt_boxes']),1)
        self.assertEqual(len(S.filtered(rows)[0]['gt_boxes']),5)  # ROI x100 inclusive
        zero = [row('xzero',[box(0,20)],[box(0,20)])]
        self.assertEqual(len(S.filtered(zero,(0,30),'front')[0]['gt_boxes']),1)
        self.assertEqual(len(S.filtered(zero,(0,30),'rear')[0]['gt_boxes']),0)

    def test_filters_match_canonical_ROI_and_ranges(self):
        rows = [row('x',[box(10),box(100),box(100.01),box(80,31),box(-85),box(0,z=-3.01)],
                         [box(10),box(100),box(100.01),box(80,31),box(-85),box(0,z=-3.01)])]
        canonical = S.METRIC._filter_center_box_range(rows,S.ROI)
        for band in S.BANDS:
            actual = S.filtered(rows,band)
            expected = S.METRIC._filter_range(canonical,band)
            for key,value in expected[0].items(): self.assertTrue(np.array_equal(actual[0][key],value))

    def test_delta_does_not_require_numeric_identity(self):
        rows = [row('one',[box(10)],[box(10)])]
        old = S.ap(rows)
        new = S.ap([row('one',[box(10)],[box(10.6)])])
        changes = S.delta(old,new)
        self.assertLess(changes['classes']['vehicle']['mean_AP_delta_pp'],0)
        self.assertIsNone(changes['classes']['pedestrian']['mean_AP_delta_pp'])
        self.assertEqual(changes['classes']['vehicle']['GT'],1)

    def head_pair(self):
        common = dict(gt_identity='a'*64,info_sha256='b'*64,checkpoint_sha256=S.E12_SHA256,
            config_sha256='c'*64,script_sha256='d'*64,inference_precision='AMP; TF32 ON; cuDNN benchmark ON',
            input_pipeline=[dict(type='OriginalNativePipeline')],point_intensity_scale=1.,
            classes=list(S.CLASSES),gpu='NVIDIA GeForce RTX 4090',torch_version='2.7.1+cu128',
            torch_cuda='12.8',strict_state_load=True,point_intensity_adapter='Identity native 0..1',
            raw_prediction_contract='Original <=200 queries',inference_timing_scope='Same original Runner',
            inference_timing_warmup_excluded=10)
        reference,candidate = deepcopy(common),deepcopy(common)
        options = dict(compile_parts=[],folded_dense_bn=0,camera_cuda_graph=False,parallel_camera=True,
                       emulate_precision_casts=True)
        reference['inference_optimization'] = dict(options,backend='camera_trt',head_runtime=None)
        candidate['inference_optimization'] = dict(options,backend='camera_trt_head_sdpa_position',
            head_runtime=dict(schema='base5_head_runtime_optimization_v1',**S.HEAD_FLAGS,
                upstream_forward_ast_sha256=S.HEAD_FORWARD_AST,attention_modules=2))
        engine = dict(contract=dict(checkpoint_sha256=S.E12_SHA256,config_sha256='c'*64),
                      engine=dict(sha256='e'*64,precision='fp16',gpu_name='NVIDIA GeForce RTX 4090'))
        source = {'/same/exporter.py':'d'*64,
                  '/same/projects/BEVFusion/bevfusion/transfusion_head.py':S.HEAD_SOURCE_SHA}
        contracts = []
        for name,flags in [('reference',{key:False for key in S.HEAD_FLAGS}),('candidate',S.HEAD_FLAGS)]:
            contracts.append(dict(contract=dict(amp=True,parallel_camera=True,tf32=True,raw_proposals=200),
                mode_head_runtime=deepcopy(flags),engine_identity=deepcopy(engine),
                receipt=dict(actual_source_sha256=dict(source,**{'/'+name+'/resolved_config.py':'f'*64})),
                source_sensor_integrity=dict(mode='sha256'),source_sensor_inventory_sha256='1'*64,
                sensor_path_aliases_sha256='2'*64,source_pins_match_start=True,
                control_pins_match_start=True,sensor_inventory_complete=True,sensor_aliases_present=True))
        return reference,candidate,*contracts

    def validate_head_pair(self,values):
        return S.validate_pair_contract(*values,'camera_trt','camera_trt_head_sdpa_position')

    def test_explicit_fresh_same_camera_head_pair_is_allowed(self):
        result = self.validate_head_pair(self.head_pair())
        self.assertEqual(result['only_head_delta'], S.HEAD_FLAGS)
        self.assertTrue(result['same_camera_engine'])
        self.assertFalse(result['numerical_identity_required'])

    def test_existing_eager_camera_pair_retains_its_contract(self):
        reference,candidate,ref_contract,new_contract = self.head_pair()
        # Existing caller roles remain eager/camera_trt; new head-mode/source
        # requirements do not retroactively invalidate the old frozen report.
        reference['inference_optimization'].update(backend='eager',head_runtime=None)
        candidate['inference_optimization'].update(backend='camera_trt',head_runtime=None)
        result = S.validate_pair_contract(reference,candidate,ref_contract,new_contract)
        self.assertEqual(result['pair'], ['eager','camera_trt'])

    def test_only_two_explicit_backend_pairs_are_accepted(self):
        values = self.head_pair()
        for pair in [('eager','camera_trt_head_sdpa_position'),('camera_trt','camera_trt'),
                     ('pool_fused_layout','camera_trt_head_sdpa_position')]:
            with self.subTest(pair=pair),self.assertRaises(ValueError):
                S.validate_pair_contract(*values,*pair)

    def test_head_pair_rejects_changed_dataset_precision_and_exporter(self):
        for field,bad in [('checkpoint_sha256','E24'),('info_sha256','another infos'),
                          ('config_sha256','different grid'),('gt_identity','different GT'),
                          ('script_sha256','older exporter'),('point_intensity_scale',255.),
                          ('inference_precision','FP32'),('gpu','other GPU'),
                          ('input_pipeline',[dict(type='ChangedPipeline')])]:
            values = self.head_pair(); values[1][field] = bad
            with self.subTest(field=field),self.assertRaises(ValueError): self.validate_head_pair(values)

    def test_head_pair_rejects_extra_numerics_topk_compile_or_pool_delta(self):
        for field,bad in [('proposal_topk',True),('attention_sdpa',False),('cache_bev_pos',False)]:
            values = self.head_pair(); values[1]['inference_optimization']['head_runtime'][field] = bad
            with self.subTest(field=field),self.assertRaises(ValueError): self.validate_head_pair(values)
        for field,bad in [('compile_parts',['camera']),('folded_dense_bn',1),('pool_fused_layout',True)]:
            values = self.head_pair(); values[1]['inference_optimization'][field] = bad
            with self.subTest(field=field),self.assertRaises(ValueError): self.validate_head_pair(values)

    def test_head_pair_rejects_engine_sha_precision_or_checkpoint_change(self):
        for field,bad in [('sha256','0'*64),('precision','int8'),('gpu_name','other GPU')]:
            values = self.head_pair(); values[3]['engine_identity']['engine'][field] = bad
            with self.subTest(field=field),self.assertRaises(ValueError): self.validate_head_pair(values)
        values = self.head_pair()
        for contract in values[2:]: contract['engine_identity']['contract']['checkpoint_sha256'] = 'E24'
        with self.assertRaises(ValueError): self.validate_head_pair(values)

    def test_head_pair_requires_source_equality_except_resolved_output_path(self):
        values = self.head_pair()
        self.validate_head_pair(values)
        values[3]['receipt']['actual_source_sha256']['/same/exporter.py'] = '0'*64
        with self.assertRaises(ValueError): self.validate_head_pair(values)
        values = self.head_pair()
        values[3]['receipt']['actual_source_sha256']['/candidate/resolved_config.py'] = '0'*64
        with self.assertRaises(ValueError): self.validate_head_pair(values)
        values = self.head_pair()
        values[3]['receipt']['actual_source_sha256']['/extra/resolved_config.py'] = 'f'*64
        with self.assertRaises(ValueError): self.validate_head_pair(values)

    def test_head_pair_requires_source_and_sensor_snapshots_not_stat_or_missing(self):
        for field,bad in [('source_sensor_inventory_sha256','different'),
                          ('sensor_path_aliases_sha256','different'),('sensor_inventory_complete',False),
                          ('sensor_aliases_present',False),('source_pins_match_start',False),
                          ('control_pins_match_start',False)]:
            values = self.head_pair(); values[3][field] = bad
            with self.subTest(field=field),self.assertRaises(ValueError): self.validate_head_pair(values)
        values = self.head_pair(); values[3]['source_sensor_integrity']['mode'] = 'stat'
        with self.assertRaises(ValueError): self.validate_head_pair(values)

    def test_head_pair_rejects_missing_or_forged_head_provenance(self):
        for field,bad in [('upstream_forward_ast_sha256','wrong AST'),('attention_modules',1),
                          ('schema','unqualified head')]:
            values = self.head_pair(); values[1]['inference_optimization']['head_runtime'][field] = bad
            with self.subTest(field=field),self.assertRaises(ValueError): self.validate_head_pair(values)
        values = self.head_pair(); values[0]['inference_optimization']['head_runtime'] = {'attention_sdpa':True}
        with self.assertRaises(ValueError): self.validate_head_pair(values)
        values = self.head_pair(); values[3]['mode_head_runtime']['proposal_topk'] = True
        with self.assertRaises(ValueError): self.validate_head_pair(values)

    def test_complete_export_loader_retains_empty_frames_zero_GT_and_low_scores(self):
        reference,candidate,ref_contract,new_contract = self.head_pair()
        authored = [dict(token=str(i),instances=[]) for i in range(4614)]
        authored[0]['instances'] = [dict(bbox_3d=[10.,0.,1.,1.,1.,2.,0.],bbox_label_3d=0,
                                       num_lidar_pts=0,asset='fixture')]
        rows = [dict(token=str(i),gt_boxes=[],gt_labels=[],gt_points=[],gt_assets=[],
                     pred_boxes=[],pred_labels=[],pred_scores=[]) for i in range(4614)]
        rows[0].update(gt_boxes=[[10.,0.,0.,1.,1.,2.,0.]],gt_labels=[0],gt_points=[0],gt_assets=['fixture'],
                       pred_boxes=[[10.,0.,0.,1.,1.,2.,0.]],pred_labels=[0],pred_scores=[.05])
        identity = S.gt_identity(rows)
        with tempfile.TemporaryDirectory() as temporary:
            loaded = []
            for name,payload,contract in [('camera_trt',reference,ref_contract),
                    ('camera_trt_head_sdpa_position',candidate,new_contract)]:
                folder = Path(temporary)/name; folder.mkdir()
                payload.update(rows=rows,frames=4614,gt_identity=identity)
                with gzip.open(folder/'predictions.json.gz','wt') as stream: json.dump(payload,stream)
                controls = {'/frozen/info.pkl':payload['info_sha256'],'/frozen/camera.engine':'e'*64}
                sensor_files = {'/frozen/sensor.bin':dict(sha256='a'*64,size=8)}
                actual_sources = contract['receipt']['actual_source_sha256']
                started = dict(backend=name,contract=dict(amp=True,parallel_camera=True,compile_camera=False,
                    raw_proposals=200,native_intensity_scale=1.,ros_score_floor=False,ros_top100=False,
                    ros_rotated_nms=False,gt_support_filter=False,empty_gt_frames_preserved=True),
                    source_sha256=actual_sources,control_sha256=controls,
                    sensor_integrity=dict(mode='sha256'),source_sensor_files=sensor_files,
                    sensor_path_aliases={'/frozen/sensor.bin':'/frozen/sensor.bin'},
                    engine_identity=contract['engine_identity'],mode_head_runtime=contract['mode_head_runtime'])
                receipt = {key:value for key,value in payload.items() if key != 'rows'}
                receipt.update(backend=name,complete=True,source_and_inputs_unchanged=True,
                    gt_matching_performed=False,actual_source_sha256=actual_sources,control_sha256=controls,
                    predictions_sha256=S.sha(folder/'predictions.json.gz'))
                S.write_new(folder/'receipt.json',receipt)
                S.write_new(folder/'STARTED.json',started)
                S.write_new(folder/'COMPLETE.json',dict(complete=True,frames=4614,gt_identity=identity,
                    receipt_sha256=S.sha(folder/'receipt.json'),predictions_sha256=receipt['predictions_sha256']))
                arrays,parsed,audit = S.load(folder,name,authored,payload['info_sha256'],{})
                self.assertEqual(len(arrays),4614)
                self.assertEqual(sum(len(row['gt_boxes']) == 0 for row in arrays),4613)
                self.assertEqual(arrays[0]['gt_points'].tolist(),[0])
                self.assertEqual(arrays[0]['pred_scores'].tolist(),[.05])
                loaded.append((parsed,audit))
            S.validate_pair_contract(loaded[0][0],loaded[1][0],loaded[0][1],loaded[1][1],
                                     'camera_trt','camera_trt_head_sdpa_position')


if __name__ == '__main__': unittest.main()
