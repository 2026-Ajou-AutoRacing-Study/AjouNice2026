import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import report_base5_crop_height_diagnostic as diagnostic


class CropHeightDiagnosticTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def fixture(self):
        folder = self.root/'raw'/'CAMERA_2'
        folder.mkdir(parents=True)
        pixels = np.zeros((480,640,4),dtype=np.uint8)
        pixels[140:150,100:110] = [1,2,3,255]
        Image.fromarray(pixels,'RGBA').save(folder/'one_Instance.png')
        (folder/'one_BBox2D.txt').write_text('Vehicle 100 140 110 150 8501 (1, 2, 3, 255)\n')
        row = dict(scene='one',capture_root=str(folder.parent),camera='CAMERA_2',
            native_rect=[100.,140.,110.,150.],camera_instance_id=8501,raw_class='Vehicle',
            native_pixels=100,training_class='vehicle',asset='car',range_m=2.)
        mask = (pixels==[1,2,3,255]).all(axis=2).astype(np.uint8)
        prefix = np.r_[0,np.cumsum(mask.sum(axis=1),dtype=np.int64)]
        row['candidates'] = dict(bottom512w1056=diagnostic.crop.mask_statistics(mask,prefix,
            row['native_rect'],diagnostic.crop.geometry('CAMERA_2',diagnostic.PROFILES[0])))
        component = dict(component_id='8501:Vehicle:0',parent_uid=8501,data_id=1,asset='car',
            training_class='vehicle',lidar_points=0,center_lidar_m=[2.,0.,0.],range_xy_m=2.,
            range_ego_xy_m=2.,cameras=[dict(camera='CAMERA_2',native_rect=row['native_rect'],
                camera_instance_id=8501,annotation_class='Vehicle',native_instance_pixels=100)])
        return row, component

    def far_fixture(self):
        row, component = self.fixture()
        root = Path(row['capture_root'])
        plan = dict(target_uid=8501,target_data_id=1,target_name='car')
        (root/'metadata.json').write_text(json.dumps(dict(spec=dict(secondary_far_vehicle=plan))))
        audit = dict(components=[component])
        (root/'audit.json').write_text(json.dumps(audit))
        (root/'scenario.json').write_text('{}')
        (root/'sensor_reference.json').write_text('{}')
        entry = dict(id='one',capture_root=str(root),audit=audit,status='review')
        campaign = self.root/'campaign'
        ledger = campaign/'chunk_0000'/'capture'/'ledger.jsonl'
        ledger.parent.mkdir(parents=True)
        ledger.write_text(json.dumps(entry)+'\n')
        return campaign, ledger, entry

    def test_actual_online_geometry_supports512_and576_not640_or704(self):
        result = diagnostic.shape_contract()
        self.assertTrue(result['passed'])
        self.assertFalse(result['model_or_checkpoint_loaded'])
        self.assertTrue(result['existing512_rgb_identical_to_bottom512_of576'])
        self.assertEqual(result['candidates']['576']['feature_size'],[72,132])
        self.assertEqual(result['candidates']['576']['cameras']['CAMERA_1']['crop_xyxy'],[0,18,1056,594])
        for h in (512,576):self.assertTrue(result['candidates'][str(h)]['common_crop_valid'])
        for h in (640,704):self.assertFalse(result['candidates'][str(h)]['common_crop_valid'])

    def test_runtime_options_reject_mismatched_feature_shape(self):
        _, options = diagnostic.runtime_geometry_functions()
        config = dict(test_dataloader=dict(dataset=dict(pipeline=[dict(type='MoraiImageAug3D',
            final_dim=[576,1056],resize_lim=[.825,.825])])),point_cloud_range=[0]*6,
            model=dict(view_transform=dict(image_size=[576,1056],feature_size=[64,132])))
        with self.assertRaisesRegex(ValueError,'stride-8'):options(config)

    def test_actual_mask_restores_upper_pixels_and_validates512_cache(self):
        row,_ = self.fixture();snapshot=copy.deepcopy(row);pins={}
        metrics,counts = diagnostic.measure([row],pins,cached512=True)
        self.assertEqual(counts['cached512_exact_matches'],1)
        self.assertEqual(metrics[0]['h512']['native_fraction'],0.)
        self.assertEqual(metrics[0]['h576']['native_fraction'],1.)
        self.assertFalse(metrics[0]['h512']['candidate_camera_gate'])
        self.assertTrue(metrics[0]['h576']['candidate_camera_gate'])
        self.assertEqual(row,snapshot)
        diagnostic.verify_pins(pins)

    def test_native_count_or_cached_metric_tamper_halts(self):
        row,_ = self.fixture()
        row['native_pixels']=99
        with self.assertRaisesRegex(RuntimeError,'pixel count'):diagnostic.measure([row],{},True)
        row['native_pixels']=100
        row['candidates']['bottom512w1056']['native_fraction']=.1
        with self.assertRaisesRegex(RuntimeError,'exactly match'):diagnostic.measure([row],{},True)

    def test_missing_annotation_identity_halts(self):
        row,_ = self.fixture();row['camera_instance_id']=8502
        with self.assertRaisesRegex(RuntimeError,'annotation identity'):diagnostic.measure([row],{})

    def test_unique_gt_gates_or_lidar_without_overcounting_views(self):
        row,component=self.fixture();metrics,_=diagnostic.measure([row],{})
        report=diagnostic.component_summary([('one',component)],[row],metrics)['by_class']['vehicle']
        self.assertEqual(report['camera_observed_unambiguous_components'],1)
        self.assertEqual(report['gained_roi_sensor_positive'],1)
        component['lidar_points']=1
        report=diagnostic.component_summary([('one',component)],[row],metrics)['by_class']['vehicle']
        self.assertEqual(report['gained_roi_sensor_positive'],0)
        self.assertEqual(report['profiles']['h512']['sensor_positive'],1)
        self.assertEqual(report['profiles']['h576']['sensor_positive'],1)

    def test_ambiguous_components_do_not_gain_synthetic_camera_evidence(self):
        row,component=self.fixture();metrics,_=diagnostic.measure([row],{})
        component['camera_component_ambiguous']=True
        report=diagnostic.component_summary([('one',component)],[row],metrics)
        self.assertEqual(report['by_class'],{})
        self.assertEqual(report['ambiguous_components_excluded_from_join'],1)

    def test_component_join_refuses_missing_other_visible_camera(self):
        row,component=self.fixture();metrics,_=diagnostic.measure([row],{})
        evidence=copy.deepcopy(component['cameras'][0]);evidence['camera']='CAMERA_1'
        component['cameras'].append(evidence)
        with self.assertRaisesRegex(RuntimeError,'partial camera views'):
            diagnostic.component_summary([('one',component)],[row],metrics)

    def test_summary_keeps_classes_and_observation_denominators_separate(self):
        row,_=self.fixture();metrics,_=diagnostic.measure([row],{})
        summary=diagnostic.summaries([row],metrics)
        self.assertEqual(len(summary),1)
        self.assertEqual(summary[0]['training_class'],'vehicle')
        self.assertEqual(summary[0]['n'],1)
        self.assertEqual(summary[0]['improved_observations'],1)

    def test_current_files_are_rehashed_not_only_parsed_from_cache(self):
        path=self.root/'input.json';path.write_text('{}');pins={}
        diagnostic.pin(pins,path);path.write_text('{"changed":true}')
        with self.assertRaisesRegex(RuntimeError,'hash changed'):diagnostic.verify_pins(pins)

    def test_far_initial_partial_or_duplicate_campaign_is_not32(self):
        campaign,ledger,entry=self.far_fixture()
        with self.assertRaisesRegex(RuntimeError,'complete initial far32'):
            diagnostic.far_diagnostic(campaign,{})
        ledger.write_text((json.dumps(entry)+'\n')*2)
        with self.assertRaisesRegex(RuntimeError,'Repeated initial far source'):
            diagnostic.far_diagnostic(campaign,{})

    def test_far_raw_audit_or_native_identity_tamper_halts(self):
        campaign,_,entry=self.far_fixture()
        (Path(entry['capture_root'])/'audit.json').write_text('{}')
        with self.assertRaisesRegex(RuntimeError,'differs from its closed'):
            diagnostic.far_diagnostic(campaign,{})

    def test_scope_cannot_expand_silently_to_all_ranges(self):
        for maximum in (0,6):
            with self.assertRaises(ValueError):diagnostic.run(self.root,distance_max=maximum)


if __name__=='__main__':unittest.main()
