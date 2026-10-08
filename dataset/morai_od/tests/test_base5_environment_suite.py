import json
import pickle
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json
from prepare_base5_environment_suite import prepare
from merge_base5_environment_infos import merge
from build_base5_bevfusion_infos import convert
from dynamic_capture_metadata import normalized_sensor_calibration


class EnvironmentTests(unittest.TestCase):
    def test_revision_prefixes_and_invalid_paths(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);source=root/'source.json'
            atomic_json(source,dict(scenes=[dict(id=str(i)) for i in range(5000)],config={},provenance={}))
            path=prepare(source,root/'suite','base5_yaw_v127','collection_5k_yaw_v127')
            report=json.loads(path.read_text());item=report['environments'][1]
            self.assertTrue(item['capture_output'].endswith('collection_5k_yaw_v127_sunny13'))
            self.assertEqual(json.loads(Path(item['manifest']).read_text())['dataset_id'],'base5_yaw_v127_sunny13')
            with self.assertRaises(ValueError):prepare(source,root/'bad','../bad','ok')

    def test_six_manifests_preserve_geometry_and_split(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);source=root/'source.json'
            scenes=[dict(id=f's{i}',scene_group_id=f's{i}',split='val' if i%5==0 else 'train') for i in range(5000)]
            atomic_json(source,dict(scenes=scenes,config={},provenance={}))
            path=prepare(source,root/'suite');report=json.loads(path.read_text())
            self.assertEqual(report['total_expected_frames'],30000)
            self.assertEqual(len(report['environments']),6)
            for item in report['environments']:
                deck=json.loads(Path(item['manifest']).read_text())
                self.assertEqual(deck['scenes'],scenes)
                self.assertEqual(deck['config']['collection_environment']['hour'],item['hour'])
            with self.assertRaises(FileExistsError):prepare(source,root/'suite')

    def infos(self,root,tag,split='train'):
        root.mkdir()
        for s in ('train','val','test'):
            rows=[dict(token=tag+':s1',environment_tag=tag,scene_group_id='s1')] if s==split else []
            with (root/f'morai_base5_infos_{s}.pkl').open('wb') as f:
                pickle.dump(dict(metainfo={'classes':('vehicle','pedestrian','obstacle','animal')},data_list=rows),f)
        return root

    def test_merge_keeps_same_group_same_split_with_distinct_tokens(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);a=self.infos(p/'a','sunny13');b=self.infos(p/'b','foggy13')
            r=merge([a,b],p/'out')
            self.assertEqual((r['samples'],r['independent_scene_groups'],r['group_split_leakage']),(2,1,0))

    def test_merge_rejects_environment_split_leakage(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);a=self.infos(p/'a','sunny13');b=self.infos(p/'b','foggy13','val')
            with self.assertRaisesRegex(ValueError,'leaks'):merge([a,b],p/'out')

    def test_merge_rejects_duplicate_tokens(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);a=self.infos(p/'a','sunny13');b=self.infos(p/'b','sunny13')
            with self.assertRaisesRegex(ValueError,'Duplicate'):merge([a,b],p/'out')

    def test_converter_tags_token_but_preserves_group_and_split(self):
        from PIL import Image
        sensor=Path(__file__).resolve().parents[3]/'configs/sensors/Sensor_competition_local.json'
        calibration=normalized_sensor_calibration(sensor)
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);cap=root/'capture';cap.mkdir()
            atomic_json(cap/'metadata.json',dict(runtime_actors=[],split='val',scene_group_id='source-cell'))
            atomic_json(cap/'audit.json',dict(review_reasons=[],components=[]))
            folder=cap/'LIDAR_5';folder.mkdir();(folder/'0_Intensity.bin').write_bytes(bytes(16))
            for cam in calibration['cameras'].values():
                folder=cap/('CAMERA_'+str(cam['sensor_unique_id']));folder.mkdir()
                Image.new('RGB',tuple(cam['resolution_px'])).save(folder/'0_Intensity.png')
            ledger=root/'ledger.jsonl'
            ledger.write_text(json.dumps(dict(id='runtime-cell',status='accepted_candidate',capture_root=str(cap)))+'\n')
            convert(ledger,root/'infos',sensor,environment_tag='sunny13')
            payload=pickle.loads((root/'infos/morai_base5_infos_val.pkl').read_bytes())
            row=payload['data_list'][0]
            self.assertEqual(row['token'],'sunny13:source-cell')
            self.assertEqual(row['scene_group_id'],'source-cell')
            self.assertEqual(row['environment_tag'],'sunny13')


if __name__=='__main__':unittest.main()
