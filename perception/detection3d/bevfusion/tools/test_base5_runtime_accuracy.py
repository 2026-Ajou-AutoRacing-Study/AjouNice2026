import json
from pathlib import Path
import pickle
import tempfile
import unittest

from summarize_base5_runtime_accuracy import make_row, summarize
from prepare_base5_runtime_validation_selection import build


def source():
    return dict(token='frame', instances=[dict(bbox_3d=[1,0,2,2,1,4,0],
        bbox_label_3d=0,num_lidar_pts=5,asset='car')])


def detection():
    return dict(center=[1,0,0],size=[2,1,4],yaw=0.,label=0,
                class_name='vehicle',score=.9)


class RuntimeAccuracyTests(unittest.TestCase):
    def test_geometric_center_is_converted_to_bottom_center(self):
        row=make_row('frame',source(),[detection()])
        self.assertEqual(row['gt_boxes'],row['pred_boxes'])
        self.assertEqual(row['gt_points'],[5])

    def test_class_name_cannot_silently_change(self):
        d=detection();d['class_name']='obstacle'
        with self.assertRaises(ValueError):make_row('frame',source(),[d])

    def test_empty_frame_remains_negative(self):
        self.assertEqual(make_row('frame',dict(instances=[]),[])['gt_boxes'],[])

    def fixture(self,root):
        info=root/'infos/512x1056';info.mkdir(parents=True)
        for split,rows in [('train',[source()]),('val',[])]:
            (info/f'morai_base5_infos_{split}.pkl').write_bytes(pickle.dumps(dict(data_list=rows)))
        report=dict(preserve_learned_config=True,model_geometry=dict(classes=['vehicle','pedestrian','obstacle']),
            decoder=dict(score_floor=.1,max_detections=100),precision='AMP',cases=[])
        for index,name in enumerate(('baseline','parallel_camera'),1):
            (root/f'{index}_{name}_predictions.json').write_text(json.dumps(dict(frame=[detection()])))
            report['cases'].append(dict(variant=name,per_sample=[dict(id='frame')],
                parity=dict(passed=True),prediction_counts=dict(total=1)))
        (root/'report.json').write_text(json.dumps(report))
        return report

    def test_same_gt_and_exports_have_zero_accuracy_delta(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);self.fixture(root)
            result=summarize(root,root/'report.json',root/'accuracy.json')
            self.assertEqual(result['split_counts'],dict(train=1,val=0))
            self.assertTrue(all(v==0 for v in result['cases'][1]['delta_pp_from_first_baseline'].values()))

    def test_shape_changed_cost_candidate_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);report=self.fixture(root);report['preserve_learned_config']=False
            (root/'report.json').write_text(json.dumps(report))
            with self.assertRaises(ValueError):summarize(root,root/'report.json',root/'accuracy.json')

    def test_other_decoder_cannot_be_silently_refiltered(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);report=self.fixture(root);report['decoder']['score_floor']=.2
            (root/'report.json').write_text(json.dumps(report))
            with self.assertRaises(ValueError):summarize(root,root/'report.json',root/'accuracy.json')

    def test_all_validation_scope_is_not_marked_as_mixed(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);self.fixture(root)
            info=root/'infos/512x1056'
            for split,rows in [('train',[]),('val',[source()])]:
                (info/f'morai_base5_infos_{split}.pkl').write_bytes(pickle.dumps(dict(data_list=rows)))
            result=summarize(root,root/'report.json',root/'accuracy.json')
            self.assertEqual(result['split_counts'],dict(train=0,val=1))
            self.assertIn('existing validation split',result['scope'])

    def selection_fixture(self, root, count=745, policy='native_rpy_fixed_yaw_enclosure_v1'):
        rows=[dict(token=f'frame_{i:04}',lidar_points=dict(lidar_path=f'{i}.bin'),
            images={name:dict(img_path=f'{name}/{i}.jpg') for name in
                ('Camera_front','Camera_front_left','Camera_front_right','Camera_back')})
            for i in reversed(range(count))]
        folder=root/'infos/512x1056';folder.mkdir(parents=True)
        path=folder/'morai_base5_infos_val.pkl'
        path.write_bytes(pickle.dumps(dict(metainfo=dict(box_geometry_policy=policy),data_list=rows)))
        return path

    def test_complete_validation_selection_is_sorted_and_distinct(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);self.selection_fixture(root)
            result=build(root,root/'selection.json')
            self.assertEqual(len(result['samples']),745)
            self.assertEqual(result['samples'][0]['id'],'frame_0000')
            self.assertTrue(all(s['split']=='val' for s in result['samples']))
            with self.assertRaises(ValueError):build(root,root/'selection.json')

    def test_validation_selection_rejects_wrong_count_and_policy(self):
        for count,policy in [(744,'native_rpy_fixed_yaw_enclosure_v1'),(745,'old')]:
            with tempfile.TemporaryDirectory() as temp:
                root=Path(temp);self.selection_fixture(root,count,policy)
                with self.assertRaises(ValueError):build(root,root/'selection.json')

    def test_validation_selection_rejects_duplicate_tokens(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);path=self.selection_fixture(root)
            info=pickle.loads(path.read_bytes());info['data_list'][0]['token']=info['data_list'][1]['token']
            path.write_bytes(pickle.dumps(info))
            with self.assertRaises(ValueError):build(root,root/'selection.json')


if __name__=='__main__':unittest.main()
