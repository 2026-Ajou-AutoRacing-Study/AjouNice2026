import argparse
from copy import deepcopy
import json
from pathlib import Path
import pickle
import tempfile
import unittest

from prepare_base5_portable_dataset import build as portable_build, validate
from prepare_base5_upright_portable import build
from run_base5_direct_training import prepare
from dataset.morai_od.scripts.base5_upright_boxes import BOX_GEOMETRY_POLICY


class UprightPortableTest(unittest.TestCase):
    def fixture(self, root):
        captures=[];records=[]
        for i in range(2):
            cap=root/f'capture{i}';cap.mkdir();(cap/'lidar.bin').write_bytes(b'points')
            images={}
            for j in range(4):
                path=cap/f'{j}.jpg';path.write_bytes(b'jpeg')
                images[str(j)]=dict(img_path=str(path),height=720,width=1280)
            component=dict(component_id='8500:Obstacle:0',parent_uid=8500,asset='PE_Drum',training_class='obstacle',
                           center_lidar_m=[10,2,-1],size_m=[.5,.5,.8],rotation_rpy=[1.5707963267948966,0,.5],lidar_points=0)
            (cap/'audit.json').write_text(json.dumps(dict(components=[component])))
            instance=dict(component_id=component['component_id'],parent_uid=8500,asset='PE_Drum',bbox_label_3d=2,
                          bbox_3d=[10,2,-1,.5,.5,.8,.5],num_lidar_pts=0)
            records.append(dict(token=f's{i}',scene_group_id=f'g{i}',images=images,
                                lidar_points=dict(lidar_path=str(cap/'lidar.bin')),instances=[instance]))
            captures.append(dict(id=f's{i}',capture_root=str(cap)))
        ledger=root/'ledger.jsonl';ledger.write_text(''.join(json.dumps(r)+'\n' for r in captures))
        infos=root/'infos';infos.mkdir()
        for split in ('train','val','test'):
            (infos/f'morai_base5_infos_{split}.pkl').write_bytes(pickle.dumps(dict(
                metainfo=dict(classes=('vehicle','pedestrian','obstacle','animal')),
                data_list=[records[0]] if split=='train' else [records[1]])))
        (infos/'calibration.json').write_text('{}')
        data=root/'source';manifest=portable_build(ledger,{'512x1056':infos},data,2)
        manifest['image_policy']=dict.fromkeys(('train','val','test'),'native_jpeg90')
        (data/'manifest.json').write_text(json.dumps(manifest))
        return data

    def test_rebuild_preserves_visibility_classes_inputs_and_source(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.fixture(root);before=(source/'manifest.json').read_bytes()
            result,report=build(source,root/'new',hardlink=True)
            self.assertEqual(result['box_geometry_policy'],BOX_GEOMETRY_POLICY)
            self.assertEqual(report['stats']['512x1056:train']['camera_only'],1)
            for split in ('train','val','test'):
                path=f'infos/512x1056/morai_base5_infos_{split}.pkl'
                old=pickle.loads((source/path).read_bytes());new=pickle.loads((root/'new'/path).read_bytes())
                self.assertEqual(new['metainfo']['box_geometry_policy'],BOX_GEOMETRY_POLICY)
                new['metainfo'].pop('box_geometry_policy')
                for a,b in zip(old['data_list'],new['data_list']):
                    self.assertAlmostEqual(b['instances'][0]['bbox_3d'][4],.8)
                    self.assertAlmostEqual(b['instances'][0]['bbox_3d'][5],.5)
                    b['instances'][0]['bbox_3d']=a['instances'][0]['bbox_3d']
                self.assertEqual(old,new)
            self.assertEqual((source/'manifest.json').read_bytes(),before)
            self.assertEqual((source/'data/s0/audit.json').stat().st_ino,(root/'new/data/s0/audit.json').stat().st_ino)
            source.rename(root/'source_hidden');validate(root/'new',True)
            with self.assertRaises(FileExistsError):build(root/'new',root/'new')
            with self.assertRaisesRegex(ValueError,'already has'):build(root/'new',root/'another')

    def test_training_rejects_old_labels_and_accepts_new_contract(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.fixture(root)
            config=Path(__file__).resolve().parents[1]/'configs/bevfusion_base5_dense_512_native_jpeg90.py'
            args=argparse.Namespace(data=source,profile='512x1056',batch=1,accumulate=1,epochs=2,
                workers=0,lr=1e-4,config=config,warmup_updates=1,seed=1,work_dir=root/'run',disable_val=True,
                wandb=False,init_checkpoint=None,resume=None,allow_random_init=True,run=False,jpeg_cache=None)
            with self.assertRaisesRegex(ValueError,'upright native-RPY'):prepare(args)
            build(source,root/'new');args.data=root/'new'
            _,manifest=prepare(args)
            self.assertEqual(manifest['identity']['box_geometry_policy'],BOX_GEOMETRY_POLICY)
            # Changing geometry is an input identity change even if size stays the same.
            args.data=source
            with self.assertRaisesRegex(ValueError,'upright native-RPY'):prepare(args)

    def test_missing_native_component_or_tampered_source_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.fixture(root)
            (source/'data/s0/audit.json').write_text('{}')
            with self.assertRaises(ValueError):build(source,root/'new')
            self.assertFalse((root/'new').exists())


if __name__=='__main__':unittest.main()
