import argparse
from copy import deepcopy
import json
from pathlib import Path
import pickle
import tempfile
import unittest

import numpy as np
from PIL import Image
from mmengine.config import Config

from prepare_base5_portable_dataset import build as png_build, validate
from prepare_base5_jpeg_portable import build
from benchmark_base5_jpeg_cache import build_cache
from run_base5_direct_training import prepare


class JPEGPortableTest(unittest.TestCase):
    def fixture(self, root):
        captures = root / 'captures'; captures.mkdir()
        rows, records = [], []
        for i in range(2):
            folder = captures / str(i); folder.mkdir()
            (folder / 'points.bin').write_bytes(b'points')
            (folder / 'native_annotation.json').write_text('{"keep":true}')
            Image.fromarray(np.ones((32,64), np.uint16)).save(folder / 'instance_mask.png')
            cameras = {}
            for c in range(4):
                path = folder / ('c%d.png' % c)
                Image.fromarray(np.random.RandomState(i*4+c).randint(0,256,(32,64,3),np.uint8)).save(path)
                cameras[str(c)] = dict(img_path=str(path), height=32, width=64)
            rows.append(dict(id='s%d'%i, capture_root=str(folder)))
            records.append(dict(token='s%d'%i, scene_group_id='g%d'%i,
                images=cameras, lidar_points=dict(lidar_path=str(folder/'points.bin')),
                instances=[dict(bbox_3d=[0,0,0,1,1,1,0], bbox_label_3d=0)]))
        ledger = root / 'ledger.jsonl'; ledger.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        folder = root / 'infos'; folder.mkdir()
        for split in ('train','val','test'):
            info = dict(metainfo=dict(classes=('vehicle','pedestrian','obstacle','animal')),
                        data_list=[records[0]] if split=='train' else [records[1]])
            (folder/('morai_base5_infos_'+split+'.pkl')).write_bytes(pickle.dumps(info))
        (folder/'calibration.json').write_text('{}')
        source = root/'png'; png_build(ledger, {'512x1056':folder}, source, 2)
        native=[]
        for split in ('train','val'):
            native.extend(pickle.loads((source/'infos/512x1056'/('morai_base5_infos_'+split+'.pkl')).read_bytes())['data_list'])
        for row in native:
            for camera in row['images'].values(): camera['img_path']=str(source/camera['img_path'])
        cache = root/'cache'; build_cache(source,native,cache)
        return source, cache

    def test_independent_relocatable_all_jpeg_and_lossless_auxiliary(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); source,cache=self.fixture(root)
            before=(source/'manifest.json').read_bytes()
            result=build(source,cache,root/'jpeg')
            self.assertEqual(result['jpeg_train_image_count'],4)
            self.assertEqual(result['jpeg_rgb_image_count'],8)
            self.assertEqual(result['image_policy'],dict.fromkeys(('train','val','test'),'native_jpeg90'))
            self.assertEqual((source/'manifest.json').read_bytes(),before)
            train=pickle.loads((root/'jpeg/infos/512x1056/morai_base5_infos_train.pkl').read_bytes())['data_list'][0]
            original=pickle.loads((source/'infos/512x1056/morai_base5_infos_train.pkl').read_bytes())['data_list'][0]
            self.assertEqual(train['instances'],original['instances'])
            self.assertEqual(train['lidar_points'],original['lidar_points'])
            for camera in train['images'].values(): self.assertTrue(camera['img_path'].endswith('.jpg'))
            for split in ('val','test'):
                converted=pickle.loads((root/'jpeg/infos/512x1056'/('morai_base5_infos_'+split+'.pkl')).read_bytes())
                prior=pickle.loads((source/'infos/512x1056'/('morai_base5_infos_'+split+'.pkl')).read_bytes())
                for new,old in zip(converted['data_list'],prior['data_list']):
                    for name,camera in new['images'].items():
                        self.assertTrue(camera['img_path'].endswith('.jpg'))
                        camera['img_path']=old['images'][name]['img_path']
                self.assertEqual(converted,prior)
            self.assertNotIn('data/s0/c0.png',result['files'])
            self.assertNotIn('data/s1/c0.png',result['files'])
            self.assertEqual((root/'jpeg/data/s1/instance_mask.png').read_bytes(),
                             (source/'data/s1/instance_mask.png').read_bytes())
            self.assertTrue((root/'jpeg/data/s0/native_annotation.json').is_file())
            source.rename(root/'png_unavailable'); cache.rename(root/'cache_unavailable')
            (root/'jpeg').rename(root/'moved')
            validate(root/'moved',True)
            with self.assertRaises(FileExistsError): build(root/'png_unavailable',root/'cache_unavailable',root/'moved')

    def test_preparation_needs_no_png_source_and_never_double_encodes(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); source,cache=self.fixture(root)
            build(source,cache,root/'jpeg')
            source.rename(root/'hidden'); cache.rename(root/'hidden_cache')
            config=Path(__file__).resolve().parents[1]/'configs/bevfusion_base5_dense_512_native_jpeg90.py'
            args=argparse.Namespace(data=root/'jpeg',profile='512x1056',batch=1,accumulate=1,
                epochs=2,workers=0,lr=1e-4,config=config,warmup_updates=5,seed=1,
                work_dir=root/'run',disable_val=False,wandb=False,init_checkpoint=None,
                resume=None,allow_random_init=True,run=True,jpeg_cache=None)
            cfg,_=prepare(args)
            self.assertFalse(any(t['type'] in ('MoraiUseNativeJPEGCache','MoraiOnlineImageDomainAug3D')
                                 for t in cfg.train_dataloader.dataset.pipeline))
            self.assertEqual(cfg.val_dataloader.dataset.pipeline,Config.fromfile(str(config)).val_dataloader.dataset.pipeline)
            args.jpeg_cache=root/'hidden_cache'
            with self.assertRaisesRegex(ValueError,'additional JPEG cache'): prepare(args)

    def test_historical_mixed_policy_is_explicit_and_current_config_rejects_it(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); source,cache=self.fixture(root)
            result=build(source,cache,root/'mixed',keep_val_png=True)
            self.assertEqual(result['image_policy']['val'],'original_png')
            self.assertEqual((root/'mixed/infos/512x1056/morai_base5_infos_val.pkl').read_bytes(),
                             (source/'infos/512x1056/morai_base5_infos_val.pkl').read_bytes())
            config=Path(__file__).resolve().parents[1]/'configs/bevfusion_base5_dense_512_native_jpeg90.py'
            args=argparse.Namespace(data=root/'mixed',profile='512x1056',batch=1,accumulate=1,
                epochs=2,workers=0,lr=1e-4,config=config,warmup_updates=5,seed=1,
                work_dir=root/'run',disable_val=False,wandb=False,init_checkpoint=None,
                resume=None,allow_random_init=True,run=True,jpeg_cache=None)
            with self.assertRaisesRegex(ValueError,'JPEG train/val/test'): prepare(args)
            args.data=source
            with self.assertRaisesRegex(ValueError,'JPEG train/val/test'): prepare(args)

    def test_all_jpeg_requires_validation_cache_coverage(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); source,cache=self.fixture(root)
            index=json.loads((cache/'manifest.json').read_text())
            index['files']={p:v for p,v in index['files'].items() if '/s0/' in p}
            index['image_count']=len(index['files'])
            (cache/'manifest.json').write_text(json.dumps(index))
            with self.assertRaisesRegex(ValueError,'missing required'): build(source,cache,root/'jpeg')
            self.assertFalse((root/'jpeg').exists())

    def test_bad_split_encoding_and_tamper_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); source,cache=self.fixture(root)
            result=build(source,cache,root/'jpeg')
            target=root/'jpeg'/next(p for p in result['files'] if p.endswith('.jpg'))
            target.write_bytes(b'broken')
            with self.assertRaises(ValueError): validate(root/'jpeg',True)
            wrong=deepcopy(json.loads((source/'manifest.json').read_text()))
            wrong['image_policy']=dict(train='native_jpeg90',val='original_png',test='original_png')
            (source/'manifest.json').write_text(json.dumps(wrong))
            with self.assertRaisesRegex(ValueError,'encoding/split'):validate(source)


if __name__=='__main__': unittest.main()
