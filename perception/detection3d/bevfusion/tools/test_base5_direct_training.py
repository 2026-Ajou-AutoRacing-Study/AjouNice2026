import argparse
import hashlib
import json
from pathlib import Path
import pickle
import tempfile
import unittest

from prepare_base5_portable_dataset import build
from run_base5_direct_training import prepare
from mmengine.config import Config


class DirectRunnerTest(unittest.TestCase):
    def test_restart_schedule_is_pinned_and_rejects_partial_cycles(self):
        with tempfile.TemporaryDirectory() as name:
            args = self.fixture(Path(name))
            args.lr_schedule = 'warmup-restarts'
            args.epochs = 8
            args.restart_cycle_epochs = 4
            args.restart_warmup_updates = 1
            args.wandb_group = 'base5-sunny15-ab-261018'
            cfg, manifest = prepare(args)
            self.assertEqual(cfg.param_scheduler[0].type, 'MoraiWarmupCosineRestarts')
            self.assertEqual(manifest['identity']['param_schedulers'][0]['cycle_steps'], 8)
            args.restart_warmup_updates = 2
            with self.assertRaisesRegex(FileExistsError, 'changed run'):
                prepare(args)
            args.epochs = 7
            with self.assertRaisesRegex(ValueError, 'complete fixed cycles'):
                prepare(args)

    def fixture(self, root):
        rows=[]; records=[]
        for i in range(2):
            capture=root/('capture%d'%i); capture.mkdir(); images={}
            (capture/'points.bin').write_bytes(b'points')
            for c in range(4):
                image=capture/('camera%d.png'%c); image.write_bytes(b'pixels')
                images[str(c)]=dict(img_path=str(image))
            rows.append(dict(id='c%d'%i,capture_root=str(capture)))
            records.append(dict(token='c%d'%i,scene_group_id='g%d'%i,
                lidar_points=dict(lidar_path=str(capture/'points.bin')),images=images))
        ledger=root/'ledger.jsonl'; ledger.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        infos=root/'infos'; infos.mkdir()
        for split in ('train','val','test'):
            info=dict(metainfo=dict(classes=('vehicle','pedestrian','obstacle','animal')),
                      data_list=records if split=='train' else [])
            (infos/('morai_base5_infos_'+split+'.pkl')).write_bytes(pickle.dumps(info))
        (infos/'calibration.json').write_text('{}')
        portable=root/'portable'; build(ledger,{'448x896':infos},portable,2)
        config=Path(__file__).resolve().parent.parent/'configs/bevfusion_base5_dense_448_separated_candidate.py'
        return argparse.Namespace(data=portable, profile='448x896', batch=1, accumulate=1,
            epochs=2, workers=0, lr=1e-4, config=config, warmup_updates=500,
            seed=12345, work_dir=root/'run', disable_val=True, wandb=False,
            init_checkpoint=None, resume=None, allow_random_init=True)

    def test_paths_schedule_and_resume_guard(self):
        with tempfile.TemporaryDirectory() as name:
            args=self.fixture(Path(name)); cfg,manifest=prepare(args)
            self.assertEqual(cfg.train_dataloader.dataset.data_root,str(args.data))
            self.assertEqual(cfg.train_dataloader.dataset.data_prefix,dict(pts='',img=''))
            self.assertFalse(cfg.train_dataloader.persistent_workers)
            self.assertEqual(cfg.param_scheduler[-1].end,4)
            self.assertEqual(cfg.param_scheduler[-1].begin,cfg.param_scheduler[0].end)
            self.assertNotIn('vel',cfg.model.bbox_head.common_heads)
            prepared_again, repeat = prepare(args)
            self.assertEqual(repeat['contract_sha256'],manifest['contract_sha256'])
            checkpoint=args.work_dir/'epoch_1.pth'; checkpoint.write_bytes(b'placeholder')
            (args.work_dir/'last_checkpoint').write_text(str(checkpoint))
            args.resume='auto'; resumed,again=prepare(args)
            self.assertTrue(resumed.resume); self.assertEqual(manifest['contract_sha256'],again['contract_sha256'])
            args.lr=2e-4
            with self.assertRaisesRegex(ValueError,'contract changed'): prepare(args)

    def test_initialization_and_accumulation_guard(self):
        with tempfile.TemporaryDirectory() as name:
            args=self.fixture(Path(name)); args.allow_random_init=False
            with self.assertRaisesRegex(ValueError,'init-checkpoint'): prepare(args)
            args.allow_random_init=True; args.accumulate=3
            with self.assertRaisesRegex(ValueError,'divide accumulation'): prepare(args)

    def test_content_integrity_on_run(self):
        with tempfile.TemporaryDirectory() as name:
            args=self.fixture(Path(name)); args.run=True
            (args.data/'data/c0/points.bin').write_bytes(b'broken')
            with self.assertRaisesRegex(ValueError,'checksum mismatch'): prepare(args)

    def test_wandb_identity_has_no_teammate_credentials(self):
        with tempfile.TemporaryDirectory() as name:
            args=self.fixture(Path(name)); args.wandb=True
            cfg,manifest=prepare(args)
            wandb=next(x for x in cfg.visualizer.vis_backends if x['type']=='MoraiWandbVisBackend')
            self.assertEqual(wandb.init_kwargs.id,args.work_dir.name+'-'+manifest['contract_sha256'][:12])
            self.assertNotIn('entity',wandb.init_kwargs)
            self.assertNotIn('api_key',wandb.init_kwargs)

    def test_cache_is_train_only_and_identity_guards_resume(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name)
            args=self.fixture(root)
            cfg=Config.fromfile(str(args.config))
            for item in cfg.train_dataloader.dataset.pipeline:
                if item['type']=='MoraiOnlineImageDomainAug3D':
                    item.update(jpeg_prob=1.,jpeg_quality=[90,90],native_resolution=True,color_order='rgb')
            config=root/'cache_config.py'; cfg.dump(str(config)); args.config=config
            source=json.loads((args.data/'manifest.json').read_text())
            cache=root/'cache'; cache.mkdir()
            rows=pickle.loads((args.data/'infos/448x896/morai_base5_infos_train.pkl').read_bytes())['data_list']
            files={}
            for row in rows:
                for camera in row['images'].values():
                    relative=camera['img_path']
                    target=cache/(hashlib.sha256(relative.encode()).hexdigest()+'.jpg')
                    target.write_bytes(b'cache-fixture')
                    files[relative]=dict(path=target.name,bytes=target.stat().st_size,
                        sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                        source_sha256=source['files'][relative]['sha256'],height=32,width=64)
            manifest=dict(version=1,complete=True,quality=90,native_resolution=True,
                source_color_order='rgb',image_count=len(files),files=files,
                source_manifest_sha256=hashlib.sha256((args.data/'manifest.json').read_bytes()).hexdigest())
            (cache/'manifest.json').write_text(json.dumps(manifest))
            args.jpeg_cache=cache
            args.disable_val=False
            selected,identity=prepare(args)
            self.assertIn('jpeg_cache_manifest_sha256',identity['identity'])
            pipeline=selected.train_dataloader.dataset.pipeline
            self.assertTrue(any(t['type']=='MoraiUseNativeJPEGCache' for t in pipeline))
            self.assertFalse(any(t['type']=='MoraiOnlineImageDomainAug3D' for t in pipeline))
            self.assertEqual(selected.val_dataloader.dataset.pipeline,cfg.val_dataloader.dataset.pipeline)
            checkpoint=args.work_dir/'epoch_1.pth'; checkpoint.write_bytes(b'placeholder')
            (args.work_dir/'last_checkpoint').write_text(str(checkpoint))
            args.resume='auto'
            _,again=prepare(args)
            self.assertEqual(identity['contract_sha256'],again['contract_sha256'])
            manifest['note']='changed cache contract'
            (cache/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError,'contract changed'):
                prepare(args)


if __name__=='__main__': unittest.main()
