import argparse
import json
from pathlib import Path
import pickle
import tempfile
import unittest

from prepare_base5_portable_dataset import build
from run_base5_direct_training import prepare


class DirectRunnerTest(unittest.TestCase):
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


if __name__=='__main__': unittest.main()
