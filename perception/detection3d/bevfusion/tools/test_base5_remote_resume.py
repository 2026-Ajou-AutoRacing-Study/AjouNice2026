"""Small real MMEngine/CUDA-AMP interruption/resume test, not detector quality."""
import argparse
import json
from pathlib import Path
import random
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np
import torch
from mmengine.config import Config
from mmengine.hooks import Hook
from mmengine.model import BaseModel
from mmengine.registry import MODELS, HOOKS, DATASETS
from torch.utils.data import Dataset

from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import (
    Base5AtomicRunner, optimizer_parameter_names, validate_optimizer_parameter_order)


@MODELS.register_module(force=True)
class RemoteResumeToy(BaseModel):
    def __init__(self):
        super().__init__()
        self.layer = torch.nn.Linear(4, 1)

    def forward(self, inputs, data_samples=None, mode='loss'):
        return dict(loss=(self.layer(inputs) - 1).square().mean())


@DATASETS.register_module(force=True)
class ToyData(Dataset):
    metainfo = dict(classes=('toy',))

    def __len__(self): return 4

    def __getitem__(self, index):
        values = torch.randn(4) + float(np.random.rand()) + random.random() + index
        return dict(inputs=values)


class PlannedStop(Exception): pass


@HOOKS.register_module(force=True)
class StopAfterCheckpoint(Hook):
    priority = 'LOWEST'
    def after_train_epoch(self, runner):
        if runner.epoch == 0:
            raise PlannedStop('Simulated interruption after atomic epoch checkpoint')


def config(work_dir, interrupted=False, resume=False, contract='test-contract'):
    hooks = [dict(type='MoraiResumeEvidenceHook', contract_sha256=contract)]
    if interrupted: hooks.append(dict(type='StopAfterCheckpoint'))
    return Config(dict(model=dict(type='RemoteResumeToy'), default_scope='mmdet3d',
        work_dir=str(work_dir), launcher='none', randomness=dict(seed=12345),
        train_dataloader=dict(batch_size=2, num_workers=0, dataset=dict(type='ToyData'),
            sampler=dict(type='DefaultSampler', shuffle=False), collate_fn=dict(type='default_collate')),
        train_cfg=dict(type='EpochBasedTrainLoop', max_epochs=2),
        optim_wrapper=dict(type='AmpOptimWrapper', loss_scale=dict(init_scale=32),
            optimizer=dict(type='AdamW', lr=.001)),
        param_scheduler=[dict(type='CosineAnnealingLR', by_epoch=False, T_max=4, begin=0, end=4)],
        default_hooks=dict(checkpoint=dict(type='CheckpointHook', interval=1, max_keep_ckpts=2),
            logger=dict(type='LoggerHook', interval=1), param_scheduler=dict(type='ParamSchedulerHook'),
            sampler_seed=dict(type='DistSamplerSeedHook')),
        custom_hooks=hooks, resume=resume,
        load_from=str(work_dir/'epoch_1.pth') if resume else None,
        env_cfg=dict(cudnn_benchmark=False, mp_cfg=dict(mp_start_method='fork', opencv_num_threads=0)),
        visualizer=dict(type='Visualizer', vis_backends=[dict(type='LocalVisBackend')]),
        val_dataloader=None, val_cfg=None, val_evaluator=None))


class ParameterOrderTest(unittest.TestCase):
    def fixture(self, reverse=False):
        # Same-shaped branches make shape-only resume checks insufficient.
        model = torch.nn.Module()
        for name in (('dim', 'height') if reverse else ('height', 'dim')):
            model.add_module(name, torch.nn.Linear(2, 2))
        optimizer = torch.optim.AdamW(model.parameters(), lr=.001)
        runner = SimpleNamespace(model=model, optim_wrapper=SimpleNamespace(optimizer=optimizer))
        checkpoint = dict(state_dict=model.state_dict(), optimizer=optimizer.state_dict(),
                          base5_optimizer_parameter_names=optimizer_parameter_names(runner))
        return runner, checkpoint

    def test_recorded_names_and_legacy_state_order_accept_original(self):
        runner, checkpoint = self.fixture()
        validate_optimizer_parameter_order(runner, checkpoint)
        checkpoint.pop('base5_optimizer_parameter_names')
        validate_optimizer_parameter_order(runner, checkpoint)

    def test_equal_shape_reordered_branches_are_rejected_before_optimizer_load(self):
        runner, checkpoint = self.fixture()
        changed, _ = self.fixture(reverse=True)
        with self.assertRaisesRegex(ValueError, 'names/order differ'):
            validate_optimizer_parameter_order(changed, checkpoint)
        checkpoint.pop('base5_optimizer_parameter_names')
        with self.assertRaisesRegex(ValueError, 'names/order differ'):
            validate_optimizer_parameter_order(changed, checkpoint)

    def test_moment_shape_corruption_is_rejected(self):
        runner, checkpoint = self.fixture()
        key = checkpoint['optimizer']['param_groups'][0]['params'][0]
        checkpoint['optimizer']['state'][key] = {'exp_avg': torch.zeros(3, 2)}
        with self.assertRaisesRegex(ValueError, 'moment shape differs'):
            validate_optimizer_parameter_order(runner, checkpoint)


class ResumeTest(unittest.TestCase):
    @unittest.skipUnless(torch.cuda.is_available(), 'CUDA AMP required')
    def test_epoch_resume(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); continuous = root/'continuous'; resumed = root/'resumed'
            Base5AtomicRunner.from_cfg(config(continuous)).train()
            with self.assertRaises(PlannedStop):
                Base5AtomicRunner.from_cfg(config(resumed, interrupted=True)).train()
            first = torch.load(resumed/'epoch_1.pth', weights_only=False, map_location='cpu')
            self.assertEqual(first['meta']['epoch'], 1)
            self.assertEqual(first['meta']['iter'], 2)
            self.assertIn('loss_scaler', first['optimizer'])
            self.assertTrue(first['base5_rng']['cuda'])
            Base5AtomicRunner.from_cfg(config(resumed, resume=True)).train()
            a = torch.load(continuous/'epoch_2.pth', weights_only=False, map_location='cpu')
            b = torch.load(resumed/'epoch_2.pth', weights_only=False, map_location='cpu')
            for key, tensor in a['state_dict'].items():
                torch.testing.assert_close(tensor, b['state_dict'][key], atol=0, rtol=0)
            self.assertEqual(a['param_schedulers'], b['param_schedulers'])
            self.assertEqual(a['optimizer']['loss_scaler'], b['optimizer']['loss_scaler'])
            self.assertEqual(b['meta']['iter'], 4)
            evidence = json.loads((resumed/'resume_start.json').read_text())
            self.assertTrue(evidence['resumed']); self.assertEqual(evidence['iteration'], 2)
            self.assertEqual(evidence['optimizer_steps'], [2.0])
            self.assertFalse(list(resumed.glob('*.partial-*')))
            with self.assertRaisesRegex(ValueError, 'contract differs'):
                Base5AtomicRunner.from_cfg(config(resumed, resume=True, contract='changed')).train()


if __name__ == '__main__': unittest.main()
