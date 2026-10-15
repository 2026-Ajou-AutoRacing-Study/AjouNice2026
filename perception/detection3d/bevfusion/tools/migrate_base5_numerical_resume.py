#!/usr/bin/env python3
"""Explicit, audited numerical-repair migration plus bounded epoch validation.

Never relax ordinary resume's immutable contract. Dataset, model parameters,
initializer, augmentation, batch, optimizer and the 24-epoch LR schedule must
match the parent. Only enumerated numerical/diagnostic/backup code may change.
The parent directory and checkpoint remain untouched.
"""
import argparse
import copy
import json
from pathlib import Path
import time

import torch
from mmengine.hooks import Hook
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import HOOKS
from run_base5_direct_training import prepare
from prepare_base5_portable_dataset import sha256


ALLOWED_SOURCE_CHANGES = {
    'perception/detection3d/bevfusion/plugin/morai_rectangular_bev.py',
    'perception/detection3d/bevfusion/plugin/morai_remote_checkpoint.py',
    'perception/detection3d/bevfusion/plugin/morai_wandb.py',
    'perception/detection3d/bevfusion/tests/test_base5_heatmap_targets.py',
    'perception/detection3d/bevfusion/tests/test_base5_wandb_visibility.py',
    'perception/detection3d/bevfusion/tools/backup_base5_checkpoints.py',
    'perception/detection3d/bevfusion/tools/backup_base5_vast_access_only.py',
    'perception/detection3d/bevfusion/tools/test_base5_checkpoint_backup.py',
    'perception/detection3d/bevfusion/tools/test_base5_vast_access_only.py',
    'perception/detection3d/bevfusion/tools/migrate_base5_numerical_resume.py',
    'perception/detection3d/bevfusion/tools/test_base5_numerical_migration.py',
    'perception/detection3d/bevfusion/tools/test_base5_remote_resume.py',
    'perception/detection3d/bevfusion/scripts/audit_base5_training_health.py',
    'perception/detection3d/bevfusion/tools/prepare_base5_lidar_initializer.py',
    'perception/detection3d/bevfusion/tools/test_base5_lidar_initializer.py',
}


def check_identity(parent, child):
    # Manifests serialize config tuples as JSON arrays. Compare the same
    # canonical representation, without weakening any numeric/code contract.
    old, new = (json.loads(json.dumps(value)) for value in (parent, child))
    old_sources, new_sources = old.pop('source_hashes'), new.pop('source_hashes')
    if old != new:
        differences = [key for key in set(old) | set(new) if old.get(key) != new.get(key)]
        raise ValueError('Migration cannot change training identity: '+repr(differences))
    changes = sorted(key for key in set(old_sources) | set(new_sources)
                     if old_sources.get(key) != new_sources.get(key))
    if not changes or set(changes)-ALLOWED_SOURCE_CHANGES:
        raise ValueError('Unapproved migration code changes: '+repr(changes))
    return changes


@HOOKS.register_module()
class Base5EpochValidationStop(Hook):
    priority = 'LOWEST'

    def __init__(self, epoch):
        self.epoch = int(epoch)

    def after_train_epoch(self, runner):
        # EpochBasedTrainLoop still runs the scheduled validation after this
        # hook, then exits normally. LR schedule stays at its original budget.
        if runner.epoch+1 >= self.epoch:
            runner.train_loop.stop_training = True


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--parent', type=Path, required=True)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--init-checkpoint', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stop-after-epoch', type=int, required=True)
    p.add_argument('--wandb', action='store_true')
    p.add_argument('--run', action='store_true')
    a = p.parse_args()
    torch.set_num_threads(4)
    if a.output.exists():
        p.error('Use a fresh migration output; parent/trial must not be overwritten')
    parent = json.loads((a.parent/'run_manifest.json').read_text())
    identity = parent['identity']
    cfg, manifest = prepare(argparse.Namespace(
        data=a.data, config=a.config, profile=identity['profile'],work_dir=a.output,
        batch=identity['batch'],accumulate=identity['accumulate'],epochs=identity['epochs'],
        workers=identity['workers'],lr=identity['lr'],warmup_updates=identity['warmup_iterations'],
        seed=identity['seed'],disable_val=not identity['val_enabled'],wandb=a.wandb,
        init_checkpoint=a.init_checkpoint,allow_random_init=False,resume=None,run=a.run))
    changes = check_identity(identity, manifest['identity'])
    checkpoint = torch.load(a.checkpoint,map_location='cpu',weights_only=False)
    if checkpoint.get('base5_contract_sha256') != parent['contract_sha256']:
        raise ValueError('Parent checkpoint contract mismatch')
    required = ('optimizer','param_schedulers','message_hub','base5_rng')
    if any(key not in checkpoint for key in required):
        raise ValueError('Parent checkpoint is not resumable')
    if not all(bool(torch.isfinite(value).all()) for value in checkpoint['state_dict'].values()
               if torch.is_floating_point(value)):
        raise ValueError('Parent weights contain nonfinite values')
    epoch = int(checkpoint['meta']['epoch'])
    if not epoch < a.stop_after_epoch <= min(identity['epochs'],epoch+2):
        raise ValueError('Bounded validation must be one or two additional completed epochs')
    receipt = dict(parent=str(a.parent),checkpoint=str(a.checkpoint),
        parent_checkpoint_sha256=sha256(a.checkpoint),parent_contract=parent['contract_sha256'],
        child_contract=manifest['contract_sha256'],changed_sources=changes,
        epoch=epoch,iteration=checkpoint['meta']['iter'],
        retained='model/optimizer/scheduler/scaler/message-hub/main RNG; no worker-prefetch replay guarantee')
    checkpoint['base5_contract_sha256'] = manifest['contract_sha256']
    checkpoint['base5_numerical_migration'] = receipt
    target = a.output/('epoch_'+str(epoch)+'.pth')
    torch.save(checkpoint,target)
    (a.output/'last_checkpoint').write_text(str(target.resolve()))
    (a.output/'migration_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    cfg.resume = True
    cfg.load_from = str(target.resolve())
    cfg.custom_hooks.append(dict(type='Base5EpochValidationStop',epoch=a.stop_after_epoch))
    cfg.dump(str(a.output/'bounded_numerical_validation.py'))
    print(json.dumps(receipt),flush=True)
    if a.run:
        import_modules_from_strings(**cfg.custom_imports)
        from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
        status = dict(state='running',stop_after_epoch=a.stop_after_epoch,started_at=time.time())
        path = a.output/'validation_status.json'
        path.write_text(json.dumps(status))
        try:
            runner = Base5AtomicRunner.from_cfg(cfg)
            runner.train()
            status.update(state='finished',epoch=runner.epoch,iteration=runner.iter)
        except Exception as error:
            status.update(state='failed',error_type=type(error).__name__)
            raise
        finally:
            status['ended_at'] = time.time()
            path.write_text(json.dumps(status,indent=2)+'\n')


if __name__ == '__main__':
    main()
