#!/usr/bin/env python3
"""One paired initializer trial, stop after full val at E1/E2, schedule stays 24.

Original source config is always used, never a sorted resolved config. Ordinary
resume keeps its immutable guards. Offline W&B separates trial from main runs.
"""
import argparse
import json
from pathlib import Path
import time

import torch
from mmengine.utils import import_modules_from_strings

from migrate_base5_numerical_resume import Base5EpochValidationStop
from run_base5_direct_training import prepare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--initializer', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--stop-after-epoch', type=int, choices=(1, 2), default=1)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if args.work_dir.exists() and not args.resume:
        parser.error('Fresh trial must use an unused work directory')
    torch.set_num_threads(4)
    cfg, manifest = prepare(argparse.Namespace(
        data=args.data, config=args.source/'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_pedestrian_merged.py',
        profile='512x1056', work_dir=args.work_dir, batch=4, accumulate=1,
        epochs=24, workers=4, lr=1e-4, warmup_updates=500, seed=261010,
        disable_val=False, wandb=True, init_checkpoint=args.initializer,
        allow_random_init=False, resume='auto' if args.resume else None, run=args.run))
    cfg.custom_hooks.append(dict(type='Base5EpochValidationStop', epoch=args.stop_after_epoch))
    assert cfg.train_cfg.max_epochs == 24
    if not args.run:
        print(json.dumps(dict(state='prepared_only', contract=manifest['contract_sha256'])))
        return
    import_modules_from_strings(**cfg.custom_imports)
    from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
    status_path = args.work_dir/'initializer_trial_status.json'
    status = dict(state='running', stop_after_epoch=args.stop_after_epoch,
                  started_at=time.time(), contract=manifest['contract_sha256'])
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    try:
        runner = Base5AtomicRunner.from_cfg(cfg)
        runner.train()
        status.update(state='finished', epoch=runner.epoch, iteration=runner.iter)
        if runner.epoch != args.stop_after_epoch:
            raise RuntimeError('Unexpected bounded epoch count')
    except Exception as error:
        status.update(state='failed', error_type=type(error).__name__)
        raise
    finally:
        status['ended_at'] = time.time()
        status_path.write_text(json.dumps(status, indent=2) + '\n')
        print(json.dumps(status), flush=True)


if __name__ == '__main__':
    main()
