#!/usr/bin/env python3
"""Recover complete E1 validation locally on the existing GPU, without training.

The exact backed-up E1 snapshot is evaluated over all original 4,614 frames.
Ordered source preparation and canonical metrics remain unchanged. No W&B,
optimizer step, checkpoint save, source mutation or partial-subset AP claim.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
E1_SHA256 = '471e1e9c5a2c2c3a92330dc858570109644286bc3ea15fd74f54382b0274ba36'
E1_BYTES = 521087941
E1_EPOCH = 1
E1_ITER = 6534
FRAMES = 4614


def metric_key(prefix, name):
    return f'{prefix}/{name}' if prefix else name


def memory_kib():
    result = {}
    path = Path('/proc/self/smaps_rollup')
    if path.is_file():
        for line in path.read_text().splitlines():
            key, _, value = line.partition(':')
            if key in ('Rss', 'Pss', 'Private_Clean', 'Private_Dirty'):
                result[key] = int(value.split()[0])
    return result


def main(argv=None):
    own_parser = argparse.ArgumentParser(allow_abbrev=False)
    own_parser.add_argument('--val-batch', type=int, choices=(1, 2, 4), default=1)
    own, forwarded = own_parser.parse_known_args(argv)
    path = Path(__file__).with_name('benchmark_base5_full_runtime.py')
    spec = importlib.util.spec_from_file_location('_full_val_recovery_common', path)
    common = importlib.util.module_from_spec(spec); spec.loader.exec_module(common)
    args = common.parse_args(forwarded)
    common.require(args.mode == 'val' and args.owned_storage_extension is not None
                   and not args.val_amp, 'FP32 val-mode and qualified owned-storage fix required')
    common.require(args.checkpoint.stat().st_size == E1_BYTES
                   and common.digest(args.checkpoint) == E1_SHA256,
                   'Only the exact backed-up full E1 snapshot is permitted')
    import torch
    from mmengine.hooks import Hook
    torch.set_num_threads(args.torch_threads)
    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
    common.require(checkpoint.get('base5_contract_sha256') == common.QUALIFIED_CONTRACT
                   and checkpoint['meta']['epoch'] == E1_EPOCH
                   and checkpoint['meta']['iter'] == E1_ITER,
                   'E1 checkpoint metadata/contract differs')
    del checkpoint
    args.output.mkdir(parents=True)
    cfg, manifest = common.reconstruct(args)
    cfg.val_dataloader.batch_size = own.val_batch
    cfg.val_cfg['fp16'] = False
    prefix = cfg.val_evaluator.get('prefix')
    center_key = metric_key(prefix, 'center/mAP')
    from mmengine.utils import import_modules_from_strings
    import_modules_from_strings(**cfg.custom_imports)
    from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
    report = dict(schema='base5_full_e1_validation_recovery_v1', complete=False, valid=False,
        evaluated_checkpoint_epoch=E1_EPOCH, evaluated_checkpoint_iteration=E1_ITER,
        checkpoint=str(args.checkpoint), checkpoint_sha256=E1_SHA256,
        original_contract_sha256=manifest['contract_sha256'], expected_frames=FRAMES,
        effective_val_batch=own.val_batch, fp16=False, workers=args.workers,
        pin_memory=args.pin_memory, prefetch_factor=args.prefetch,
        owned_storage_extension_sha256=args.owned_storage_sha256,
        qualified_manifest_sha256=args.qualified_manifest_sha256,
        metric_prefix=prefix, center_mAP_key=center_key,
        wandb_writes=False, optimizer_steps=0, checkpoint_saved=False,
        original_work_modified=False, metrics=None, progress=[])

    def save():
        (args.output/'recovery.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')

    class Capture(Hook):
        priority = 'VERY_HIGH'
        def before_val(self, runner):
            self.frames = 0; self.started = time.perf_counter()
            self.peak_allocated = self.peak_reserved = 0.
            report['initial_open_fds'] = common.open_fd_count()
            report['initial_memory_kib'] = memory_kib(); save()

        def after_val_iter(self, runner, batch_idx, data_batch=None, outputs=None):
            self.frames += len(data_batch['data_samples'])
            self.peak_allocated = max(self.peak_allocated, torch.cuda.max_memory_allocated()/2**20)
            self.peak_reserved = max(self.peak_reserved, torch.cuda.max_memory_reserved()/2**20)
            if (batch_idx+1) % 100 == 0 or self.frames == FRAMES:
                row = dict(batch=batch_idx+1, frames=self.frames,
                    elapsed_seconds=time.perf_counter()-self.started,
                    open_fds=common.open_fd_count(), memory_kib=memory_kib())
                report['progress'].append(row); report['observed_frames'] = self.frames; save()
                print(json.dumps(dict(event='full_e1_val_progress', **row)), flush=True)

        def after_val_epoch(self, runner, metrics=None):
            common.require(self.frames == FRAMES, 'The complete original 4,614 frames are required')
            common.require(metrics is not None and center_key in metrics,
                           'Canonical full validation metrics are missing')
            expected = {name: 750 for name in ('sunny11', 'sunny13', 'sunny15',
                                               'foggy11', 'foggy13', 'foggy15')}
            common.require(all(metrics.get(metric_key(prefix, f'environment/{name}/frames')) == count
                               for name, count in expected.items())
                           and metrics.get(metric_key(prefix, 'supplement/sunny_obstacle/frames')) == 114,
                           'Six environment/supplement frame accounting differs')
            report.update(metrics={key: float(value) for key, value in metrics.items()},
                observed_frames=self.frames, elapsed_seconds=time.perf_counter()-self.started,
                final_open_fds=common.open_fd_count(), final_memory_kib=memory_kib(),
                peak_allocated_mib=self.peak_allocated, peak_reserved_mib=self.peak_reserved,
                canonical_full_token_identity_checked=True, environment_frames=expected,
                supplement_frames=114)
            save()

    runner = Base5AtomicRunner.from_cfg(cfg)
    runner.register_hook(Capture())
    try:
        runner.val()
        common.require(report['observed_frames'] == FRAMES and report['metrics'] is not None,
                       'Validation did not produce complete metrics')
        common.require(common.digest(args.checkpoint) == E1_SHA256,
                       'Checkpoint bytes changed during read-only validation')
        report.update(complete=True, valid=True); save()
    except Exception as error:
        report.update(error_type=type(error).__name__, error=str(error)); save(); raise
    print(json.dumps(dict(event='full_e1_val_recovered', complete=True,
        evaluated_checkpoint_epoch=1, frames=FRAMES, center_mAP=report['metrics'][center_key],
        elapsed_seconds=report['elapsed_seconds'], wandb_writes=False)), flush=True)


if __name__ == '__main__': main()
