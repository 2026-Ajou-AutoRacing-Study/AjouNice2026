#!/usr/bin/env python3
"""Sequential, matched training-cost ablations INSIDE an existing GPU image.

No remote access, full epoch, checkpoint promotion or dataset mutation. Each
child starts from the same audited initialization and augmentation RNG seed.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


CASES = {
    'baseline': [],
    'warp': ['--pool-backward', 'warp'],
    'compact': ['--compact-indices'],
    'fused': ['--fused-adamw'],
    'logs': ['--batched-loss-log'],
    'combined': ['--pool-backward', 'warp', '--compact-indices',
                 '--batched-loss-log', '--fused-adamw'],
    'grouped': ['--pool-backward', 'grouped', '--compact-indices',
                '--batched-loss-log', '--fused-adamw'],
    'autotune': ['--pool-backward', 'grouped', '--compact-indices',
                 '--batched-loss-log', '--fused-adamw', '--cudnn-benchmark'],
    'combined-autotune': ['--pool-backward', 'warp', '--compact-indices',
                          '--batched-loss-log', '--fused-adamw', '--cudnn-benchmark'],
    'compiled': ['--pool-backward', 'warp', '--compact-indices',
                 '--batched-loss-log', '--fused-adamw', '--geometry-mode', 'batched_checked', '--compile-camera'],
    'geometry': ['--pool-backward', 'warp', '--compact-indices',
                 '--batched-loss-log', '--fused-adamw', '--geometry-mode', 'batched_checked'],
    'geometry-only': ['--geometry-mode', 'batched_checked'],
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--init-checkpoint', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--batch', type=int, default=2)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--steps', type=int, default=64)
    p.add_argument('--warmup', type=int, default=16)
    p.add_argument('--depth-step', type=float, choices=[.25, .5, 1.], default=.5)
    p.add_argument('--spatial-case', choices=['current075','common050','common075','common100'])
    p.add_argument('--cases', nargs='+', choices=list(CASES), default=['baseline','geometry'])
    p.add_argument('--case-timeout-sec',type=int,default=600)
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--profile-steps', type=int, default=0)
    p.add_argument('--audit-gradients', action='store_true')
    a = p.parse_args()
    if a.output.exists() or not 1 <= a.repeats <= 3 or not 60<=a.case_timeout_sec<=1800:
        p.error('Fresh output required; at most three repeats')
    a.output.mkdir(parents=True)
    report = dict(scope='bounded local/remote cost qualification, not accuracy',
                  arguments={k: str(v) if isinstance(v, Path) else v
                             for k, v in vars(a).items()}, results=[])
    reference = None;reference_parameters=None;reference_input=None
    for repeat in range(a.repeats):
        # Reverse order on the next repetition to reduce warm/clock-order bias.
        for case in (a.cases if repeat % 2 == 0 else list(reversed(a.cases))):
            name = f'{case}_r{repeat + 1}'
            work = a.output / name
            cmd = [sys.executable, str(Path(__file__).with_name('benchmark_base5_real_loader.py')),
                   '--data', str(a.data), '--config', str(a.config), '--profile', '512x1056',
                   '--work-dir', str(work), '--init-checkpoint', str(a.init_checkpoint),
                   '--batch', str(a.batch), '--workers', str(a.workers), '--steps', str(a.steps),
                   '--warmup', str(a.warmup), '--depth-step', str(a.depth_step), *CASES[case]]
            cmd.extend(['--profile-steps',str(a.profile_steps)])
            if a.audit_gradients:cmd.append('--audit-gradients')
            if a.spatial_case:cmd.extend(['--spatial-case',a.spatial_case])
            print(f'START {name}: batch={a.batch}, depth={a.depth_step}', flush=True)
            with (a.output / (name + '.log')).open('w') as log:
                child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                timed_out=False
                try:code=child.wait(timeout=a.case_timeout_sec)
                except subprocess.TimeoutExpired:
                    timed_out=True
                    os.killpg(child.pid,signal.SIGTERM)
                    try:code=child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL);code=child.wait()
            if code or timed_out:
                report['results'].append(dict(name=name,valid=False,returncode=code,timeout=timed_out))
            else:
                cost = json.loads((work / 'cost.json').read_text())
                identity = [(r['scene_ids'], r['gt_geometry_sha256']) for r in cost['per_step']]
                if reference is None:
                    reference = identity
                    reference_parameters=cost['initial_parameters_sha256']
                    reference_input=cost['first_input_sha256']
                if identity != reference:
                    raise ValueError('Ablation inputs/augmented GT differ; comparison invalid')
                if cost['initial_parameters_sha256']!=reference_parameters or cost['first_input_sha256']!=reference_input:
                    raise ValueError('Ablation initialization/first sensor inputs differ')
                report['results'].append(dict(name=name, valid=cost['valid'],
                    timing=cost['timing'], peak_allocated_mib=cost['peak_allocated_mib'],
                    optimizer_steps=cost['optimizer_steps'],
                    augmented_gt_and_sample_order_identical=True,
                    initial_parameters_sha256=cost['initial_parameters_sha256'],
                    first_input_sha256=cost['first_input_sha256'],
                    bounded_config_sha256=cost['bounded_config_sha256']))
                print(f"DONE {name}: {cost['timing']['total_ms']['mean']:.2f} ms/step", flush=True)
            (a.output / 'matrix.json').write_text(json.dumps(report, indent=2) + '\n')
    if any(not r['valid'] for r in report['results']):
        raise SystemExit('Some probes failed; see per-case logs, do not silently adopt')


if __name__ == '__main__':
    main()
