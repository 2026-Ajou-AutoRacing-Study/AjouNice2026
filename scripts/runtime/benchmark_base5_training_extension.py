#!/usr/bin/env python3
"""Run the frozen bounded train benchmark with a separately qualified extension.

This adds no optimizer/scheduler or source changes. The original E1 full-resume
and ordered-configuration checks stay in the common benchmark. No real epoch.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
COMMON_SHA256 = '3e49125952ce5bf936ca2965140c9ba8aaa0268b8246011e8e6478c287fed42e'
VERSION = 'base5_skip_discarded_heatmap_v1'


def require(condition, message):
    if not condition: raise ValueError(message)


def validate_qualification(proof, extension_sha, checkpoint_sha, contract):
    require(proof.get('complete') is True and proof.get('valid') is True
            and proof.get('expected_fixture_batches') == 3
            and proof.get('contract_sha256') == contract
            and proof.get('checkpoint_sha256') == checkpoint_sha,
            'Exact complete three-fixture E1 qualification required')
    require(proof.get('extension', {}).get('version') == VERSION
            and proof['extension'].get('script_sha256') == extension_sha
            and proof.get('compile_camera') is False
            and all(proof.get(name) is True for name in
                    ('no_optimizer_step', 'no_checkpoint_write', 'model_original_unmodified')),
            'Qualification extension or isolation contract differs')
    rows = proof.get('fixtures', [])
    require(len(rows) == 3 and all(
        all(row.get(name) is True for name in
            ('targets_exact', 'losses_exact', 'finite', 'RNG_exact'))
        and row.get('gradient', {}).get('exact') is True
        and row['gradient'].get('max_abs') == 0.
        and row.get('baseline_losses') == row.get('candidate_losses')
        for row in rows), 'Target/loss/gradient/RNG equality must pass on every fixture')


def main(argv=None):
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--training-extension', type=Path, required=True)
    parser.add_argument('--training-extension-sha256', required=True)
    parser.add_argument('--qualification-receipt', type=Path, required=True)
    parser.add_argument('--qualification-sha256', required=True)
    own, forwarded = parser.parse_known_args(argv)
    path = Path(__file__).with_name('benchmark_base5_full_runtime.py')
    spec = importlib.util.spec_from_file_location('_training_extension_common', path)
    common = importlib.util.module_from_spec(spec); spec.loader.exec_module(common)
    require(common.digest(path) == COMMON_SHA256, 'Frozen bounded benchmark source changed')
    args = common.parse_args(forwarded)
    require(args.mode == 'train', 'Only the isolated bounded training mode is supported')
    require(own.training_extension.is_absolute() and own.qualification_receipt.is_absolute()
            and common.digest(own.training_extension) == own.training_extension_sha256
            and common.digest(own.qualification_receipt) == own.qualification_sha256,
            'Explicit qualification and extension checksums required')
    proof = json.loads(own.qualification_receipt.read_text())
    validate_qualification(proof, own.training_extension_sha256,
                           common.digest(args.checkpoint), common.QUALIFIED_CONTRACT)
    extension = common.load_module(own.training_extension, '_qualified_training_cost_extension')
    require(extension.VERSION == VERSION, 'Unknown training extension')
    original = common.reconstruct

    def reconstruct(arguments):
        cfg, manifest = original(arguments)
        require(manifest['contract_sha256'] == proof['contract_sha256'],
                'Reconstructed ordered model differs from fixture qualification')
        extension.install()
        return cfg, manifest

    common.reconstruct = reconstruct
    try:
        common.main(forwarded)
        report_path = args.output/'report.json'
        report = json.loads(report_path.read_text())
        require(report.get('valid') is True and report.get('mode') == 'train',
                'Bounded optimizer-cost comparison did not complete')
        report['training_extension'] = dict(**extension.receipt(),
            qualification_receipt=str(own.qualification_receipt),
            qualification_sha256=own.qualification_sha256)
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    finally:
        extension.uninstall(); common.reconstruct = original


if __name__ == '__main__': main()
