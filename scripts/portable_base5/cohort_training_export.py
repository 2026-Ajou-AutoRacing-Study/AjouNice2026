#!/usr/bin/env python3
"""완료한 여섯 환경에 동일한 동결 Q90/upright 전달 규칙을 적용한다.

기존 Sunny13 실행기의 입력·변환·검증·압축을 그대로 호출한다. 별도 wrapper는
명시한 환경과 최종 manifest를 확인하고 전달본 identity만 일반화한다.
원본 PNG·native GT·기존 export는 삭제하거나 수정하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
from types import SimpleNamespace

ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
PARENT_SHA256 = '188dc9126792f97953a68073eaaab9dfe950608bd3ea6236aad81235890e282c'
SCHEMA = 'base5_cohort_borrowed_training_export_v1'


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b''):
            result.update(block)
    return result.hexdigest()


def parent():
    path = Path(__file__).absolute().with_name('borrowed_training_export.py')
    if path.is_symlink() or not path.is_file() or sha(path) != PARENT_SHA256:
        raise ValueError('Exact unchanged Sunny13 export implementation required')
    spec = importlib.util.spec_from_file_location('_base5_original_cohort_export', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cohort_scope(original, args):
    """원형의 모든 입력 검사를 유지하고 환경 identity만 분리한다."""
    original.require(args.cohort in ENVIRONMENTS, 'Explicit supported collection environment required')
    original.require(re.fullmatch(r'base5_' + args.cohort
        + r'_5k_jpeg90_upright3_[0-9]{6}(?:[0-9]{2})?\.tar\.zst', args.archive_name),
        'Archive filename must explicitly identify the requested cohort')
    # The frozen original's sole cohort-specific gate checks this argument.
    # Its ledger, overlay, reference, sensor and path checks remain unchanged.
    checked = argparse.Namespace(**vars(args))
    checked.cohort = 'sunny13'
    repo, root, expected, contract = original.scope(checked)
    finalized = original.read(Path(contract['selected']).parent / 'result.json')
    manifest = original.physical(finalized['manifest'])
    original.require(manifest.parent == Path(contract['selected']).parent
        and manifest.name == 'manifest.json'
        and finalized.get('manifest_sha256') == original.sha(manifest),
        'Finalized same-folder cohort manifest must match its receipt')
    document = original.read(manifest)
    scenes = document.get('scenes', [])
    original.require(document.get('config', {}).get('environment_tag') == args.cohort
        and document.get('count') == len(scenes) == args.expected_scenes,
        'Finalized manifest belongs to another environment or incomplete collection')
    raw = original.physical(contract['selected']).read_bytes()
    original.require(raw.endswith(b'\n'), 'Final selected ledger must be newline terminated')
    rows = [json.loads(line) for line in raw.splitlines()]
    original.require(len(rows) == args.expected_scenes
        and all(row.get('status') == 'accepted_candidate' for row in rows),
        'Exactly 5K native approved observations required')
    for field in ('id', 'capture_root', 'scenario_sha256'):
        original.require(len({row[field] for row in rows}) == args.expected_scenes,
            'Duplicate finalized selection ' + field)
    logical_ids = [row.get('source_cell_id', row['id']) for row in rows]
    original.require(len(set(logical_ids)) == args.expected_scenes
        and {scene['id'] for scene in scenes} == set(logical_ids),
        'Final selected logical slots differ from cohort manifest')
    contract.update(schema=SCHEMA, cohort=args.cohort, driver_sha256=sha(__file__),
        original_export_sha256=PARENT_SHA256, finalized_manifest=str(manifest),
        cohort_identity_policy='Explicit cohort plus finalized manifest; conversion rules unchanged')
    contract['input_pins'][str(manifest)] = original.sha(manifest)
    return repo, root, expected, contract


def execute(args):
    original = parent()
    frozen_scope = original.scope

    def adapted_scope(arguments):
        # Use a detached namespace so the original gate is never bypassed by
        # mutating any shared process arguments or the old module on disk.
        namespace = SimpleNamespace(**vars(original))
        namespace.scope = frozen_scope
        return cohort_scope(namespace, arguments)

    original.scope = adapted_scope
    try:
        result = original.execute(args)
        result.update(cohort=args.cohort, original_png_deleted=False,
            unchanged_conversion_rules=True, original_export_sha256=PARENT_SHA256)
        return result
    finally:
        original.scope = frozen_scope


def parser():
    result = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    result.add_argument('stage', nargs='?', choices=('plan', 'qualify', 'pack', 'validate'), default='plan')
    result.add_argument('--repo', type=Path, default=Path('/home/luv/AjouNice2026'))
    for name in ('export-root', 'selected', 'split-overlay', 'sensor', 'expected-helpers'):
        result.add_argument('--' + name, type=Path, required=True)
    result.add_argument('--cohort', choices=ENVIRONMENTS, required=True)
    result.add_argument('--archive-name', required=True)
    result.add_argument('--expected-scenes', type=int, default=5000)
    result.add_argument('--usage-root', type=Path, action='append', default=[])
    result.add_argument('--max-usage-gib', type=float, default=100.)
    result.add_argument('--min-free-gib', type=float, default=10.)
    result.add_argument('--compression-level', type=int, default=6)
    result.add_argument('--threads', type=int, default=2)
    result.add_argument('--run', action='store_true')
    return result


if __name__ == '__main__':
    try:
        print(json.dumps(execute(parser().parse_args()), ensure_ascii=False, indent=2), flush=True)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        print(json.dumps(dict(event='cohort_training_export_blocked', error=str(error),
            original_png_deleted=False, partial_products_preserved=True), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
