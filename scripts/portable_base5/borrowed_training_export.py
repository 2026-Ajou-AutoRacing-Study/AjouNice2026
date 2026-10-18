#!/usr/bin/env python3
"""빌린 PC의 완료 선택을 같은 Sunny15 CPU 규칙으로 변환·검증·압축한다.

기본은 읽기 전용 계획이다. qualify/pack은 --run이 있어야 새 export에 쓴다.
namespace·원본 RO 보호·의존성 준비는 호출자가 담당한다. 수집·PNG 삭제·SSH·
Drive·학습은 실행하지 않는다. trusted repository pickle만 입력으로 받는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import re

LOGICAL_REPO = Path('/home/luv/AjouNice2026')
SCHEMA = 'base5_borrowed_training_export_v1'
ARCHIVE_NAME = 'base5_sunny13_5k_jpeg90_upright3_261018.tar.zst'
HELPERS = {
    'training_convert_adapter.py':'762c51dbd8c27345a792c19b45bc96c5bd78c62640b4571cc73954de031b0ebc',
    'pack_training.py':'65961499d398f8ec1f35759667d5a8e2972441599d436c508fd171425a9afae4',
    'finish.py':'219e1e04b80182d3574a3d3b5de9f9f207a18e8ba5b21aedbbbc3f1ae60c1fe5',
}
REFERENCE_PATHS = (
    'dataset/morai_od/scripts/qualify_base5_training_inputs.py',
    'docker/bevfusion/app/online_preprocessor.py',
    'dataset/morai_od/scripts/report_base5_obstacle_support.py',
    'dataset/morai_od/scripts/build_base5_support_pilot_infos.py',
    'dataset/morai_od/scripts/base5_upright_boxes.py',
    'dataset/morai_od/scripts/report_base5_compound_evidence.py',
    'perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py',
    'perception/detection3d/bevfusion/tools/prepare_base5_pedestrian_merged.py',
    'perception/detection3d/bevfusion/scripts/benchmark_base5_jpeg_cache.py',
    'perception/detection3d/bevfusion/plugin/morai_jpeg_cache.py',
    'perception/detection3d/bevfusion/tools/prepare_base5_jpeg_portable.py',
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def physical(value, exists=True, directory=False):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and not any(p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink export path required: '+str(path))
    if exists:
        require(path.is_dir() if directory else path.is_file(), 'Missing/wrong export input: '+str(path))
    return path


def sha(path):
    result = hashlib.sha256()
    with physical(path).open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024**2), b''):
            result.update(block)
    return result.hexdigest()


def read(path):
    return json.loads(physical(path).read_text())


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def reference_helpers(path, repo):
    doc = read(path)
    expected = doc.get('helper_sha256', doc)
    require(isinstance(expected, dict) and set(expected) == {str(repo/p) for p in REFERENCE_PATHS}
        and all(isinstance(value, str) and re.fullmatch('[a-f0-9]{64}', value)
                for value in expected.values()), 'Exact Sunny15 CPU helper SHA reference required')
    require({name:sha(name) for name in expected} == expected,
            'Sunny15 CPU helper implementation differs; never silently substitute')
    return expected


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, physical(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def components(repo):
    root = physical(__file__).parent
    require({name:sha(root/name) for name in HELPERS} == HELPERS,
            'Frozen offline adapter/packer/finisher changed')
    adapter = load(root/'training_convert_adapter.py', '_borrowed_frozen_training_conversion')
    adapter.ROOT, adapter.SCRIPTS = repo, repo/'dataset/morai_od/scripts'
    adapter.verify_sources()
    packer = load(root/'pack_training.py', '_borrowed_frozen_training_packer')
    packer.finish.ROOT, packer.finish.SCRIPTS = repo, adapter.SCRIPTS
    return adapter, packer


@contextmanager
def source_readonly_copy(qualification, expected):
    """첫 native→PNG portable 복사만 변경한다. 픽셀/GT/후속 export는 원형이다."""
    portable = qualification.portable
    original = portable.build
    source = physical(inspect.getsourcefile(original))
    require(original.__name__ == 'build' and source == physical(portable.__file__)
        and expected.get(str(source)) == sha(source), 'Unexpected original portable build function/source SHA')
    signature = inspect.signature(original)
    require(tuple(signature.parameters) == ('ledger', 'profiles', 'output', 'expected_scenes', 'copy_mode')
        and signature.parameters['copy_mode'].default == 'copy',
        'Original portable build signature changed')
    calls = []

    def copied(*args, **kwargs):
        bound = signature.bind(*args, **kwargs)
        bound.apply_defaults()
        require(not calls and bound.arguments['copy_mode'] == 'hardlink',
                'Only one original hardlink request may become the source-readonly copy')
        bound.arguments['copy_mode'] = 'copy'
        calls.append(dict(copy_mode='copy', source=str(bound.arguments['ledger']),
                          output=str(bound.arguments['output']), source_sha256=sha(source)))
        return original(*bound.args, **bound.kwargs)

    portable.build = copied
    try:
        yield calls
        require(len(calls) == 1, 'Exactly one native-to-portable copy required')
    finally:
        changed = portable.build is not copied
        portable.build = original
        require(not changed, 'Portable storage-only adapter ownership changed')


def scope(args):
    repo = physical(args.repo, directory=True)
    require(repo == LOGICAL_REPO, 'Use the approved logical repository inside the borrowed namespace')
    require(args.cohort == 'sunny13' and args.expected_scenes == 5000,
            'This delivery is one complete Sunny13 5K, not a partial cohort')
    require(re.fullmatch(r'[A-Za-z0-9_.-]+\.tar\.zst', args.archive_name), 'Safe single archive filename required')
    selected, overlay, sensor, reference = map(physical,
        (args.selected, args.split_overlay, args.sensor, args.expected_helpers))
    require(selected.name == 'derived_selected_ledger.jsonl' and overlay.parent == selected.parent
        and overlay.name == 'split_overlay.json', 'Finalized derived selection and same-folder overlay required')
    finalized = selected.parent/'result.json'
    result = read(finalized)
    require(result.get('frames') == args.expected_scenes
        and result.get('derived_selected_ledger') == str(selected)
        and result.get('derived_selected_ledger_sha256') == sha(selected)
        and result.get('split_overlay') == str(overlay)
        and result.get('split_overlay_sha256') == sha(overlay), 'Final selection/overlay receipt changed')
    root = physical(args.export_root, exists=False)
    for source in (selected.parent, selected, overlay, sensor, reference, repo):
        require(root != source and root not in source.parents and source not in root.parents,
                'Fresh export must not overlap protected source/repository')
    expected = reference_helpers(reference, repo)
    reference_doc = read(reference)
    if 'helper_sha256' in reference_doc:
        require(reference_doc.get('sensor_sha256') == sha(sensor),
                'Sensor calibration source differs from Sunny15 reference')
        expected_adapter = reference_doc.get('conversion_adapter_sha256',
            reference_doc.get('portable_training_conversion_adapter', {}).get('adapter_sha256'))
        require(expected_adapter == HELPERS['training_convert_adapter.py'],
                'Sunny15 conversion adapter reference changed')
    contract = dict(schema=SCHEMA, cohort=args.cohort, expected_scenes=args.expected_scenes,
        logical_repo=str(repo), export_root=str(root), selected=str(selected), split_overlay=str(overlay),
        sensor=str(sensor), reference=str(reference), reference_helpers=expected,
        input_pins={str(path):sha(path) for path in (selected, overlay, sensor, reference, finalized)},
        driver_sha256=sha(__file__), archive_name=args.archive_name,
        storage_policy='Only first native-to-PNG portable build uses copy instead of hardlink; pixels/GT unchanged',
        original_png_deleted=False, capture_started=False, training_started=False)
    return repo, root, expected, contract


def qualification_receipt(root, contract, adapter, packer):
    receipt = read(root/'QUALIFIED.json')
    require(receipt.get('contract') == contract and receipt.get('complete') is True,
            'Qualified delivery contract changed')
    report = root/'qualified/report.json'
    package = root/'qualified'/contract['cohort']/'training3_jpeg_portable'
    require(set(receipt.get('output_pins', {})) == {str(report), str(package/'manifest.json')},
            'Exact qualified report and package manifest pins required')
    for path, digest in receipt.get('output_pins', {}).items():
        require(sha(path) == digest, 'Qualified report/package manifest changed')
    packer.finish.validate_training_package(package, contract['expected_scenes'])
    packer.qualification(report, package, Path(contract['selected']),
                         Path(contract['split_overlay']), contract['expected_scenes'])
    return receipt, report, package


def execute(args):
    repo, root, expected, contract = scope(args)
    require(not args.run or args.stage in ('qualify', 'pack'), '--run applies only to explicit qualify/pack')
    if args.stage == 'plan' or (args.stage in ('qualify', 'pack') and not args.run):
        return dict(event='borrowed_training_export_plan', stage=args.stage, export_root=str(root),
            execute=False, files_written=False, same_sunny15_cpu_helpers=True,
            source_readonly_copy=True, original_png_deleted=False, training_started=False)
    adapter, packer = components(repo)
    if args.stage == 'qualify':
        entries = list(root.iterdir()) if root.is_dir() else []
        require(not root.exists() or root.is_dir() and all(path.name == 'tmp'
            and not path.is_symlink() and path.is_dir() and not any(path.iterdir()) for path in entries),
                'Fresh empty export required; never overwrite a partial or qualified product')
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        write_new(root/'INTENT.json', contract)
        converter, qualification = adapter.load_modules()
        with source_readonly_copy(qualification, expected) as copies:
            result = adapter.run({args.cohort:Path(contract['selected'])}, root/'qualified',
                Path(contract['sensor']), Path(contract['split_overlay']))
        require(result.get('helper_sha256') == expected, 'Executed CPU helper closure differs from Sunny15 reference')
        require(result.get('unique_raw_observations') == args.expected_scenes,
                'Qualified frame count differs from final 5K')
        report = root/'qualified/report.json'
        package = root/'qualified'/args.cohort/'training3_jpeg_portable'
        packer.finish.validate_training_package(package, args.expected_scenes)
        packer.qualification(report, package, Path(contract['selected']),
                             Path(contract['split_overlay']), args.expected_scenes)
        require(all(sha(path) == digest for path, digest in contract['input_pins'].items()),
                'Pinned delivery input changed during qualification')
        receipt = dict(contract=contract, complete=True, storage_only_copy=copies,
            output_pins={str(path):sha(path) for path in (report, package/'manifest.json')},
            source_unchanged=True, native_payload_deleted=False)
        write_new(root/'QUALIFIED.json', receipt)
        return dict(event='borrowed_training_export_qualified', frames=args.expected_scenes,
                    report=str(report), package=str(package), source_unchanged=True)
    receipt, report, package = qualification_receipt(root, contract, adapter, packer)
    if args.stage == 'validate':
        return dict(event='borrowed_training_export_validated', frames=args.expected_scenes,
                    package=str(package), files_written=False, original_png_deleted=False)
    require(args.stage == 'pack', 'Unknown offline stage')
    argv = ['--package', str(package), '--qualification-report', str(report),
        '--selected', contract['selected'], '--split-overlay', contract['split_overlay'],
        '--output', str(root/args.archive_name), '--receipt-dir', str(root/'archive_receipt'),
        '--expected-scenes', str(args.expected_scenes), '--max-usage-gib', str(args.max_usage_gib),
        '--min-free-gib', str(args.min_free_gib), '--usage-root', str(root),
        '--compression-level', str(args.compression_level), '--threads', str(args.threads), '--run']
    for path in args.usage_root:
        argv += ['--usage-root', str(physical(path, directory=True))]
    return packer.pack(packer.parser().parse_args(argv))


def parser():
    result = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    result.add_argument('stage', nargs='?', choices=('plan', 'qualify', 'pack', 'validate'), default='plan')
    result.add_argument('--repo', type=Path, default=LOGICAL_REPO)
    for name in ('export-root', 'selected', 'split-overlay', 'sensor', 'expected-helpers'):
        result.add_argument('--'+name, type=Path, required=True)
    result.add_argument('--cohort', choices=('sunny13',), default='sunny13')
    result.add_argument('--expected-scenes', type=int, default=5000)
    result.add_argument('--archive-name', default=ARCHIVE_NAME)
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
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        print(json.dumps(dict(event='borrowed_training_export_blocked', error=str(error),
            original_png_deleted=False, partial_products_preserved=True), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
