#!/usr/bin/env python3
"""Native 승인 장면과 실제 crop-positive GT를 분리하여 학습 입력을 만든다.

동결 변환기의 primary-positive 장면 필터만 좁게 대체한다. primary가 현재
crop에서 지원되지 않더라도 지원되는 보조 GT가 있으면 장면을 보존한다.
지원되지 않는 GT를 되살리거나, actor가 있는데 GT가 전혀 없는 장면을
background로 바꾸지 않는다. 원본·수집 승인·분할·개별 GT 정책은 그대로다.
"""
import argparse
import ast
from contextlib import contextmanager
import hashlib
import inspect
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'dataset/morai_od/scripts'
POLICY = 'native-approved-frame-with-current-positive-components-v1'
PINS = {
    'build_base5_bevfusion_infos.py': '8a204bb2acd126371f9ca5d877e16a0dc66b464e19dd38d1b7307445a19d5795',
    'build_base5_support_pilot_infos.py': '6b1b5d7cb6ab43892d694f9dc42c428ba0ef019f3184fb928dd4138170af6660',
    'qualify_base5_training_inputs.py': 'cb61018ee08dad425008a1e55fc5ff957280757647bb4aeda0c4164e2f0ec6d9',
}
PRIMARY_GUARD = """
if meta['runtime_actors']:
    primary_uid=meta['runtime_actors'][0]['unique_id']
    if not any(c['parent_uid']==primary_uid and c['positive_'+policy] and in_roi(c['center_lidar_m'],roi) for c in audit['components']):
        review.append(scene_id)
        continue
"""


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_sources():
    actual = {name: digest(SCRIPTS / name) for name in PINS}
    require(actual == PINS, 'Frozen qualification source changed; adapter must be reviewed')
    return actual


def load_modules():
    verify_sources()
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    import build_base5_bevfusion_infos as converter
    import qualify_base5_training_inputs as qualification
    require(Path(converter.__file__).resolve() == SCRIPTS / 'build_base5_bevfusion_infos.py',
            'Unexpected converter module binding')
    require(Path(qualification.__file__).resolve() == SCRIPTS / 'qualify_base5_training_inputs.py',
            'Unexpected qualification module binding')
    return converter, qualification


def adapted_convert(original):
    """Compile a detached copy after an exact SHA and AST gate match."""
    source = Path(inspect.getsourcefile(original)).resolve()
    require(source == SCRIPTS / 'build_base5_bevfusion_infos.py'
            and digest(source) == PINS[source.name], 'Unexpected original conversion function')
    tree = ast.parse(source.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'convert']
    require(len(functions) == 1 and original.__name__ == 'convert', 'Expected one original convert function')
    function = functions[0]
    expected = ast.dump(ast.parse(PRIMARY_GUARD).body[0], include_attributes=False)
    matching = [node for node in ast.walk(function) if isinstance(node, ast.If)
                and ast.dump(node, include_attributes=False) == expected]
    require(len(matching) == 1, 'Expected one exact primary-positive admission guard')
    guard = matching[0]
    # Keep the same primary/crop/ROI predicate. Record its exception instead of
    # discarding a frame whose other components still have valid supervision.
    # The later original `runtime_actors and not instances` rejection remains.
    guard.body[1].body = ast.parse('__portable_primary_crop_exclusions.append(scene_id)').body
    code = compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                   str(source), 'exec')

    def convert(*args, **kwargs):
        exceptions = []
        scope = dict(original.__globals__, __portable_primary_crop_exclusions=exceptions)
        exec(code, scope)
        report = scope['convert'](*args, **kwargs)
        retained = [ident for ident in exceptions if ident not in report['excluded_review_scenes']]
        report.update(frame_admission_policy=POLICY,
            primary_crop_excluded_but_other_gt_retained=retained,
            native_approval_and_component_positive_policy_unchanged=True,
            unsupported_primary_forced_positive=False,
            actor_scenes_with_no_current_positive_gt_still_excluded=True,
            portable_adapter_sha256=digest(__file__), frozen_converter_sha256=digest(source))
        output = Path(args[1] if len(args) > 1 else kwargs['output'])
        (output / 'conversion_audit.json').write_text(json.dumps(report, indent=2) + '\n')
        return report
    return convert


@contextmanager
def scoped_conversion(converter, qualification):
    """Use only the reviewed import binding and restore it even after failure."""
    verify_sources()
    derive = qualification.derive_infos
    require(Path(inspect.getsourcefile(derive)).resolve() == SCRIPTS / 'build_base5_support_pilot_infos.py',
            'Unexpected derived-info function binding')
    namespace = derive.__globals__
    original = namespace.get('convert')
    require(original is converter.convert, 'Converter already replaced; refusing nested or foreign adapter')
    replacement = adapted_convert(original)
    namespace['convert'] = replacement
    try:
        yield
    finally:
        namespace['convert'] = original


def run(cohorts, output, sensor, split_overlay):
    converter, qualification = load_modules()
    with scoped_conversion(converter, qualification):
        result = qualification.run(cohorts, output, sensor, split_overlay=split_overlay)
    adapter = dict(policy=POLICY, adapter_sha256=digest(__file__), frozen_sources=verify_sources(),
        changes='Frame admission only; retain approved frames with any supported current GT.',
        unchanged=['native approval', 'component visibility', 'ROI', 'upright geometry',
                   'class mapping', 'split/group binding', 'nonempty actor-scene supervision'])
    result['portable_training_conversion_adapter'] = adapter
    for name, cohort in result['cohorts'].items():
        report = qualification.read(Path(output) / name / 'derived/conversion_report.json')
        cohort['portable_training_conversion_adapter'] = adapter
        cohort['primary_crop_excluded_but_other_gt_retained'] = report[
            'primary_crop_excluded_but_other_gt_retained']
        qualification.atomic_json(Path(output) / name / 'report.json', cohort)
    qualification.atomic_json(Path(output) / 'report.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cohort', action='append', required=True, help='name=/selected_ledger.jsonl')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sensor', type=Path, default=ROOT / 'configs/sensors/Sensor_competition_local.json')
    parser.add_argument('--split-overlay', type=Path, required=True)
    args = parser.parse_args()
    cohorts = dict(item.split('=', 1) for item in args.cohort)
    require(len(cohorts) == len(args.cohort), 'Duplicate cohort name')
    result = run(cohorts, args.output, args.sensor, args.split_overlay)
    print(json.dumps(dict(frames=result['unique_raw_observations'], elapsed_sec=result['elapsed_sec'],
                         adapter=result['portable_training_conversion_adapter']), indent=2))


if __name__ == '__main__':
    main()
