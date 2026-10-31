#!/usr/bin/env python3
"""같은 정적 배치의 시간대 RGB 비교를 새 진단 manifest에 등록한다.

MORAI 환경을 자동 설정하거나 실행하지 않는다. 이 표본은 독립 학습 데이터나
새 환경의 본수집 승인이 아니며 기존 payload·metadata·split을 수정하지 않는다.
"""
import argparse
import copy
import json
from pathlib import Path

import base5_layout_split as layout
import base5_shadow_ground_reuse as shadow
from base5_capture_safety import atomic_json, sha256

DEFAULT_SUFFIXES = ('03669', '00693', '03598', '04241', '04938', '01744',
    '00734', '03271', '01205', '03077', '04097', '02554', '04429', '03169',
    '00928', '01568')
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')


def read(path):
    return json.loads(Path(path).read_text())


def select(source, baseline_rows, suffixes):
    if not 1 <= len(suffixes) <= 32 or len(suffixes) != len(set(suffixes)):
        raise ValueError('중복 없는 1~32개 정적 비교 ID가 필요하다')
    entries = {e['id']: e for e in source['scenes']}
    baselines = {r['id']: r for r in baseline_rows}
    if len(entries) != len(source['scenes']) or len(baselines) != len(baseline_rows):
        raise ValueError('source 또는 baseline ID가 중복됐다')
    selected, evidence = [], []
    for suffix in suffixes:
        matches = [ident for ident in baselines if ident == suffix or ident.endswith('_'+suffix)]
        if len(matches) != 1 or matches[0] not in entries:
            raise ValueError('baseline과 source에서 유일한 ID를 찾지 못했다: '+suffix)
        ident = matches[0]; entry = entries[ident]; baseline = baselines[ident]
        meta = read(entry['metadata'])
        if (baseline.get('status') != 'accepted_candidate'
                or any(a['motion'] not in ('static', 'standing') for a in meta['runtime_actors'])
                or shadow.KEY in meta):
            raise ValueError('정적·정상 baseline만 RGB 대조에 사용할 수 있다: '+ident)
        root = Path(baseline['capture_root']).resolve(strict=True)
        expected = entry['scenario_sha256']
        if any(value != expected for value in (sha256(entry['scenario']),
                baseline['scenario_sha256'], sha256(root/'scenario.json'))):
            raise ValueError('baseline과 source payload가 다르다: '+ident)
        # 원본의 정밀 성분/pose/센서 근거도 관측 전에 고정한다.
        pins = {str(p.resolve()): sha256(p) for p in sorted(root.rglob('*')) if p.is_file()}
        for name in ('metadata.json', 'telemetry.json', 'sensor_reference.json', 'audit.json'):
            if str(root/name) not in pins:
                raise ValueError('baseline 근거가 없다: '+name)
        selected.append(copy.deepcopy(entry))
        evidence.append(dict(id=ident, scenario_sha256=expected, baseline_capture=str(root),
            baseline_files=pins, original_training_split=entry.get('derived_training_split', entry['split']),
            original_layout_group=entry.get('derived_layout_group_id')))
    return selected, evidence


def prepare(source, baseline_ledger, output, environment, confirmed_environment, suffixes=DEFAULT_SUFFIXES):
    source, baseline_ledger, output = map(lambda p: Path(p).resolve(), (source, baseline_ledger, output))
    if environment not in ENVIRONMENTS or confirmed_environment != environment:
        raise ValueError('사용자가 설정한 UI 환경을 명시적으로 일치시켜야 한다')
    if output.exists():
        raise FileExistsError('RGB pilot은 새 output에 등록한다')
    deck = read(source)
    layout.validate_manifest_overlay(deck)
    if environment == deck['config'].get('environment_tag'):
        raise ValueError('다른 시간/날씨의 대조 환경을 지정한다')
    rows = [json.loads(line) for line in baseline_ledger.read_text().splitlines() if line.strip()]
    for baseline in rows:
        root = Path(baseline['capture_root']).resolve(strict=True)
        if output == root or root in output.parents:
            raise ValueError('원본 capture 내부에는 진단 manifest도 만들 수 없다')
    selected, evidence = select(deck, rows, suffixes)
    candidate = copy.deepcopy(deck)
    candidate.update(count=len(selected), scenes=selected,
        status='STATIC_PAIRED_RGB_DIAGNOSTIC_ONLY', bulk_ready=False,
        bulk_started=False, training_started=False)
    config = candidate['config']
    original_environment = config.get('environment_tag')
    # environment_tag는 far/shadow 등 동결 geometry 계약의 source identity다.
    # 이를 바꾸거나 검사기를 제거하지 않고 실제 촬영 환경을 별도 기록한다.
    config.update(collection_environment=dict(
        weather=environment[:-2], hour=int(environment[-2:]),
        operator_confirmation=confirmed_environment,
        authority='User-set MORAI UI; declaration, not automatic readback or environment setter'))
    registration = dict(schema='base5_static_environment_rgb_pilot_v1',
        source_manifest=dict(path=str(source), sha256=sha256(source)),
        baseline_ledger=dict(path=str(baseline_ledger), sha256=sha256(baseline_ledger)),
        script_sha256=sha256(__file__), source_environment=original_environment,
        capture_environment=config['collection_environment'], count=len(selected), pairs=evidence,
        static_only=True, exact_geometry_preserved=True, source_metadata_preserved=True,
        existing_layout_group_and_split_preserved=True, diagnostic_only=True,
        independent_training_samples=0, native_gates_unchanged=True, bulk_ready=False,
        source_environment_tag_preserved_for_geometry_contract=True,
        limitations=['UI time/weather and graphics options are not programmatically read back.',
            'Existing source metadata retains its historical environment lineage.',
            'config.environment_tag identifies the frozen source geometry; collection_environment is the physical capture declaration.',
            'Paired layouts must not be counted as independent train/val samples.'])
    candidate['environment_rgb_pilot'] = dict(source_environment=original_environment,
        physical_capture_environment=environment, diagnostic_only=True,
        independent_training_samples=0, registration_path=str(output/'registration.json'))
    layout.validate_manifest_overlay(candidate, allow_subset=True)
    output.mkdir(parents=True)
    atomic_json(output/'registration.json', registration)
    atomic_json(output/'manifest.json', candidate)
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--baseline-ledger', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    parser.add_argument('--confirmed-environment', choices=ENVIRONMENTS, required=True)
    parser.add_argument('--ids', help='정적 baseline ID 또는 마지막 번호, 쉼표 구분')
    args = parser.parse_args()
    print(prepare(args.source, args.baseline_ledger, args.output, args.environment,
        args.confirmed_environment, tuple(args.ids.split(',')) if args.ids else DEFAULT_SUFFIXES))
