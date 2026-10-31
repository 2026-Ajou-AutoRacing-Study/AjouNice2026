#!/usr/bin/env python3
"""Base5 새 환경의 공통 CPU 생성 진입점. 기존 release나 수집은 변경하지 않는다.

여섯 환경 모두 동일한 버전의 전체 파이프라인을 사용한다. 환경과 seed는
새 배치의 identity이며 정책 선택기가 아니다. 기본 실행은 읽기 전용 계획이고
--run도 CPU 생성만 수행한다. Sunny13 재생성은 새 출력과 명시적 opt-in이 필요하다.
과거 helper는 동결 수집의 재현을 위해 그대로 두고 새 생성은 이 진입점을 쓴다.
"""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import threading

SCRIPTS = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import base5_front_release_inputs as inputs
from base5_capture_safety import atomic_json, sha256
import prepare_base5_unit_environment as assembly


ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
POLICY = 'base5-common-unit-generation-v160'
SCHEMA = 'base5_common_generation_receipt_v1'
ASSEMBLY_POLICY = 'independent-unit-environment-assembly-v159'
ADJACENT_POLICY = 'frozen-ordinary-secondary-legal-neighbor-strata-v1'
SPREAD_POLICY = 'nearest-seven-plus-guarded-source-eighth-v1'
STAGES = (
    'fresh_geometry', 'freeze_capture_split', 'front_support', 'primary_range',
    'far_secondary', 'far_spread', 'static_extent', 'unit_review', 'close_primary',
    'layout_overlay', 'cross_environment_audit',
)
_ADAPTER_LOCK = threading.Lock()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def environment_settings(environment, seed, allow_new_sunny13=False):
    """환경은 기대 UI 설정과 독립 샘플링 identity만 결정한다."""
    require(environment in ENVIRONMENTS, 'One of the six competition environments is required')
    require(type(seed) is int and 0 <= seed < 2**63, 'Explicit nonnegative integer seed required')
    require(type(allow_new_sunny13) is bool, 'Sunny13 opt-in must be a boolean')
    require(environment != 'sunny13' or allow_new_sunny13,
            'New Sunny13 requires --allow-new-sunny13-plan; existing data stays immutable')
    return dict(environment=environment, weather=environment[:-2], hour=int(environment[-2:]), seed=seed)


@contextmanager
def common_assembly_adapter(allow_new_sunny13=False):
    """옛 helper 파일을 고치지 않고 새 프로세스 안에서만 6환경을 연결한다.

    기존 assembly가 임시 함수 adapter를 쓰므로 같은 프로세스의 중첩·동시
    호출은 거부한다. 정상/예외 종료 모두 원래 허용 목록과 함수를 복원한다.
    Sunny13도 나머지 환경과 같은 generate 함수에 기존의 명시적 opt-in만 전달한다.
    """
    require(_ADAPTER_LOCK.acquire(blocking=False), 'Concurrent or nested generation adapters are forbidden')
    original_environments = assembly.ENVIRONMENTS
    original_generate = assembly.fresh.generate

    def generate(source, output, environment, *args, **kwargs):
        if environment == 'sunny13':
            require(allow_new_sunny13, 'Sunny13 generation was not explicitly authorized')
            require(len(args) <= 2, 'Use keyword arguments for generation safety options')
            kwargs['allow_new_sunny13'] = True
        return original_generate(source, output, environment, *args, **kwargs)

    assembly.ENVIRONMENTS = ENVIRONMENTS
    assembly.fresh.generate = generate
    try:
        yield
    finally:
        assembly.ENVIRONMENTS = original_environments
        assembly.fresh.generate = original_generate
        _ADAPTER_LOCK.release()


def policy_contract(request):
    """환경·seed·출력 경로를 제외한 공통 생성 정책과 입력 계보이다."""
    stages = request['stages']
    if stages and isinstance(stages[0], dict):
        # 옛 build receipt의 stages는 계획 문자열 대신 실행별 timing 기록이다.
        # cross-environment audit는 별도 필드이며 실제 성공 여부는 prepare에서 확인한다.
        stages = [stage['stage'] for stage in stages]+['cross_environment_audit']
    require(tuple(stages) == STAGES, 'Underlying generation stages changed; review and version the policy')
    require(request['policy'] == assembly.POLICY == ASSEMBLY_POLICY,
            'Underlying assembly policy differs; review and version the common policy')
    require(assembly.ADJACENT_POLICY == ADJACENT_POLICY and assembly.SPREAD_POLICY == SPREAD_POLICY,
            'Underlying proposal policies differ; review and version the common policy')
    require(request['continue_from_unqualified_far_stage'] is None,
            'Canonical generation always runs every stage; historical CPU continuation is separate')
    return dict(
        policy=POLICY, assembly_policy=ASSEMBLY_POLICY, stages=list(STAGES),
        source_sha256=request['source_sha256'], baseline_sha256=request['baseline_sha256'],
        expected_scenes=request['expected_scenes'],
        adjacent_lane_policy=ADJACENT_POLICY,
        far_spread_proposal_policy=SPREAD_POLICY,
        static_extent_counts=copy.deepcopy(request['static_extent_counts']),
        range_reference=copy.deepcopy(request['range_reference']),
        unit_reference_specs=copy.deepcopy(request['unit_reference_specs']),
        known_map_layout_split=True, capture_split_frozen_before_regeneration=True,
        actor_scale='unit_only', native_qa_relaxed=False,
        environment_specific_policy_branch=False,
    )


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def plan(source, baseline, output, environment, seed, allow_new_sunny13=False):
    settings = environment_settings(environment, seed, allow_new_sunny13)
    with common_assembly_adapter(allow_new_sunny13):
        request = assembly.plan(source, baseline, output, environment, seed)
    policy = policy_contract(request)
    runtime = inputs.collect_runtime(entrypoints=(Path(assembly.__file__).name,))
    runtime[str(Path(__file__).resolve())] = sha256(__file__)
    runtime_identity = {Path(path).name: value for path, value in runtime.items()}
    return dict(
        schema=SCHEMA, policy=POLICY, settings=settings, assembly_request=request,
        shared_policy=policy, shared_policy_sha256=digest(policy),
        runtime_hashes=runtime, implementation_sha256=digest(runtime_identity),
        source_and_baseline_preserved=True, native_qualified=False, bulk_ready=False,
        bulk_started=False, simulator_access=False, remote_access=False,
    )


def prepare(source, baseline, output, environment, seed, allow_new_sunny13=False):
    request = plan(source, baseline, output, environment, seed, allow_new_sunny13)
    with common_assembly_adapter(allow_new_sunny13):
        final = Path(assembly.prepare(source, baseline, output, environment, seed))
    output = Path(request['assembly_request']['output'])
    build_path = output/'build_receipt.json'
    build = assembly.read(build_path)
    require(policy_contract(build) == request['shared_policy'], 'Generated policy differs from the initial plan')
    require(build['environment'] == environment and build['seed'] == seed,
            'Generated environment or seed differs from the initial plan')
    require(Path(build['final_manifest']).resolve() == final.resolve()
            and sha256(final) == build['final_manifest_sha256'], 'Final manifest differs from the assembly receipt')
    deck = assembly.read(final)
    require(deck['config']['environment_tag'] == environment
            and deck['config']['collection_environment']['weather'] == request['settings']['weather']
            and deck['config']['collection_environment']['hour'] == request['settings']['hour'],
            'Final manifest environment differs from the common generation request')
    require(tuple(stage['stage'] for stage in build['stages']) == STAGES[:-1],
            'Canonical generation skipped or reordered a generation stage')
    require(build['cross_environment_audit']['passed'], 'Cross-environment geometry audit failed')
    inputs.verify_hashes(request['runtime_hashes'], 'Common generation implementation')
    for key in ('source', 'baseline'):
        require(sha256(request['assembly_request'][key]) == request['assembly_request'][key+'_sha256'],
                'Immutable generation input changed: '+key)
    receipt = dict(request, status='CPU_ONLY_PASS; native environment qualification required',
        final_manifest=str(final.resolve()), final_manifest_sha256=sha256(final),
        assembly_receipt=dict(path=str(build_path.resolve()), sha256=sha256(build_path)),
        existing_frozen_releases_rewritten=False, completion_live_qualified=False,
        training_started=False,
    )
    atomic_json(output/'common_generation_receipt.json', receipt)
    return final


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='Immutable pre-front v150 richfloor source')
    parser.add_argument('--baseline', type=Path, required=True, help='Immutable current Sunny13 unit reference')
    parser.add_argument('--output', type=Path, required=True, help='New output only; never a capture resume root')
    parser.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--allow-new-sunny13-plan', action='store_true',
                        help='Explicit separate Sunny13 candidate; never rewrites existing Sunny13')
    parser.add_argument('--run', action='store_true', help='CPU generation only; never starts MORAI or collection')
    args = parser.parse_args()
    function = prepare if args.run else plan
    result = function(args.source, args.baseline, args.output, args.environment, args.seed,
                      args.allow_new_sunny13_plan)
    print(result if args.run else json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
