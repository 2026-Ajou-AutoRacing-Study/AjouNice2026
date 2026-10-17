#!/usr/bin/env python3
"""신규 세션에서만 사용하는 opt-in 정적 지면 연결 판정 실행기.

원형 collector 파일은 수정하지 않는다. 모든 기존 QA와 최종 telemetry 기록
뒤, audit/ledger 기록 직전에 좁은 full-RPY 불확실성 분류만 별도로 적용한다.
과거 audit/ledger를 수정하거나 기존 세션에 이 adapter를 붙이지 않는다.
"""
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path
import sys

import base5_front_release_inputs as pins
from base5_capture_safety import atomic_json, sha256
import base5_static_surface_attitude_candidate as candidate
from run_base5_capture_optimized import SOURCE, static_fast_ready

POLICY = 'opt-in-full-rpy-static-surface-runtime-v1'
SCHEMA = 'base5_static_surface_attitude_runtime_v1'
BASE_AUDIT = 'audit_original_static_ground.json'
DECISION = 'static_surface_attitude_decision.json'
RECEIPT = 'static_surface_attitude_receipt.json'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def json_digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def is_atomic(node, filename):
    return (isinstance(node,ast.Expr) and isinstance(node.value,ast.Call)
        and isinstance(node.value.func,ast.Name) and node.value.func.id == 'atomic_json'
        and len(node.value.args) >= 2 and isinstance(node.value.args[0],ast.BinOp)
        and isinstance(node.value.args[0].op,ast.Div) and isinstance(node.value.args[0].left,ast.Name)
        and node.value.args[0].left.id == 'dest' and isinstance(node.value.args[0].right,ast.Constant)
        and node.value.args[0].right.value == filename)


def compiled_source(setup_profile, expected_source_sha256, source=SOURCE):
    require(setup_profile in ('baseline','static-ready'), 'Known setup profile required')
    require(sha256(source) == expected_source_sha256, 'Frozen collector SHA changed')
    tree = ast.parse(Path(source).read_text())
    anchors = []
    for parent in ast.walk(tree):
        for value in ast.iter_fields(parent):
            statements = value[1]
            if isinstance(statements,list):
                for index,node in enumerate(statements):
                    if is_atomic(node,'audit.json'):
                        require(index > 0 and is_atomic(statements[index-1],'metadata.json')
                            and any(is_atomic(prior,'telemetry.json') for prior in statements[:index]),
                            'Final audit hook must follow final telemetry and metadata writes')
                        require(isinstance(node.value.args[1],ast.Name) and node.value.args[1].id == 'audit',
                                'Final audit RAM variable changed')
                        anchors.append(node)
    require(len(anchors) == 1, 'Exactly one final audit write anchor required')
    counts = dict(delay=0,options=0,audit=0)

    class Adapter(ast.NodeTransformer):
        def visit_Expr(self,node):
            self.generic_visit(node)
            if is_atomic(node,'audit.json'):
                counts['audit'] += 1
                hook = ast.parse('audit = _surface_attitude_audit(dest, meta, audit)').body[0]
                return [ast.copy_location(hook,node),node]
            if (isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute)
                    and isinstance(node.value.func.value,ast.Name) and node.value.func.value.id == 'time'
                    and node.value.func.attr == 'sleep' and len(node.value.args) == 1
                    and isinstance(node.value.args[0],ast.Constant) and node.value.args[0].value == .7):
                counts['delay'] += 1
                if setup_profile == 'static-ready':
                    condition = ast.parse('not _static_fast_ready(meta, state, load_time)',mode='eval').body
                    return ast.copy_location(ast.If(test=condition,body=[node],orelse=[]),node)
            return node

        def visit_Assign(self,node):
            self.generic_visit(node)
            if any(isinstance(target,ast.Name) and target.id == 'runner_options' for target in node.targets):
                counts['options'] += 1
                receipt = ast.parse('runner_options.update(static_surface_attitude_adapter=_adapter_contract)').body[0]
                return [node,ast.copy_location(receipt,node)]
            return node

    tree = Adapter().visit(tree)
    require(counts == dict(delay=1,options=1,audit=1), 'Frozen collector AST shape changed: '+str(counts))
    require(sha256(source) == expected_source_sha256, 'Collector changed during adapter compilation')
    return compile(ast.fix_missing_locations(tree),str(source),'exec')


def configuration(manifest, enabled, allow_pedestrians, setup_profile):
    require(enabled or not allow_pedestrians, 'Static pedestrian extension requires explicit attitude opt-in')
    runtime = pins.collect_runtime(entrypoints=(Path(__file__).name,))
    map_paths = dict(source_map_link_sha256=candidate.gen.geo.DEFAULT_MGEO/'link_set.json',
        source_map_boundary_sha256=candidate.gen.geo.DEFAULT_MGEO/'lane_boundary_set.json')
    return dict(schema=SCHEMA,policy=POLICY,enabled=enabled,allow_static_pedestrians=allow_pedestrians,
        setup_profile=setup_profile,manifest_sha256=sha256(manifest),source_sha256=sha256(SOURCE),
        adapter_sha256=sha256(__file__),candidate_sha256=sha256(candidate.__file__),
        runtime_hashes=runtime,map_source_sha256={key:sha256(path) for key,path in map_paths.items()},
        map_source_files={key:str(path.resolve()) for key,path in map_paths.items()},
        qa_scope='Only static_ground_severe_below supported by full-RPY competing-level evidence; all other QA retained',
        bulk_qualified=False)


def validate_contract(contract):
    try:
        require(contract.get('schema') == SCHEMA and contract.get('policy') == POLICY
            and contract.get('source_sha256') == sha256(SOURCE)
            and contract.get('adapter_sha256') == sha256(__file__)
            and contract.get('candidate_sha256') == sha256(candidate.__file__), 'Runtime attitude contract changed')
        pins.verify_hashes(contract['runtime_hashes'])
        require(all(sha256(path) == contract['map_source_sha256'][key]
            for key,path in contract['map_source_files'].items()), 'Pinned MGeo bytes changed')
    except (OSError,ValueError,KeyError,TypeError) as error:
        # 원형 collector는 RuntimeError를 즉시 전체 중단한다. 계약 손상은 부족분이 아니다.
        raise RuntimeError('Static attitude runtime integrity failure: '+str(error)) from error


def adjudicated_audit(original_audit, observation, contract):
    """원본 audit는 불변이다. 다른 QA reason·component·verified는 손대지 않는다."""
    result = copy.deepcopy(original_audit)
    if not contract['enabled'] or not observation.get('classification_changed'):
        return result
    require(observation.get('attitude_qualified') is True, 'Changed decision requires verified attitude relationship')
    original_reasons = original_audit.get('review_reasons', [])
    expected = [reason for reason in original_reasons if reason != 'static_ground_severe_below']
    require('static_ground_severe_below' in original_reasons
        and observation.get('original_review_reasons') == original_reasons
        and observation.get('candidate_review_reasons') == expected,
        'Candidate may remove only the supported static below-plane classification')
    ground = original_audit.get('ground_support') or {}
    below = [row['uid'] for row in ground.get('actors', [])
        if row.get('reason') == 'actor_annotation_deep_below_ground'
        and row.get('minimum_annotation_ground_gap_m',0) < -.40]
    eligible = {row['uid'] for row in observation.get('actors', []) if row.get('eligible') is True}
    require(below and set(below) <= eligible, 'Every severe-below actor must have independent candidate evidence')
    require(contract['allow_static_pedestrians'] or not any(row.get('pedestrian_extension_opt_in')
        for row in observation.get('actors', [])), 'Pedestrian scope cannot be enabled by a decision file')
    result['review_reasons'] = expected
    result['accepted_candidate'] = not expected
    result['static_surface_attitude_adjudication'] = dict(policy=POLICY,
        original_review_reasons=original_reasons,retained_review_reasons=expected,
        reclassified_reason='static_ground_severe_below',affected_uids=sorted(set(below)),
        replacement='actor_ground_surface_association_unverified',
        original_audit_json_sha256=json_digest(original_audit),decision_json_sha256=json_digest(observation),
        runtime_contract_json_sha256=json_digest(contract),ground_contact_verified=False,
        original_planes_gaps_and_other_qa_preserved=True)
    return result


def final_audit_hook(root, metadata, base_audit, contract):
    if not contract['enabled']:
        return base_audit
    root = Path(root).resolve(strict=True)
    validate_contract(contract)
    require(not (root/'audit.json').exists() and all(not (root/name).exists() for name in (BASE_AUDIT,DECISION,RECEIPT)),
            'Adapter only runs before the first final audit of a new capture')
    require(read(root/'metadata.json') == metadata and (root/'telemetry.json').is_file(),
            'Final metadata and telemetry must exist before attitude adjudication')
    # collector는 이 hook 뒤 attempt.json만 capture_requested→complete로 바꾼다.
    # 원문/native/meta/telemetry는 고정하되 운영 상태 파일을 원본 GT로 오인하지 않는다.
    paths = [path for path in root.rglob('*') if path.is_file() and path.name != 'attempt.json']
    before = candidate.pins(paths)
    observation = candidate.analyze(root,allow_pedestrians=contract['allow_static_pedestrians'],
        map_hashes=contract['map_source_sha256'],original_audit=base_audit)
    result = adjudicated_audit(base_audit,observation,contract)
    require(before == candidate.pins(paths), 'Native source changed during attitude hook')
    atomic_json(root/BASE_AUDIT,copy.deepcopy(base_audit))
    atomic_json(root/DECISION,observation)
    receipt = dict(schema=SCHEMA,policy=POLICY,contract=contract,
        base_audit_sha256=sha256(root/BASE_AUDIT),decision_sha256=sha256(root/DECISION),
        final_audit_json_sha256=json_digest(result),source_files=before,
        classification_changed=observation.get('classification_changed') is True,
        original_native_inputs_preserved=True,ground_contact_certified=False,
        bulk_qualified=False)
    atomic_json(root/RECEIPT,receipt)
    return result


def verify_capture(root, contract):
    """새 세션의 재개·release 검증용. 과거 출력을 승격하지 않는다."""
    root = Path(root).resolve(strict=True);validate_contract(contract)
    receipt = read(root/RECEIPT)
    require(receipt.get('contract') == contract and receipt.get('policy') == POLICY,
            'Saved attitude runtime contract differs')
    require(sha256(root/BASE_AUDIT) == receipt['base_audit_sha256']
            and sha256(root/DECISION) == receipt['decision_sha256'], 'Saved original audit/decision changed')
    source_paths = [Path(path) for path in receipt['source_files']]
    require(candidate.pins(source_paths) == receipt['source_files'], 'Saved native input bytes changed')
    base = read(root/BASE_AUDIT)
    recomputed = candidate.analyze(root,allow_pedestrians=contract['allow_static_pedestrians'],
        map_hashes=contract['map_source_sha256'],original_audit=base)
    require(recomputed == read(root/DECISION), 'Saved attitude decision differs from recomputation')
    expected = adjudicated_audit(base,recomputed,contract)
    require(read(root/'audit.json') == expected and json_digest(expected) == receipt['final_audit_json_sha256'],
            'Final audit changed or removed unrelated QA')
    return dict(verified=True,classification_changed=receipt['classification_changed'],raw_modified=False,
                prior_audit_modified=False,ground_contact_certified=False)


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--enable-static-surface-attitude',action='store_true')
    parser.add_argument('--allow-static-pedestrians',action='store_true')
    parser.add_argument('--setup-profile',choices=['baseline','static-ready'],default='static-ready')
    options,rest = parser.parse_known_args()
    require('--approve-bulk-capture' not in rest, 'Prototype adapter has no 5K bulk qualification')
    paths = argparse.ArgumentParser(add_help=False)
    paths.add_argument('--manifest',type=Path,required=True);paths.add_argument('--output',type=Path,required=True)
    known,_ = paths.parse_known_args(rest)
    contract = configuration(known.manifest,options.enable_static_surface_attitude,
        options.allow_static_pedestrians,options.setup_profile)
    output = known.output.resolve();guard = output/'static_surface_attitude_runtime_contract.json'
    protected = known.manifest.resolve().parent
    require(output != protected and protected not in output.parents, 'New output must be outside frozen source')
    if output.exists():
        require(guard.is_file() or not any(output.iterdir()), 'Cannot attach attitude adapter to legacy output')
        if guard.is_file():require(read(guard) == contract, 'Attitude resume contract mismatch')
    else:output.mkdir(parents=True)
    if not guard.exists():atomic_json(guard,contract)
    validate_contract(contract)
    if contract['enabled'] and (output/'ledger.jsonl').is_file():
        for line in (output/'ledger.jsonl').read_text().splitlines():
            row=json.loads(line)
            if row.get('status') in ('accepted_candidate','review'):
                verify_capture(row['capture_root'],contract)
    namespace = dict(__file__=str(SOURCE),__name__='base5_static_attitude_capture',
        _static_fast_ready=static_fast_ready,_adapter_contract=contract,
        _surface_attitude_audit=lambda root,meta,audit:final_audit_hook(root,meta,audit,contract))
    exec(compiled_source(options.setup_profile,contract['source_sha256']),namespace)
    sys.argv = [str(SOURCE)]+rest
    namespace['main']()


if __name__ == '__main__':main()
