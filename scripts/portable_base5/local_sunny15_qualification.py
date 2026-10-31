#!/usr/bin/env python3
"""Sunny15 v159의 새 객체 목록 보존 복구를 촬영 없이 검증한다.

원형 source/runtime/QA는 수정하지 않는다. --run은 새 CPU 근거 작성만 허용하며
ROS import, socket, subprocess와 새 output 밖 쓰기는 차단한다.
"""
import argparse
import copy
import importlib
import json
from pathlib import Path
import ssl  # socket 차단 전에 표준 SSLSocket 클래스를 정의한다.
import sys
import uuid  # 표준 플랫폼 초기화는 CPU 실행 가드를 설치하기 전에 끝낸다.

# Shapely의 libgeos 탐색은 ldconfig를 읽는다. 라이브러리 초기화 뒤에 엄격한
# CPU 가드를 설치하며, 그 안의 생성·검사에서는 subprocess를 허용하지 않는다.
import shapely.geometry

import recovery_qualification as shared
import route_compat as files
import route_qualification as isolated

PROJECT = Path(__file__).resolve().parents[2]
SCRIPTS = PROJECT/'dataset/morai_od/scripts'
FINAL = PROJECT/'artifacts/evaluation/base5_sunny15_261017/finalization_v1'
MANIFEST = FINAL/'assembled_source_v2/final_source/manifest.json'
RELEASE = FINAL/'release_v159_sunny15/release.json'
OUTPUT = FINAL/'local_population_recovery_v1'
MANIFEST_SHA = 'b320b67716b9df8b649b73635f1da40c633ecde70080dc82f04c720f872d0ebf'
RELEASE_SHA = '9098b8959eb1fc25b4f5ce47ba6e2d638a8b68c199cf8c34fdcdc278a83ec28d'
SCHEMA = 'base5_local_population_recovery_cpu_qualification_v1'
POLICY = 'sunny15-v159-independent-branch-set-before-outcomes-v1'
MANDATORY = ('sunny15_sunny13_sunny11_b5v2_00082',
             'sunny15_sunny13_sunny11_b5v2_00076',
             'sunny15_sunny13_sunny11_b5v2_03754',
             'sunny15_sunny13_sunny11_b5v2_04027')


def require(value, message):
    if not value:
        raise RuntimeError(message)


def select_cases(records):
    """새 source ID를 명시하고 결과와 무관한 분기 집합을 먼저 고정한다."""
    index = {row['entry']['id']:row for row in records}
    require(len(index) == len(records) and all(s in index for s in MANDATORY),
            'Actual local mandatory source cells are missing')
    required = set().union(*(set(row['tokens']) for row in records))
    selected = [index[s] for s in MANDATORY]
    covered = set().union(*(set(row['tokens']) for row in selected))
    remaining = [row for row in records if row not in selected]
    while required-covered and remaining and len(selected) < shared.MAXIMUM_CASES:
        best = min(remaining, key=lambda row:(-len(set(row['tokens'])-covered),
            files.digest([POLICY,row['entry']['id']])))
        require(set(best['tokens'])-covered, 'Local source has an uncovered branch')
        selected.append(best);remaining.remove(best);covered.update(best['tokens'])
    return selected, dict(policy=POLICY, maximum_cases=shared.MAXIMUM_CASES,
        mandatory_ids=list(MANDATORY), required=sorted(required), covered=sorted(covered),
        missing=sorted(required-covered), all_branches_selected=required == covered,
        selection_before_outcomes=True,
        coverage_scope='Independent actual branch axes; not the full Cartesian product',
        selected=[dict(id=r['entry']['id'],tokens=r['tokens']) for r in selected])


def source_contract():
    require(files.sha256(MANIFEST) == MANIFEST_SHA and files.sha256(RELEASE) == RELEASE_SHA,
            'Only the exact qualified Sunny15 v159 source/release is allowed')
    release = files.read(RELEASE)
    runtime = release['runtime_hashes']
    require(len(runtime) == 438, 'Original 438-file Sunny15 runtime is required')
    pins = dict(release['input_hashes'])
    for path, digest in runtime.items():
        require(path not in pins or pins[path] == digest, 'Conflicting immutable source pin')
        pins[path] = digest
    pins.update({str(MANIFEST):MANIFEST_SHA,str(RELEASE):RELEASE_SHA})
    shared.verify_pins(pins)
    return pins


def verify_replacement(deck, source, manifest, *, quality, layout, unit, output):
    """현재 부모와 기존 복구의 원래 뿌리를 구분해 원형 계보를 대조한다."""
    child = files.read(files.no_symlink(manifest))
    before,old = source['entry'],source['metadata']
    parent = quality.replacement_contract(deck,before,old)
    require(child.get('count') == len(child.get('scenes',[])) == 1,
            'One actual local CPU replacement is required')
    require(shared.exact(child['config'],deck['config']), 'Frozen generator config changed')
    entry = child['scenes'][0];new = files.read(entry['metadata'])
    require(Path(output).resolve() in Path(manifest).resolve().parents, 'Candidate escaped its case')
    for key in ('scenario','metadata'):
        require(Path(output).resolve() in files.no_symlink(entry[key]).resolve().parents,
                'Candidate payload escaped its new output')
    require(files.sha256(entry['scenario']) == entry['scenario_sha256'] != before['scenario_sha256'],
            'Replacement must be a fresh distinct payload')
    require(shared.exact(new['spec'],old['spec']) and shared.identity(new) == shared.identity(old),
            'Frozen spec, ordered actor identity, motion or scale changed')
    for key in ('family','mission','role','distance_bounds','subfamily','motion','split'):
        require(shared.exact(entry.get(key),before.get(key)), 'Frozen cell field changed: '+key)
    require(new['split'] == old['split'] == entry['split']
            and new.get(unit.KEY) == old.get(unit.KEY) == deck['config'].get(unit.KEY),
            'Capture split or unit changed')
    for key in ('scene_group_id','environment_split_contract','logical_quota_slot_id','fresh_environment_provenance'):
        require(shared.exact(new.get(key),old.get(key)), 'Source lineage changed: '+key)
    fields = shared.POPULATION_FIELDS
    require(shared.exact({k:old.get('population_budget',{}).get(k) for k in fields},
                         {k:new.get('population_budget',{}).get(k) for k in fields}),
            'Frozen population/visibility budget changed')
    unit.validate_entry(entry,child);layout.validate_manifest_overlay(child,allow_subset=True)
    frozen = quality.replacement_contract(child,entry,new)
    proof = new.get('quality_replacement_contract',{});fresh = new.get('fresh_completion_source',{})
    require(frozen.get('overlay') is True and frozen.get('root_entry') == parent['root_entry']
            and proof.get('root_entry') == parent['root_entry']
            and proof.get('root_metadata_sha256') == parent['root_metadata_sha256']
            and proof.get('ordinal') == parent['ordinal']
            and proof.get('immediate_parent_entry') == before
            and proof.get('immediate_parent_metadata_sha256') == files.sha256(before['metadata'])
            and proof.get('raw_gt_or_parent_modified') is False
            and fresh.get('source_cell_id') == parent['root_entry']['id']
            and fresh.get('source_scenario_sha256') == parent['root_entry']['scenario_sha256'],
            'Canonical root, actual immediate parent or ordinal changed')
    a = layout.resolve(layout.overlay_for_metadata(old),before['id'],before,old)
    b = layout.resolve(layout.overlay_for_metadata(new),entry['id'],entry,new)
    require((a['split'],a['group_id']) == (b['split'],b['group_id']), 'Effective split/group changed')
    return dict(passed=True,source_id=before['id'],candidate_id=entry['id'],
        candidate_manifest=str(manifest),candidate_manifest_sha256=files.sha256(manifest),
        actor_count=len(new['runtime_actors']),ordered_actor_roster=shared.identity(new),
        nested_parent=parent['root_entry'] != before,lineage_ordinal=parent['ordinal'],
        spec_preserved=True,roster_preserved=True,motion_preserved=True,unit_preserved=True,
        population_budget_preserved=True,split_group_preserved=True,canonical_parent_preserved=True,
        native_capture_executed=False,qa_changed=False)


def run(execute=False):
    """실제 촬영 경로가 아닌 CPU replacement만 호출한다."""
    sys.dont_write_bytecode = True
    require(not OUTPUT.exists(), 'Preserve old qualification; use only the fresh fixed output')
    with isolated.cpu_only():
        if str(SCRIPTS) not in sys.path:
            sys.path.insert(0,str(SCRIPTS))
        modules = {name:importlib.import_module(name) for name in (
            'run_base5_quality_loop','run_base5_capture_campaign','base5_spatial_diversity',
            'base5_layout_split','base5_unit_geometry_contract',
            'run_base5_capture_campaign_direction_reference','recovery_population')}
        quality,campaign,spatial,layout,unit,direction,population = modules.values()
        require(population.POLICY == shared.POPULATION_POLICY, 'Wrong population policy')
        pins = source_contract()
        deck = files.read(MANIFEST)
        records,inventory,sources = shared.source_inventory(deck,layout=layout)
        for row in records:
            row['tokens'] = sorted(set(row['tokens']) | {
                'lineage/nested' if row['metadata'].get('quality_replacement_contract') else 'lineage/original'})
        selected,matrix = select_cases(records)
        require(matrix['all_branches_selected'], 'Local source exceeds the fixed24 case bound')
        helper_paths = (Path(__file__).resolve(), Path(__file__).with_name('local_sunny15.py'),
            Path(population.__file__).resolve(), Path(shared.__file__).resolve(),
            Path(files.__file__).resolve(), Path(isolated.__file__).resolve())
        helpers = {str(files.no_symlink(p)):files.sha256(p) for p in helper_paths}
        recipe = dict(schema=SCHEMA,policy=POLICY,manifest=str(MANIFEST),manifest_sha256=MANIFEST_SHA,
            release=str(RELEASE),release_sha256=RELEASE_SHA,source_runtime_count=438,
            population_policy=population.POLICY,source_inventory=inventory,matrix=matrix,
            helper_hashes=helpers,source_hashes=sources,captures_requested=0,
            native_capture_executed=False,qa_changed=False,selection_before_outcomes=True)
        if not execute:
            return dict(plan_only=True,files_written=False,recipe=recipe,output=str(OUTPUT))
        OUTPUT.mkdir(exist_ok=False)
        reports = []
        with shared.output_only(OUTPUT):
            isolated.atomic_new(OUTPUT/'recipe.json',recipe)
            try:
                # 현재 spatial builder를 보호하고 direction AST에 복사된 실제
                # guarded replacement를 호출한다. native execute는 호출하지 않는다.
                with spatial.installed(), population.installed(quality_loop=quality) as state, \
                        direction.installed(dict(quality_source_sha256=files.sha256(quality.__file__))):
                    replacement = quality.execute.__globals__['replacement']
                    require(replacement is quality.replacement, 'Direction AST omitted the population guard')
                    for number,row in enumerate(selected):
                        folder = OUTPUT/('case_%02d_%s' % (number,row['entry']['id']))
                        folder.mkdir()
                        subset = folder/'source_manifest.json'
                        isolated.atomic_new(subset,campaign.subset_deck(deck,[copy.deepcopy(row['entry'])]))
                        try:
                            candidate = replacement(subset,folder/'replacement',
                                known_payloads={row['entry']['scenario_sha256']})
                            report = verify_replacement(deck,row,candidate,quality=quality,
                                layout=layout,unit=unit,output=folder)
                            report['outcome'] = 'candidate_bound'
                        except quality.PlacementProposalExhausted as error:
                            report = dict(passed=False,outcome='bounded_exhaustion',
                                source_id=row['entry']['id'],error_type=type(error).__name__,
                                error=str(error),native_capture_executed=False,qa_changed=False)
                        path = folder/'report.json';isolated.atomic_new(path,report)
                        reports.append(dict(source_id=row['entry']['id'],passed=report['passed'],
                            outcome=report['outcome'],path=str(path),sha256=files.sha256(path)))
                shared.verify_pins(pins);shared.verify_pins(sources);shared.verify_pins(helpers)
                result = dict(recipe,passed=all(r['passed'] for r in reports),actual_python=sys.version,
                    recipe_sha256=files.sha256(OUTPUT/'recipe.json'),replacements=reports,
                    replacement_count=sum(r['passed'] for r in reports),
                    bounded_exhaustions=sum(not r['passed'] for r in reports),unexpected_contract_stops=0,
                    problem_00082_candidate_pass=any(r['source_id'] == MANDATORY[0] and r['passed'] for r in reports),
                    population_guard=state,direction_ast_uses_guard=True,source_inputs_unchanged=True,
                    native_qa_unchanged=True,bulk_authorized=False,
                    limitation='CPU replacement and direction AST binding only; no new native capture, surface reference, collection or detector quality claim')
                result['passed'] = result['passed'] and result['problem_00082_candidate_pass']
                path = OUTPUT/'qualification.json';isolated.atomic_new(path,result)
                return path
            except BaseException as error:
                isolated.atomic_new(OUTPUT/'qualification_failed.json',dict(schema=SCHEMA,passed=False,
                    error_type=type(error).__name__,error=str(error),replacements=reports,
                    captures_requested=0,native_capture_executed=False,unexpected_contract_stops=1))
                raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true',help='Write CPU evidence only; never start capture')
    args = parser.parse_args(argv)
    result = run(args.run)
    if isinstance(result,Path):
        print(result)
    else:
        recipe = result['recipe']
        print(json.dumps(dict(plan_only=True,files_written=False,captures_requested=0,
            native_capture_executed=False,output=result['output'],source_scenes=5000,
            source_runtime_count=438,selected=len(recipe['matrix']['selected']),
            branches=len(recipe['matrix']['required']),all_branches_selected=recipe['matrix']['all_branches_selected']),
            ensure_ascii=False,indent=2))
    return 0 if not isinstance(result,Path) or files.read(result).get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
