#!/usr/bin/env python3
"""동결 v158a의 실제 재생성 분기를 사전 고정한 CPU 검증.

원형 route intent/release/427개 runtime/QA를 먼저 확인한다. 기본은 읽기 전용
계획이며 --run도 새 CPU 증거만 작성한다. 기존 수집·69장·ROS는 실행하지 않는다.
"""
import argparse
from collections import Counter
from contextlib import contextmanager
import copy
import importlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import route_compat as compat
import route_dispatch
import route_qualification as original

SCHEMA = 'base5_portable_recovery_cpu_qualification_v1'
POLICY = 'frozen5k-branch-set-cover-before-outcomes-v1'
POPULATION_POLICY = 'frozen-recovery-roster-preserving-v2'
MAXIMUM_CASES = 24
MANDATORY_IDS = ('sunny13_sunny11_b5v2_00082', 'sunny13_sunny11_b5v2_00076')
FAMILIES = {'highway', 'roundabout', 'other_vehicle', 'obstacle',
            'pedestrian_related', 'shadow', 'background'}
IDENTITY_FIELDS = ('data_id', 'unique_id', 'name', 'source_list', 'motion',
                   'population_pool', 'requested_motion', 'training_class')
POPULATION_FIELDS = ('mode', 'layout', 'actor_count', 'costs', 'total_cost',
                     'requested_bounds', 'required_visible_actors')
BRANCH_FLAGS = ('secondary_obstacle_support', 'secondary_obstacle_front_support',
                'primary_obstacle_range', 'close_primary_observation', 'spatial_sampling_plan',
                'vehicle_close_phase', 'roundabout_phase', 'dynamic_observation_phase')


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def identity(metadata):
    result = []
    for actor in metadata['runtime_actors']:
        scale = actor.get('asset_scale') or {}
        result.append([*[actor.get(key) for key in IDENTITY_FIELDS], scale.get('profile_id')])
    return result


def exact(left,right):
    return compat.digest(left) == compat.digest(right)


def branch_tokens(entry, metadata, effective_split=None):
    """결과·retry 성공 여부를 읽지 않는 고정 분기 서명."""
    spec = metadata['spec'];actors = metadata['runtime_actors']
    tokens = {'family/'+spec['family'], 'capture_split/'+metadata['split'],
              'effective_split/'+(effective_split or entry.get('derived_training_split') or metadata['split'])}
    tokens.add('population/'+str(spec.get('population_mode', 'legacy')))
    if actors:
        tokens.add('roster/frozen' if spec.get('frozen_population_data_ids') else 'roster/unfrozen')
    else:
        tokens.add('roster/empty')
        if spec['family'] == 'shadow':tokens.add('shadow/empty')
    if spec.get('population_rng_reference'):
        tokens.add('roster/legacy_rng_reference')
    if spec.get('vehicle_only_population'):
        tokens.add('population/vehicle_only')
    if len(actors) >= 7:
        tokens.add('population/near_actor_limit')
    if spec.get('population_layout'):
        tokens.add('layout/'+spec['population_layout'])
    if 'secondary_far_vehicle' not in spec:
        tokens.add('chooser_family/'+spec['family'])
        if spec.get('population_mode') in ('generic_regular','generic_dense'):
            tokens.add('generic_chooser_family/'+spec['family'])
            tokens.add('generic_chooser_mode/'+spec['population_mode'])
    role = spec.get('role')
    if role in ('front', 'rear', 'left', 'right'):
        tokens.add('role/'+('side' if role in ('left', 'right') else role))
    if spec.get('mission'):
        tokens.add('mission/situation')
    if spec.get('avoidance_origin') == 'generic_close':
        tokens.add('close/generic')
    for flag in BRANCH_FLAGS:
        if flag in spec:
            tokens.add('branch/'+flag)
    support = spec.get('secondary_obstacle_support') or {}
    if support:
        tokens.add('support/added' if support.get('new_actor_ordinals') else 'support/no_added')
        for name in ('protected_near_ordinals', 'protected_mid_ordinals'):
            if support.get(name):tokens.add('support/'+name)
    extent = spec.get('static_mission_extent')
    if extent:
        tokens.add('extent/'+extent['stratum'])
    far = spec.get('secondary_far_vehicle')
    if far:
        # This builder returns before population.choose; keep both signs explicit.
        target = far.get('target_ordinal')
        actor = actors[target] if type(target) is int and 0 <= target < len(actors) else {}
        direction = actor.get('actual_ego_role') or far.get('role') or spec.get('role')
        tokens.add('far/'+str(direction))
    if spec.get('dynamic_observation_phase'):
        tokens.add('dynamic/vehicle' if spec.get('dynamic_mission_pool') else 'dynamic/pedestrian')
    if spec['family'] == 'shadow' and actors:
        primary = actors[0]
        tokens.add('shadow/animal' if primary.get('population_pool') == 'animal'
                   else 'shadow/compound' if primary.get('population_pool') in ('pet_person','stroller','rider')
                   else 'shadow/person')
    return tokens


def source_inventory(deck, *, layout):
    """실제 5000개의 spec/ordered roster와 기본 budget을 읽어 감사한다."""
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000, 'Exact complete source5000 required')
    require(deck.get('split_overlay'), 'Frozen source training overlay required')
    overlay = layout.read_overlay(deck['split_overlay']['path'])
    pins, records, ids, payloads = {}, [], set(), set()
    for entry in deck['scenes']:
        require(entry['id'] not in ids, 'Repeated source canonical ID');ids.add(entry['id'])
        path = compat.no_symlink(entry['metadata']);metadata = compat.read(path)
        scenario = compat.no_symlink(entry['scenario'])
        digest = compat.sha256(scenario)
        require(digest == entry['scenario_sha256'] and digest not in payloads, 'Changed/repeated source payload')
        payloads.add(digest);pins[str(path)] = compat.sha256(path);pins[str(scenario)] = digest
        spec, actors = metadata.get('spec'), metadata.get('runtime_actors')
        require(isinstance(spec, dict) and isinstance(actors, list), 'Source spec/roster missing')
        require(spec.get('family') in FAMILIES and spec['family'] == entry.get('family'), 'Unknown/source family mismatch')
        require(metadata.get('split') == entry.get('split') and metadata['split'] in ('train','val'), 'Capture split changed')
        require(type(spec.get('actors')) is int and spec['actors'] == len(actors), 'Source actor count differs from frozen spec')
        require((bool(actors) or spec['family'] in ('background','shadow'))
                and (spec['family'] != 'background' or not actors),
                'Unexpected source empty/nonempty roster: '+entry['id'])
        require(len({a.get('unique_id') for a in actors}) == len(actors), 'Repeated source parent UID')
        require(all(type(a.get('data_id')) is int and type(a.get('unique_id')) is int
                    and all(isinstance(a.get(key),str) and a[key] for key in ('name','source_list','motion'))
                    for a in actors), 'Malformed ordered source actor identity')
        if actors and spec.get('primary_identity') is not None:
            require(actors[0]['data_id'] == spec['primary_identity'], 'Source primary identity changed')
        if spec.get('frozen_population_data_ids') is not None:
            require(spec['frozen_population_data_ids'] == [a['data_id'] for a in actors], 'Frozen roster IDs differ')
        if spec.get('vehicle_only_population'):
            require(all(a.get('population_pool') == 'vehicle' for a in actors), 'Nonvehicle in source vehicle-only roster')
        budget = metadata.get('population_budget') or {}
        if budget:
            charges = [a.get('population_cost') for a in actors]
            require(all(type(value) in (int,float) and math.isfinite(value) and value>0 for value in charges),
                    'Invalid source population charge')
            require(budget.get('actor_count') == len(actors) and budget.get('costs') == charges
                    and budget.get('total_cost') == sum(charges), 'Source budget/count accounting differs')
            if spec.get('population_mode') in ('generic_regular','generic_dense'):
                bounds = spec['population_budget_bounds']
                require(2 <= len(actors) <= spec['population_max_actors']
                        and bounds[0] <= sum(charges) <= bounds[1], 'Source generic budget/count outside contract')
        binding = layout.resolve(overlay,entry['id'],entry,metadata)
        require(binding['split'] == entry.get('derived_training_split')
                and binding['group_id'] == entry.get('derived_layout_group_id'), 'Source effective split/group changed')
        records.append(dict(entry=entry,metadata=metadata,
                            tokens=sorted(branch_tokens(entry,metadata,binding['split']))))
    require({r['metadata']['spec']['family'] for r in records} == FAMILIES, 'Full source family coverage changed')
    counts = Counter(token for row in records for token in row['tokens'])
    summary = dict(passed=True, source_scenes=5000, unique_ids=len(ids), unique_payloads=len(payloads),
        source_files=len(pins), ordered_rosters_valid=True, actor_counts_valid=True, budgets_valid=True,
        effective_split_groups_valid=True, branch_counts=dict(sorted(counts.items())),
        native_observations=0, limitation='Source contract audit; not observed visibility, GT or native approval')
    return records, summary, pins


def select_cases(records):
    """Fixed signatures and tie-break, never substitute by a successful outcome."""
    index = {row['entry']['id']:row for row in records}
    require(len(index) == len(records) and all(name in index for name in MANDATORY_IDS), 'Mandatory actual problem cells missing')
    required = set().union(*(set(row['tokens']) for row in records))
    selected = [index[name] for name in MANDATORY_IDS]
    covered = set().union(*(set(row['tokens']) for row in selected))
    remaining = [row for row in records if row not in selected]
    while required-covered and remaining and len(selected) < MAXIMUM_CASES:
        best = min(remaining, key=lambda row:(-len(set(row['tokens'])-covered),
                   compat.digest([POLICY,row['entry']['id']])))
        require(set(best['tokens'])-covered, 'Uncoverable source branch matrix')
        selected.append(best);remaining.remove(best);covered.update(best['tokens'])
    missing = sorted(required-covered)
    return selected, dict(policy=POLICY, maximum_cases=MAXIMUM_CASES,
        mandatory_ids=list(MANDATORY_IDS), required=sorted(required), covered=sorted(covered),
        missing=missing, all_branches_selected=not missing, selection_before_outcomes=True,
        coverage_scope='Each actual family/chooser and generic family plus independent mode/policy axes; not full Cartesian family-by-mode coverage',
        tie_break='sha256(policy,canonical source ID)',
        selected=[dict(id=row['entry']['id'],tokens=row['tokens']) for row in selected])


def verify_replacement(deck, source, manifest, *, quality, layout, unit, output):
    manifest = compat.no_symlink(manifest)
    require(Path(output).resolve() in manifest.resolve().parents, 'CPU candidate escaped its case output')
    child = compat.read(manifest);before,old = source['entry'],source['metadata']
    require(child.get('count') == len(child.get('scenes',[])) == 1, 'One actual replacement required')
    require(exact(child.get('config'),deck.get('config')), 'Candidate changed frozen generator config')
    entry = child['scenes'][0];new = compat.read(entry['metadata'])
    for key in ('scenario','metadata'):
        require(Path(output).resolve() in compat.no_symlink(entry[key]).resolve().parents, 'Candidate file escaped case output')
    require(compat.sha256(entry['scenario']) == entry['scenario_sha256'] != before['scenario_sha256'], 'Fresh different payload required')
    require(exact(new['spec'],old['spec']) and exact(identity(new),identity(old)), 'Frozen spec/ordered actor identity or motion changed')
    require(len(new['runtime_actors']) == len(old['runtime_actors']), 'Actor count changed')
    for key in ('family','mission','role','distance_bounds','subfamily','motion','split'):
        require(exact(entry.get(key),before.get(key)), 'Original cell field changed: '+key)
    require(new['split'] == old['split'] == entry['split']
            and new.get(unit.KEY) == old.get(unit.KEY) == deck['config'].get(unit.KEY), 'Unit/capture split changed')
    for key in ('scene_group_id','environment_split_contract','logical_quota_slot_id','fresh_environment_provenance'):
        require(exact(new.get(key),old.get(key)), 'Original metadata lineage changed: '+key)
    old_budget, new_budget = old.get('population_budget',{}),new.get('population_budget',{})
    require(exact({key:old_budget.get(key) for key in POPULATION_FIELDS},
            {key:new_budget.get(key) for key in POPULATION_FIELDS}), 'Population budget/visibility requirement changed')
    unit.validate_entry(entry,child);layout.validate_manifest_overlay(child,allow_subset=True)
    frozen = quality.replacement_contract(child,entry,new)
    require(frozen.get('overlay') is True and frozen.get('root_entry') == before, 'Canonical replacement root detached')
    proof = new.get('quality_replacement_contract',{})
    fresh = new.get('fresh_completion_source',{})
    require(proof.get('root_entry') == before and proof.get('immediate_parent_entry') == before
            and proof.get('immediate_parent_metadata_sha256') == compat.sha256(before['metadata'])
            and proof.get('raw_gt_or_parent_modified') is False
            and fresh.get('source_cell_id') == before['id']
            and fresh.get('source_scenario_sha256') == before['scenario_sha256'], 'Replacement parent lineage changed')
    a = layout.resolve(layout.overlay_for_metadata(old),before['id'],before,old)
    b = layout.resolve(layout.overlay_for_metadata(new),entry['id'],entry,new)
    require((a['split'],a['group_id']) == (b['split'],b['group_id']), 'Effective training split/group changed')
    return dict(passed=True,source_id=before['id'],candidate_id=entry['id'],
        candidate_manifest=str(manifest),candidate_manifest_sha256=compat.sha256(manifest),
        original_payload_sha256=before['scenario_sha256'],candidate_payload_sha256=entry['scenario_sha256'],
        ordered_actor_roster=identity(old), actor_count=len(old['runtime_actors']), empty_background=not old['runtime_actors'],
        tokens=source['tokens'],spec_preserved=True,roster_preserved=True,motion_preserved=True,
        population_budget_preserved=True,unit_preserved=True,split_group_preserved=True,
        canonical_parent_preserved=True,native_capture_executed=False,qa_changed=False)


def destination(document, output, allow_temporary=False):
    output = compat.no_symlink(output)
    fixed = Path(document['workspace'])/'work/recovery-qualification/v1'
    temporary = Path(tempfile.gettempdir()).resolve()
    temporary_ok = allow_temporary and output.parent == temporary and output.name.startswith('base5-recovery-qualification-')
    require(output == fixed or temporary_ok, 'Only fixed new workspace qualification/v1 or explicit fresh /tmp diagnostic output allowed')
    require(not output.exists(), 'Fresh CPU qualification output required')
    protected = [Path(document['manifest']).resolve().parent,Path(document['release']).resolve().parent,
                 Path(document['output']).resolve(),Path(document['witness']).resolve()]
    for root in protected:
        require(output != root and root not in output.parents and output not in root.parents, 'CPU output overlaps preserved input/raw')
    return output


@contextmanager
def output_only(output):
    """새 output 밖의 실제 쓰기와 프로세스/네트워크는 별도 guard도 거부한다."""
    active = [True];output = Path(output).resolve()
    def location(value, directory_fd=None):
        if isinstance(value,int):return Path(os.readlink('/proc/self/fd/'+str(value))).resolve()
        path = Path(os.fsdecode(value))
        if not path.is_absolute():
            path = (Path(os.readlink('/proc/self/fd/'+str(directory_fd))) if directory_fd is not None and directory_fd>=0 else Path.cwd())/path
        return path.resolve()
    def permitted(value, directory_fd=None):
        path = location(value,directory_fd)
        require(path == output or output in path.parents, 'CPU write outside new qualification output: '+str(path))
    def audit(event,args):
        if not active[0]:return
        if event == 'open' and args[2] & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND):permitted(args[0])
        elif event in ('os.mkdir','os.remove','os.rmdir','os.unlink'):
            fd = args[-1] if type(args[-1]) is int else None
            if event == 'os.mkdir' and location(args[0],fd).is_dir():return
            permitted(args[0],fd)
        elif event in ('os.rename','os.replace'):
            permitted(args[0],args[2] if len(args)>2 else None);permitted(args[1],args[3] if len(args)>3 else None)
        elif event in ('os.chmod','os.chown','os.utime','os.truncate'):permitted(args[0])
        elif event in ('os.symlink','os.link','subprocess.Popen','os.system','os.exec','os.fork','os.posix_spawn') or event.startswith(('socket.connect','socket.bind','socket.sendto','socket.getaddrinfo')):
            raise RuntimeError('CPU side effect forbidden: '+event)
    sys.addaudithook(audit)
    try:yield
    finally:active[0] = False


def verify_pins(pins):
    require(all(compat.sha256(path) == expected for path,expected in pins.items()), 'Immutable source/helper changed during CPU qualification')


def run(parent_intent, output, execute=False, *, allow_temporary=False):
    parent_intent = compat.no_symlink(parent_intent)
    with original.cpu_only():
        document = route_dispatch.validate_intent(parent_intent)
        output = destination(document,output,allow_temporary)
        modules = {name:importlib.import_module(name) for name in (
            'generate_base5_independent','base5_static_mission_extent','run_base5_quality_loop',
            'run_base5_capture_campaign','base5_spatial_diversity','base5_layout_split',
            'base5_unit_geometry_contract','recovery_population')}
        gen,extent,quality,campaign,spatial,layout,unit,population = [modules[name] for name in (
            'generate_base5_independent','base5_static_mission_extent','run_base5_quality_loop',
            'run_base5_capture_campaign','base5_spatial_diversity','base5_layout_split',
            'base5_unit_geometry_contract','recovery_population')]
        require(quality.gen is gen and spatial.gen is gen, 'Actual generator module identities differ')
        require(population.POLICY == POPULATION_POLICY, 'Explicit tested roster-preserving profile required')
        helper_paths = {Path(__file__).resolve(),Path(compat.__file__).resolve(),Path(route_dispatch.__file__).resolve(),
                        Path(original.__file__).resolve(),Path(population.__file__).resolve()}
        helpers = {str(compat.no_symlink(path)):compat.sha256(path) for path in helper_paths}
        source = Path(document['manifest']);deck = compat.read(source)
        records, inventory, sources = source_inventory(deck,layout=layout)
        sources.update({str(parent_intent):compat.sha256(parent_intent),str(source):compat.sha256(source),
                        str(Path(document['witness'])):document['witness_sha256'],str(Path(document['release'])):route_dispatch.RELEASE_SHA})
        selected,matrix = select_cases(records)
        require(matrix['all_branches_selected'], 'Source branch matrix exceeds the fixed24 bound')
        recipe = dict(schema=SCHEMA,policy=POLICY,parent_intent=str(parent_intent),
            parent_intent_sha256=compat.sha256(parent_intent),manifest_sha256=compat.CANONICAL_MANIFEST_SHA256,
            source_runtime_count=427,source_inventory=inventory,matrix=matrix,population_policy=population.POLICY,
            helper_sha256=compat.sha256(__file__),helper_hashes=helpers,
            source_hashes=sources,captures_requested=0,native_capture_executed=False,
            qa_changed=False,bulk_authorized=False,selection_before_outcomes=True)
        if not execute:
            return dict(schema=SCHEMA,passed=False,plan_only=True,files_written=False,
                        captures_requested=0,native_capture_executed=False,recipe=recipe,output=str(output))
        output.parent.mkdir(parents=True,exist_ok=True);output.mkdir(exist_ok=False)
        reports = [];guard_state = None
        with output_only(output):
            original.atomic_new(output/'recipe.json',recipe)
            try:
                options = dict(generator=gen,extent=extent,manifest=source,
                               expected_witness_sha256=document['witness_sha256'])
                with compat.installed(document['witness'],**options),spatial.installed(),population.installed(quality_loop=quality) as state:
                    require(state.get('policy') == POPULATION_POLICY and all(state.get(key) is True for key in
                        ('forced_roster','source_roster_preserved','transient_choose_row_only',
                         'original_choose_rng_and_pool_budget_checks','native_qa_unchanged')),
                        'Explicit original-chooser population profile evidence required')
                    guard_state = state
                    for number,row in enumerate(selected):
                        folder = output/('case_%02d_%s' % (number,row['entry']['id']));folder.mkdir()
                        source_manifest = folder/'source_manifest.json'
                        original.atomic_new(source_manifest,campaign.subset_deck(deck,[copy.deepcopy(row['entry'])]))
                        try:
                            candidate = quality.replacement(source_manifest,folder/'replacement',known_payloads={row['entry']['scenario_sha256']})
                            report = verify_replacement(deck,row,candidate,quality=quality,layout=layout,unit=unit,output=folder)
                            report['outcome'] = 'candidate_bound'
                        except quality.PlacementProposalExhausted as error:
                            report = dict(passed=False,outcome='bounded_exhaustion',source_id=row['entry']['id'],
                                error_type=type(error).__name__,error=str(error),tokens=row['tokens'],
                                native_capture_executed=False,qa_changed=False)
                        path = folder/'report.json';original.atomic_new(path,report)
                        reports.append(dict(source_id=row['entry']['id'],passed=report['passed'],outcome=report['outcome'],
                                            path=str(path),sha256=compat.sha256(path)))
                route_dispatch.validate_intent(parent_intent);verify_pins(sources);verify_pins(helpers)
                all_passed = all(row['passed'] for row in reports)
                result = dict(schema=SCHEMA,policy=POLICY,passed=all_passed,actual_python=sys.version,
                    parent_intent_sha256=compat.sha256(parent_intent),manifest_sha256=compat.CANONICAL_MANIFEST_SHA256,
                    population_policy=population.POLICY,
                    recipe_sha256=compat.sha256(output/'recipe.json'),
                    helper_sha256=compat.sha256(__file__),helper_hashes=helpers,
                    matrix=matrix,source_inventory=inventory,replacements=reports,replacement_count=sum(r['passed'] for r in reports),
                    bounded_exhaustions=sum(not r['passed'] for r in reports),unexpected_contract_stops=0,
                    problem_00082_candidate_pass=any(r['source_id']==MANDATORY_IDS[0] and r['passed'] for r in reports),
                    population_guard=guard_state,source_inputs_unchanged=True,native_qa_unchanged=True,
                    captures_requested=0,native_capture_executed=False,bulk_authorized=False,
                    limitation='CPU replacement contracts only; no native approval, imported69 reapproval, collection completion or detector quality claim')
                result['passed'] = result['passed'] and result['problem_00082_candidate_pass']
                path = output/'qualification.json'
                original.atomic_new(path,result)
                return path
            except BaseException as error:
                original.atomic_new(output/'qualification_failed.json',dict(schema=SCHEMA,policy=POLICY,passed=False,
                    error_type=type(error).__name__,error=str(error),replacements=reports,population_guard=guard_state,
                    unexpected_contract_stops=1,captures_requested=0,native_capture_executed=False,
                    qa_changed=False,bulk_authorized=False))
                raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent-intent',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--run',action='store_true')
    args = parser.parse_args(argv)
    result = run(args.parent_intent,args.output,args.run)
    print(str(result) if isinstance(result,Path) else json.dumps(result,ensure_ascii=False,indent=2))
    if isinstance(result,Path) and not compat.read(result).get('passed'):return 1
    return 0


if __name__ == '__main__':raise SystemExit(main())
