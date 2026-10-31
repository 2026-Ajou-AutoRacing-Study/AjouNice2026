#!/usr/bin/env python3
"""원형 v158a 보충 생성기의 route 호환성을 실제 CPU 후보로만 검증한다.

--run은 새 검증 파일 작성 승인일 뿐 촬영 권한이 아니다. ROS import, network,
subprocess를 차단하고 원형 q.replacement 및 원형 layout/unit 검사를 재사용한다.
"""
import argparse
from contextlib import contextmanager
import copy
import importlib
import importlib.abc
import json
import os
from pathlib import Path
import socket
import subprocess
import sys

import route_compat as compat
import route_dispatch


SCHEMA = 'base5_portable_route_cpu_qualification_v1'
POLICY = 'actual-problem-cell-plus-bounded-static-extent-replacements-v1'
PROBLEM_ID = 'sunny13_sunny11_b5v2_00076'
MAXIMUM_CASES = 7
ROS_MODULES = ('rospy','roslib','roslaunch','rosgraph','rosnode','rostopic','rosservice',
    'rosbag','rosbridge_library','rosbridge_server','genpy','morai_msgs',
    'sensor_msgs','geometry_msgs','std_msgs')


def require(value, message):
    if not value:
        raise RuntimeError(message)


def atomic_new(path, document):
    path = compat.no_symlink(path)
    require(not path.exists(), 'Fresh qualification evidence required: '+str(path))
    payload = (json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(payload);stream.flush();os.fsync(stream.fileno())


@contextmanager
def cpu_only():
    """이미 불러온 ROS도 거부한다. 이 실행기는 service/child를 시작할 수 없다."""
    def forbidden_module(name):
        return any(name == prefix or name.startswith(prefix+'.') for prefix in ROS_MODULES)
    require(not any(forbidden_module(name) for name in sys.modules), 'CPU qualification must not run inside a ROS process')
    class NoROS(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            if forbidden_module(fullname):
                raise RuntimeError('CPU qualification forbids ROS import: '+fullname)
            return None
    def denied(*args, **kwargs):
        raise RuntimeError('CPU qualification forbids network, subprocess and simulator execution')
    finder = NoROS();sys.meta_path.insert(0,finder)
    functions = [(subprocess,'Popen'),(socket,'socket'),(socket,'create_connection'),(os,'system')]
    originals = [(owner,name,getattr(owner,name)) for owner,name in functions]
    for owner,name,_ in originals:setattr(owner,name,denied)
    try:
        yield
    finally:
        for owner,name,original in originals:setattr(owner,name,original)
        sys.meta_path.remove(finder)


def output_path(document, path):
    output = compat.no_symlink(path)
    allowed = compat.no_symlink(Path(document['workspace'])/'work/route-compat-validation')
    require(output == allowed or allowed in output.parents,
            'CPU output must stay within work/route-compat-validation')
    require(not output.exists(), 'Fresh CPU qualification output required; old evidence is preserved')
    return output


def select_cases(deck, quality):
    """모든 actual metadata를 읽고 outcome과 무관한 최대7개 후보를 먼저 고정한다."""
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000, 'Frozen full 5K source required')
    records = {};eligible = []
    for entry in deck['scenes']:
        require(entry['id'] not in records, 'Repeated frozen source cell ID')
        metadata = compat.read(entry['metadata'])
        require(metadata.get('split') == entry.get('split'), 'Actual source capture split differs')
        plan = metadata.get('spec', {}).get('static_mission_extent')
        row = dict(entry=entry,metadata=metadata)
        records[entry['id']] = row
        if plan:
            require(plan.get('stratum') in ('before','intersection','after')
                and metadata['split'] in ('train','val'), 'Unknown original extent/split stratum')
            eligible.append(row)
    require(PROBLEM_ID in records and records[PROBLEM_ID] in eligible,
            'The actual stopped cell must belong to the frozen extent source')
    required = {('stratum',row['metadata']['spec']['static_mission_extent']['stratum']) for row in eligible}
    required |= {('split',row['metadata']['split']) for row in eligible}
    require(required == {('stratum',v) for v in ('before','intersection','after')} |
            {('split','train'),('split','val')}, 'Actual extent must cover all three strata and both splits')
    selected = [records[PROBLEM_ID]]
    def key(row):
        meta = row['metadata'];plan = meta['spec']['static_mission_extent']
        return plan['stratum'],meta['split']
    def order(row):return compat.digest([POLICY,row['entry']['id']])
    groups = {}
    for row in sorted(eligible,key=order):groups.setdefault(key(row),[]).append(row)
    # One of every actually present stratum/capture-split combination first.
    for group,values in sorted(groups.items()):
        if not any(key(row) == group for row in selected):
            selected.append(values[0])
    require(len(selected) <= MAXIMUM_CASES, 'Extent representative allocation exceeds seven cells')
    # Fill remaining slots with new ordered actor identities/avoidance phases.
    rest = [row for row in eligible if row not in selected]
    while rest and len(selected) < MAXIMUM_CASES:
        seen = {(row['metadata']['spec'].get('avoidance_phase'),
            tuple(a['data_id'] for a in row['metadata']['runtime_actors'])) for row in selected}
        rest.sort(key=lambda row:(
            (row['metadata']['spec'].get('avoidance_phase'),tuple(a['data_id'] for a in row['metadata']['runtime_actors'])) in seen,
            order(row)))
        selected.append(rest.pop(0))
    return selected


def identity(metadata):
    return [[a.get(key) for key in ('data_id','unique_id','name','source_list','motion')]
            for a in metadata['runtime_actors']]


def verify_replacement(original_deck, source, manifest, *, quality, layout, unit):
    before,old = source['entry'],source['metadata']
    deck = compat.read(manifest)
    require(deck.get('count') == len(deck.get('scenes', [])) == 1, 'One actual CPU replacement required')
    entry = deck['scenes'][0];new = compat.read(entry['metadata'])
    require(compat.sha256(entry['scenario']) == entry['scenario_sha256'] != before['scenario_sha256'],
            'CPU recovery must produce a different serialized scenario payload')
    require(new['spec'] == old['spec'] and identity(new) == identity(old) and identity(old),
            'Replacement changed original primary/spec/ordered actor roster or motion')
    for name in ('family','role','distance_bounds','subfamily','motion','split'):
        require(entry.get(name) == before.get(name), 'Replacement changed original cell field: '+name)
    require(new['split'] == old['split'] == entry['split']
        and new.get(unit.KEY) == old.get(unit.KEY) == original_deck['config'].get(unit.KEY),
        'Replacement changed native unit or capture split contract')
    unit.validate_entry(entry,deck)
    layout.validate_manifest_overlay(deck,allow_subset=True)
    frozen = quality.replacement_contract(deck,entry,new)
    require(frozen.get('overlay') is True and frozen['root_entry'] == before,
            'Replacement detached original canonical layout root')
    root = new.get('fresh_completion_source',{})
    proof = new.get('quality_replacement_contract',{})
    require(root.get('source_cell_id') == before['id']
        and root.get('source_scenario_sha256') == before['scenario_sha256']
        and proof.get('root_entry') == before
        and proof.get('immediate_parent_entry') == before
        and proof.get('immediate_parent_metadata_sha256') == compat.sha256(before['metadata'])
        and proof.get('raw_gt_or_parent_modified') is False,
        'Replacement canonical/immediate-parent lineage changed')
    old_overlay,new_overlay = layout.overlay_for_metadata(old),layout.overlay_for_metadata(new)
    old_binding = layout.resolve(old_overlay,before['id'],before,old)
    new_binding = layout.resolve(new_overlay,entry['id'],entry,new)
    require(old_binding['split'] == new_binding['split'] and old_binding['group_id'] == new_binding['group_id'],
            'Replacement changed canonical effective training split/group')
    return dict(passed=True,source_id=before['id'],candidate_id=entry['id'],
        original_payload_sha256=before['scenario_sha256'],candidate_payload_sha256=entry['scenario_sha256'],
        candidate_manifest=str(Path(manifest).resolve()),candidate_manifest_sha256=compat.sha256(manifest),
        candidate_metadata=str(Path(entry['metadata']).resolve()),candidate_metadata_sha256=compat.sha256(entry['metadata']),
        stratum=old['spec']['static_mission_extent']['stratum'],capture_split=old['split'],
        effective_split=old_binding['split'],ordered_actor_roster=identity(old),
        spec_preserved=True,primary_preserved=True,ordered_roster_preserved=True,motion_preserved=True,
        unit_preserved=True,layout_root_preserved=True,different_payload=True,
        native_capture_executed=False,qa_changed=False)


def run(intent, output, run=False):
    intent = compat.no_symlink(intent)
    with cpu_only():
        document = route_dispatch.validate_intent(intent)
        output = output_path(document,output)
        own = str(Path(__file__).absolute())
        require(document.get('files', {}).get(own) == compat.sha256(__file__), 'Qualification helper must be pinned by the intent')
        if not run:
            return dict(schema=SCHEMA,passed=False,plan_only=True,output=str(output),
                maximum_replacements=MAXIMUM_CASES,captures_requested=0,native_capture_executed=False,
                files_written=False,bulk_authorized=False)
        gen = importlib.import_module('generate_base5_independent')
        extent = importlib.import_module('base5_static_mission_extent')
        quality = importlib.import_module('run_base5_quality_loop')
        campaign = importlib.import_module('run_base5_capture_campaign')
        spatial = importlib.import_module('base5_spatial_diversity')
        layout = importlib.import_module('base5_layout_split')
        unit = importlib.import_module('base5_unit_geometry_contract')
        require(quality.gen is gen and spatial.gen is gen, 'CPU qualification generator module identity differs')
        manifest = Path(document['manifest']);deck = compat.read(manifest)
        selected = select_cases(deck,quality)
        options = dict(generator=gen,extent=extent,manifest=manifest,
            expected_witness_sha256=document['witness_sha256'])
        ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        observation = compat.verify_context(ctx,document['witness'],**options)
        compat.restore_context(ctx,document['witness'],**options)
        require(extent.route_sha256(ctx) == compat.CANONICAL_ROUTE_SHA256, 'Restored context does not pass original extent SHA')
        sources = {str(manifest):compat.sha256(manifest),str(intent):compat.sha256(intent)}
        for row in selected:
            entry = row['entry']
            require(compat.sha256(entry['scenario']) == entry['scenario_sha256'], 'Original selected payload changed')
            sources[str(Path(entry['scenario']))] = entry['scenario_sha256']
            sources[str(Path(entry['metadata']))] = compat.sha256(entry['metadata'])
        output.mkdir(parents=True,exist_ok=False)
        recipe = dict(schema=SCHEMA,policy=POLICY,helper_sha256=compat.sha256(__file__),
            intent=str(intent),intent_sha256=compat.sha256(intent),witness_sha256=document['witness_sha256'],
            manifest_sha256=compat.CANONICAL_MANIFEST_SHA256,source_hashes=sources,
            selected=[dict(id=row['entry']['id'],stratum=row['metadata']['spec']['static_mission_extent']['stratum'],
                capture_split=row['metadata']['split']) for row in selected],
            maximum_replacements=MAXIMUM_CASES,selection_before_outcomes=True,qa_changed=False,
            captures_requested=0,native_capture_executed=False,bulk_authorized=False)
        atomic_new(output/'recipe.json',recipe)
        atomic_new(output/'route_context.json',dict(observation,restored_route_sha256=extent.route_sha256(ctx),arc_restored=True))
        reports = []
        try:
            with compat.installed(document['witness'],**options),spatial.installed():
                for ordinal,row in enumerate(selected):
                    folder = output/('case_%02d_%s' % (ordinal,row['entry']['id']))
                    folder.mkdir()
                    source = folder/'source_manifest.json'
                    atomic_new(source,campaign.subset_deck(deck,[row['entry']]))
                    candidate = quality.replacement(source,folder/'replacement',
                        known_payloads={row['entry']['scenario_sha256']})
                    report = verify_replacement(deck,row,candidate,quality=quality,layout=layout,unit=unit)
                    report_path = folder/'report.json';atomic_new(report_path,report)
                    reports.append(dict(source_id=row['entry']['id'],path=str(report_path),sha256=compat.sha256(report_path)))
                    require(all(compat.sha256(path) == expected for path,expected in sources.items()), 'Qualification changed a frozen source')
            route_dispatch.validate_intent(intent)
            require(all(compat.sha256(path) == expected for path,expected in sources.items()), 'Frozen inputs changed after qualification')
            result = dict(schema=SCHEMA,policy=POLICY,passed=True,helper_sha256=compat.sha256(__file__),
                actual_python=sys.version,manifest_sha256=compat.CANONICAL_MANIFEST_SHA256,
                intent_sha256=compat.sha256(intent),recipe_sha256=compat.sha256(output/'recipe.json'),
                route_context_sha256=compat.sha256(output/'route_context.json'),route_observation=observation,
                restored_route_sha256=compat.CANONICAL_ROUTE_SHA256,replacement_count=len(reports),replacements=reports,
                maximum_replacements=MAXIMUM_CASES,captures_requested=0,native_capture_executed=False,
                source_inputs_unchanged=True,qa_changed=False,native_qa_unchanged=True,bulk_authorized=False,
                limitation='CPU replacement portability only; no new native observation, collection completion or detector quality claim.')
            path = output/'qualification.json';atomic_new(path,result)
            return path
        except BaseException as error:
            atomic_new(output/'qualification_failed.json',dict(schema=SCHEMA,policy=POLICY,passed=False,
                error_type=type(error).__name__,error=str(error),completed_replacements=reports,
                captures_requested=0,native_capture_executed=False,qa_changed=False))
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--intent',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--run',action='store_true',help='Permit new CPU qualification files only; never capture')
    args = parser.parse_args(argv)
    result = run(args.intent,args.output,args.run)
    print(str(result) if isinstance(result,Path) else json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
