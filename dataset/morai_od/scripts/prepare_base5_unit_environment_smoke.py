#!/usr/bin/env python3
"""새 환경 원형 5K에서 native 소량 검증 대상을 관측 전에 등록한다.

CPU 조립 receipt와 전체 source를 검사하고 metadata만으로 정확히 32장을
고른다. 필요하면 사용자가 48장 이하의 target을 명시한다. ROS, MORAI,
수집기는 실행하지 않으며 미준비 음영 이동 장면은 승인하지 않는다.
"""
import argparse
from collections import Counter
import copy
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import time

import audit_base5_scenarios as scenario_audit
import base5_close_primary_observation as close
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_motion_ground as ground
import base5_shadow_ground_reuse as shadow
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from base5_intersection_ground import POLICY as EDITOR_POLICY
from base5_scale_capture_contract import validate_entry
import prepare_base5_unit_only_stability as raw_unit

SCHEMA = 'base5_unit_environment_smoke_registration_v1'
POLICY = 'independent-unit-environment-exact-smoke-v159'
BUILD_POLICY = 'independent-unit-environment-assembly-v159'
ENVIRONMENTS = ('sunny11', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
DEFAULT_TARGET = 32
MAX_FRAMES = 48
COMPOSITES = ('BabyStroller1', 'BabyStroller2', 'BabyStroller3', 'Ped_Pet1')
CROSSWALKS = ('CW000000', 'CW000002')
PHASES = ('approach', 'entry', 'alongside', 'clearing')


def read(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def motion_kind(meta):
    """새 음영 경로는 과거 exact terrain 승인과 결합하지 않는다."""
    require(not meta.get('calibration_only'), 'Calibration-only source is not eligible')
    require(shadow.KEY not in meta, 'New environment cannot inherit old shadow reuse')
    actors = meta['runtime_actors']
    require(all(a.get('motion') in ('static', 'standing', 'walking', 'running') for a in actors),
            'Unknown source actor motion')
    moving = ground.moving_actors(meta)
    if not moving:
        require(not meta['spec'].get('shadow_moving'), 'Moving shadow cannot masquerade as static')
        return 'static'
    if meta['spec']['family'] == 'shadow':
        require(meta['spec'].get('shadow_moving') is True
                and all(not a.get('motion_path', {}).get('ground_path_evidence') for a in moving),
                'Fresh shadow must retain explicit unprepared independent paths')
        return 'unprepared_shadow'
    require(all(a.get('motion_path', {}).get('ground_path_evidence', {}).get('policy') == EDITOR_POLICY
                for a in moving), 'Non-shadow motion requires editor-prepared crossing evidence')
    return 'editor_prepared'


def features(meta, effective_split, assets, config=None):
    spec = meta['spec']
    result = {'training_split/'+effective_split}
    if meta.get('training_split_overlay'):
        result.add('pipeline/layout_overlay')
    if meta.get(unit.KEY):
        result.add('pipeline/unit_reference')
    tag = spec.get(close.KEY, meta.get(close.KEY))
    if tag is not None:
        require(config is not None and close.KEY in config, 'Close-primary tag requires source policy')
        require(close.KEY not in spec or close.KEY not in meta or spec[close.KEY] == meta[close.KEY],
                'Ambiguous close-primary metadata/spec tag')
        row = dict(spec, **{close.KEY:tag})
        require(meta['runtime_actors'], 'Close-primary tag requires physical primary')
        primary = meta['runtime_actors'][0]
        close.validate(row, config, primary, primary.get('geometry_reference_known'))
        require(meta.get(unit.KEY) == tag['native_unit_reference'], 'Close-primary unit reference differs')
        result.add('pipeline/close_primary')
    if spec.get('static_mission_extent'):
        stratum = spec['static_mission_extent'].get('stratum')
        require(spec['family'] == 'obstacle' and spec.get('mission') is True
                and stratum in ('before','intersection','after') and spec.get('avoidance_phase') in PHASES,
                'Static mission extent contract differs')
        result.update(('extent_stratum/'+stratum, 'extent_phase/'+spec['avoidance_phase'],
                       'extent_training_split/'+effective_split))
    for actor in meta['runtime_actors']:
        name, distance = actor['name'], actor.get('planned_center_distance_m', -1)
        if name in assets:
            result.add('physical_asset/'+name)
            if 0 <= distance < 20:result.add('obstacle_range/0_20m')
            if 20 <= distance < 30:result.add('obstacle_range/20_30m')
        if name in COMPOSITES:result.add('compound/'+name)
        if (spec['family'] != 'shadow' and actor.get('mission_crossing_id') in CROSSWALKS
                and actor['motion'] in ('walking','running')):
            result.add('moving_crosswalk/'+actor['mission_crossing_id'])
        if spec['family'] == 'highway' and actor['source_list'] == 'vehicleList':
            for lo,hi in ((50,80),(80,100)):
                if lo <= distance < hi and actor.get('actual_ego_role') in ('front','rear'):
                    result.add(f'far/{lo}_{hi}m/'+actor['actual_ego_role'])
    if spec['family'] == 'shadow' and motion_kind(meta) == 'static':
        stage = spec.get('shadow_observation_stage')
        if stage in ('approach_far', 'approach_near', 'approach_threshold'):
            result.add('static_shadow/approach')
        if stage == 'interior':
            result.add('static_shadow/interior')
    return result


def order_key(label, ident):
    return hashlib.sha256((POLICY+':'+label+':'+ident).encode()).hexdigest(), ident


def select(deck, metadata, overlay, assets, target=DEFAULT_TARGET, maximum=MAX_FRAMES):
    require(type(target) is int and type(maximum) is int and 32 <= target <= maximum <= 48,
            'Explicit 32..48 target and maximum required; no automatic expansion')
    entries = {entry['id']:entry for entry in deck['scenes']}
    require(len(entries) == len(deck['scenes']) and set(entries) == set(metadata),
            'Complete unique source metadata required')
    require(len(assets) == 19, 'All 19 physical unit assets are required')
    classified = {ident:motion_kind(meta) for ident,meta in metadata.items()}
    eligible = {ident for ident,kind in classified.items() if kind != 'unprepared_shadow'}
    resolved = {ident:layout.resolve(overlay, ident, entries[ident], meta)
                for ident,meta in metadata.items()}
    config = deck.get('config', {})
    if close.KEY in config:
        require(config[close.KEY] == close.configuration(config.get(unit.KEY)), 'Source close-primary policy differs')
    covers = {ident:features(metadata[ident], resolved[ident]['split'], assets, config) for ident in eligible}
    required = {'physical_asset/'+name for name in assets} | {'compound/'+name for name in COMPOSITES}
    required.update('extent_stratum/'+v for v in ('before', 'intersection', 'after'))
    required.update('extent_phase/'+v for v in PHASES)
    required.update(('extent_training_split/train', 'extent_training_split/val',
        'training_split/train', 'training_split/val', 'pipeline/layout_overlay',
        'pipeline/unit_reference', 'obstacle_range/0_20m', 'obstacle_range/20_30m',
        'static_shadow/approach', 'static_shadow/interior'))
    controls = ['moving_crosswalk/'+crossing for crossing in CROSSWALKS]
    controls += [f'far/{lo}_{hi}m/{role}' for lo,hi in ((50,80),(80,100)) for role in ('front','rear')]
    required.update(controls)
    available = set().union(*covers.values()) if covers else set()
    if close.KEY in config and 'pipeline/close_primary' in available:
        required.add('pipeline/close_primary')
    require(required <= available, 'Prepared source lacks coverage: '+str(sorted(required-available)))
    frequency = Counter(t for tokens in covers.values() for t in tokens)
    selected, labels, groups = [], {}, set()

    def add(ident, label):
        require(ident in eligible and ident not in labels, 'Missing or duplicate eligible control')
        group_id = resolved[ident]['group_id']
        require(group_id not in groups, 'Native smoke must use independent layout groups')
        labels[ident] = [label];selected.append(ident);groups.add(group_id)

    def choices(token=None):
        return [ident for ident in eligible if ident not in labels
            and resolved[ident]['group_id'] not in groups and (token is None or token in covers[ident])]

    for token in controls:
        candidates = choices(token)
        require(candidates, 'Independent protected control missing: '+token)
        add(min(candidates, key=lambda ident:order_key(token, ident)), 'protected/'+token)
    moving_ids = [ident for ident in selected if classified[ident] == 'editor_prepared']
    while len(moving_ids) < 3:
        candidates = [ident for ident in choices() if classified[ident] == 'editor_prepared'
            and any(token.startswith('moving_crosswalk/') for token in covers[ident])]
        require(candidates, 'Three independent editor-prepared crossing scenes required')
        ident = min(candidates, key=lambda value:order_key('third_crossing', value))
        add(ident, 'protected/third_editor_crossing');moving_ids.append(ident)
    protected_count = len(selected)

    def missing(ids):
        return required-set().union(*(covers[ident] for ident in ids))

    uncovered = missing(selected)
    while uncovered:
        candidates = [ident for ident in choices() if covers[ident]&uncovered]
        require(candidates and len(selected) < target,
                'Explicit target cannot cover required metadata: '+str(sorted(uncovered)))
        ident = min(candidates, key=lambda value:(-len(covers[value]&uncovered),
            -sum((Fraction(1, frequency[t]) for t in covers[value]&uncovered), Fraction()),
            order_key('cover', value)))
        add(ident, 'metadata_set_cover');uncovered = missing(selected)
    for ident in selected[protected_count:][::-1]:
        remaining = [value for value in selected if value != ident]
        if not missing(remaining):
            selected.remove(ident);del labels[ident];groups.remove(resolved[ident]['group_id'])
    set_cover_frames = len(selected)
    for ident in sorted(choices(), key=lambda value:order_key('fill', value)):
        if len(selected) == target:
            break
        if resolved[ident]['group_id'] not in groups:
            add(ident, 'fixed_target_extra')
    require(len(selected) == target, 'Insufficient independent prepared layouts for fixed target')
    rows = [dict(id=ident, scenario_sha256=entries[ident]['scenario_sha256'],
        metadata_sha256=sha256(entries[ident]['metadata']), selection_roles=labels[ident],
        coverage=sorted(covers[ident]), effective_training_split=resolved[ident]['split'],
        layout_group_id=resolved[ident]['group_id'], source_entry_preserved=True) for ident in selected]
    inventory = dict(source_scenes=len(metadata), scene_counts=dict(Counter(classified.values())),
        eligible_count=len(eligible), excluded_ids=sorted(set(metadata)-eligible),
        excluded_count=len(metadata)-len(eligible), excluded_reason='Fresh shadow paths need new on-demand preparation',
        old_shadow_reuse_adopted=False, source_quota_modified=False, native_outcomes_used_for_selection=False)
    return rows, dict(required=sorted(required), covered=sorted(set().union(*(covers[i] for i in selected))),
        frames=len(rows), target_frames=target, maximum_frames=maximum, protected_frames=protected_count,
        set_cover_frames=set_cover_frames, editor_crossing_scenes=len(moving_ids),
        coverage_passed=True, native_outcomes_used_for_selection=False, eligibility=inventory)


def inspect_source(manifest, build_receipt):
    manifest, build_receipt = Path(manifest).resolve(strict=True), Path(build_receipt).resolve(strict=True)
    deck, receipt = read(manifest), read(build_receipt)
    env = deck['config'].get('environment_tag')
    require(env in ENVIRONMENTS and deck.get('count') == len(deck.get('scenes', [])) == 5000,
            'Complete fresh non-Sunny13 environment 5K required')
    require(receipt.get('policy') == BUILD_POLICY and receipt.get('environment') == env
            and receipt.get('final_manifest') == str(manifest)
            and receipt.get('final_manifest_sha256') == sha256(manifest)
            and receipt.get('status') == 'CPU_ONLY_PASS; native environment qualification required'
            and receipt.get('source_and_baseline_preserved') is True,
            'Matching complete CPU assembly receipt required')
    require(deck.get('bulk_ready') is False and deck.get('native_qualified') is False
            and not receipt.get('native_qualified') and not receipt.get('bulk_started'),
            'Source must not claim native or bulk qualification')
    environment = deck['config'].get('collection_environment', {})
    require(environment.get('weather') == env[:-2] and environment.get('hour') == int(env[-2:]),
            'Physical collection declaration must match fresh source environment')
    require('asset_scale_policy' not in deck['config'] and shadow.KEY not in deck['config']
            and deck.get('split_overlay') and deck['config'].get(unit.KEY),
            'Fresh layout/unit policy required; no old scale or shadow reuse policy')
    pins.verify_hashes(receipt['original_input_hashes']);pins.verify_hashes(receipt['runtime_hashes'])
    layout_audit = layout.validate_manifest_overlay(deck)
    overlay = layout.read_overlay(deck['split_overlay']['path'])
    policy = unit.load(deck['config'][unit.KEY])
    catalog = {a['data_id']:a for a in read(policy['catalog_reference']['path'])['assets']}
    require(len(policy['assets']) == 19 and set(policy['assets']) ==
            {a['name'] for a in catalog.values() if a.get('semantic_class') == 'obstacle'},
            'All catalog physical unit assets must be covered')
    entries, metadata, scale_counts = {}, {}, Counter()
    for entry in deck['scenes']:
        ident = entry['id'];require(ident not in entries, 'Duplicate source ID')
        require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Source scenario SHA differs')
        scene, meta = read(entry['scenario']), read(entry['metadata'])
        require(entry['split'] == meta['split'] and meta.get(unit.KEY) == deck['config'][unit.KEY]
                and meta.get('fresh_environment_provenance', {}).get('environment') == env,
                'Capture split, native unit reference or fresh environment identity differs')
        raw_unit.reject_scale_metadata(meta);scale_counts.update(raw_unit.raw_unit_scales(scene))
        raw_unit.bind_actors(scene, meta, catalog);validate_entry(entry, deck)
        if close.KEY in meta['spec'] or close.KEY in meta.get('avoidance_observation', {}):
            close.validate_entry(entry, deck)
        for container in raw_unit.CONTAINERS:
            require([a['UNIQUEID'] for a in scene.get(container, [])] ==
                [a['unique_id'] for a in meta['runtime_actors'] if a['source_list'] == container],
                'Raw source actor order differs')
        if motion_kind(meta) == 'editor_prepared':
            ground.validate_motion_ground(meta, scene)
        entries[ident], metadata[ident] = entry, meta
    inputs = pins.collect_inputs(manifest)
    inputs.update(overlay.get('source_hashes', {}));inputs.update(overlay.get('finalization_source_hashes', {}))
    inputs[str(build_receipt)] = sha256(build_receipt)
    inputs[str(Path(deck['split_overlay']['path']).resolve())] = sha256(deck['split_overlay']['path'])
    pins.verify_hashes(inputs)
    return dict(deck=deck, entries=entries, metadata=metadata, overlay=overlay, unit_assets=set(policy['assets']),
        input_hashes=inputs, build_receipt=build_receipt, layout_audit=layout_audit,
        raw_scale_counts=dict(scale_counts), environment=env)


def prepare(manifest, build_receipt, output, target=DEFAULT_TARGET, maximum=MAX_FRAMES):
    manifest, output = Path(manifest).resolve(strict=True), Path(output).resolve()
    import generate_base5_independent as gen
    for protected in (manifest.parent, gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()):
        require(output != protected and protected not in output.parents,
                'Fresh output must be outside source and simulator runtime')
    require(not output.exists(), 'Fresh smoke output required')
    runtime = pins.collect_runtime(entrypoints=(Path(__file__).name,))
    context = inspect_source(manifest, build_receipt)
    rows, coverage = select(context['deck'], context['metadata'], context['overlay'],
        context['unit_assets'], target, maximum)
    audit = scenario_audit.audit(manifest)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
            and audit.get('scenes') == audit.get('unique_payloads') == 5000, 'Full source offline audit failed')
    candidate = copy.deepcopy(context['deck'])
    candidate.update(count=len(rows), scenes=[copy.deepcopy(context['entries'][r['id']]) for r in rows],
        status='PREREGISTERED_NEW_ENVIRONMENT_UNIT_SMOKE_ONLY', bulk_ready=False,
        native_qualified=False, bulk_started=False, training_started=False)
    # 긴 전수 검사 중 다른 실행이 output을 만들었으면 기존 등록을 덮어쓰지 않는다.
    output.mkdir(parents=True, exist_ok=False)
    atomic_json(output/'manifest.json', candidate);atomic_json(output/'source_audit.json', audit)
    doc = dict(schema=SCHEMA, policy=POLICY, helper_sha256=sha256(__file__),
        registered_at_unix=time.time(), native_observations_must_follow_registration=True,
        source_manifest=str(manifest), source_manifest_sha256=sha256(manifest), environment=context['environment'],
        build_receipt=dict(path=str(context['build_receipt']), sha256=sha256(context['build_receipt'])),
        subset_manifest_sha256=sha256(output/'manifest.json'), source_audit_sha256=sha256(output/'source_audit.json'),
        selected=rows, coverage=coverage, input_hashes=context['input_hashes'], runtime_hashes=runtime,
        raw_scale_counts=context['raw_scale_counts'], native_qa_unchanged=True,
        native_qualified=False, bulk_ready=False, bulk_started=False, training_started=False,
        completion_live_qualified=False, old_shadow_reuse_adopted=False, capture_executed=False,
        source_entry_policy='Exact prepared source entries; no reselection using native outcomes',
        limitation='No UI environment readback or live qualification; all fresh shadow motion remains on-demand.')
    atomic_json(output/'registration.json', doc)
    pins.verify_hashes(runtime);pins.verify_hashes(context['input_hashes'])
    return output/'manifest.json'


def registration_context(manifest, registration):
    """사전 등록 이후 source·runtime·선정·출력 변경을 허용하지 않는다."""
    manifest = Path(manifest).resolve(strict=True)
    registration = Path(registration).resolve(strict=True)
    doc = read(registration)
    require(doc.get('schema') == SCHEMA and doc.get('policy') == POLICY
            and doc.get('helper_sha256') == sha256(__file__), 'Current smoke registration required')
    require(doc.get('source_manifest') == str(manifest)
            and doc.get('source_manifest_sha256') == sha256(manifest), 'Registered full source changed')
    for name in ('native_qualified', 'bulk_ready', 'bulk_started', 'training_started',
                 'completion_live_qualified', 'old_shadow_reuse_adopted', 'capture_executed'):
        require(doc.get(name) is False, 'Registration cannot claim '+name)
    require(doc.get('native_qa_unchanged') is True
            and doc.get('native_observations_must_follow_registration') is True,
            'Unchanged native gates and preregistration required')
    stamp = doc.get('registered_at_unix')
    require(type(stamp) in (int,float) and math.isfinite(stamp) and 0 < stamp <= time.time(),
            'Valid registration-before-observation timestamp required')
    runtime = pins.collect_runtime(entrypoints=(Path(__file__).name,))
    require(doc.get('runtime_hashes') == runtime, 'Runtime changed after registration')
    pins.verify_hashes(runtime)
    receipt = doc['build_receipt']
    require(sha256(receipt['path']) == receipt['sha256'], 'Build receipt changed')
    context = inspect_source(manifest, receipt['path'])
    require(doc.get('environment') == context['environment']
            and doc.get('input_hashes') == context['input_hashes']
            and doc.get('raw_scale_counts') == context['raw_scale_counts'], 'Registered input contract changed')
    pins.verify_hashes(context['input_hashes'])
    rows, coverage = select(context['deck'], context['metadata'], context['overlay'], context['unit_assets'],
        doc['coverage']['target_frames'], doc['coverage']['maximum_frames'])
    require(rows == doc.get('selected') and coverage == doc.get('coverage'), 'Registered metadata selection changed')
    candidate = copy.deepcopy(context['deck'])
    candidate.update(count=len(rows), scenes=[copy.deepcopy(context['entries'][r['id']]) for r in rows],
        status='PREREGISTERED_NEW_ENVIRONMENT_UNIT_SMOKE_ONLY', bulk_ready=False,
        native_qualified=False, bulk_started=False, training_started=False)
    subset = registration.parent/'manifest.json'
    require(sha256(subset) == doc.get('subset_manifest_sha256') and read(subset) == candidate
            and sha256(registration.parent/'source_audit.json') == doc.get('source_audit_sha256'),
            'Registered subset or full source audit changed')
    context.update(registration=registration, doc=doc, subset_path=subset, subset=candidate,
                   selected={r['id']:r for r in rows}, runtime=runtime)
    return context


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--build-receipt', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--target-frames', type=int, default=DEFAULT_TARGET)
    parser.add_argument('--maximum-frames', type=int, default=MAX_FRAMES)
    args = parser.parse_args()
    print(prepare(args.manifest, args.build_receipt, args.output, args.target_frames, args.maximum_frames))
