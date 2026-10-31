#!/usr/bin/env python3
"""Preregister a bounded native regression from the exact new frozen 5K.

This reads no new capture outcomes and never edits a scenario or simulator.
Approved scales, extent cells and protected motion/far controls are selected
deterministically before capture. A missing coverage cell fails explicitly.
"""
import argparse
from collections import Counter
import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path

import audit_base5_scenarios
import base5_asset_scale as scale
import base5_front_release_inputs as pins
import base5_shadow_ground_reuse as reuse
import prepare_base5_front_support_qualification as qualification
from base5_capture_safety import atomic_json, sha256
from prepare_base5_far_vehicle_qualification import source_audit_pins


POLICY = 'qualified-scale-extent-protected-native-smoke-v155'
SCHEMA = 'base5_scale_extent_smoke_registration_v1'
MAX_FRAMES = 32
APPROVED_ASSETS = frozenset(('CargoBox', 'WoodBox', 'YellowBarrel',
    'PE_Drum', 'PE_Firewall_Orange', 'Steel_Barricade', 'Concrete_Barricade01'))
CROSSWALKS = ('CW000000', 'CW000002')
PHASES = ('approach', 'entry', 'alongside', 'clearing')
CONTEXTS = ('generic', 'static_mission', 'shadow', 'multi')


class SmokeContractError(RuntimeError):
    """Coverage/provenance errors cannot become ordinary capture deficits."""


def read(path):
    return json.loads(Path(path).read_text())


def order_key(label, ident):
    return hashlib.sha256((POLICY + ':' + label + ':' + ident).encode()).hexdigest(), ident


def approved_pairs(policy):
    pairs = {(name, ident) for name, asset in policy['assets'].items()
             for ident, profile in asset['profiles'].items() if scale._qualified(profile)}
    if ({name for name, _ in pairs} != APPROVED_ASSETS or len(pairs) != 14
            or any(sum(name == asset for name, _ in pairs) != 2 for asset in APPROVED_ASSETS)):
        raise SmokeContractError('Exactly the seven approved assets and two native-qualified profiles each required; RedBarrel variants are deferred')
    return pairs


def variants(meta, pairs):
    values = []
    for ordinal, actor in enumerate(meta.get('runtime_actors', [])):
        assignment = actor.get('asset_scale')
        if not assignment or assignment.get('profile_id') == scale.BASE:
            continue
        pair = (actor.get('name'), assignment.get('profile_id'))
        if (pair not in pairs or assignment.get('candidate_only') is not False
                or assignment.get('profile_status') != 'qualified'
                or assignment.get('name') != actor.get('name')
                or assignment.get('ordinal') != ordinal):
            raise SmokeContractError('Unregistered/candidate scale actor in source metadata')
        values.append(dict(name=pair[0], profile_id=pair[1], ordinal=ordinal,
                           role='primary' if ordinal == 0 else 'auxiliary'))
    return values


def features(meta, pairs):
    row, actors = meta['spec'], meta.get('runtime_actors', [])
    changed = variants(meta, pairs)
    result = {'profile/' + a['name'] + '/' + a['profile_id'] for a in changed}
    for actor in changed:
        result.add('scaled_role/' + actor['role'])
    if changed:
        if row.get('family') == 'shadow':
            result.add('scaled_context/shadow')
        elif row.get('family') == 'obstacle' and row.get('mission'):
            result.add('scaled_context/static_mission')
        elif not row.get('mission'):
            result.add('scaled_context/generic')
        if len(actors) > 1:
            result.add('scaled_context/multi')
    plan = row.get('static_mission_extent')
    if plan:
        if (row.get('family') != 'obstacle' or row.get('mission') is not True
                or plan.get('stratum') not in ('before', 'intersection', 'after')
                or row.get('avoidance_phase') not in PHASES or meta.get('split') not in ('train', 'val')):
            raise SmokeContractError('Malformed static mission extent cell')
        result.update(('extent_stratum/' + plan['stratum'],
                       'extent_phase/' + row['avoidance_phase'], 'extent_split/' + meta['split']))
    return result


def select(deck, metadata, pairs, exact_shadow_ids, unchanged_ids, maximum=MAX_FRAMES, target=None):
    """Deterministic greedy set-cover, with eight unchanged protected slots.

    Coverage is based solely on frozen actor/spec data, never native success.
    Available scale contexts/roles are mandatory; impossible contexts are
    reported explicitly, not fabricated. Profile and extent cells are strict.
    """
    if type(maximum) is not int or not 8 <= maximum <= MAX_FRAMES:
        raise SmokeContractError('Native smoke must have a finite 8..32 frame cap')
    if target is not None and (type(target) is not int or not 8 <= target <= maximum):
        raise SmokeContractError('Fixed native target must remain inside the declared frame cap')
    entries = {entry['id']: entry for entry in deck['scenes']}
    if len(entries) != len(deck['scenes']) or set(entries) != set(metadata):
        raise SmokeContractError('Duplicate or missing source metadata identity')
    if len(exact_shadow_ids) != 2 or len(set(exact_shadow_ids)) != 2:
        raise SmokeContractError('Exactly two registered exact-source shadow controls required')
    cover = {ident: features(meta, pairs) for ident, meta in metadata.items()}
    available = set().union(*cover.values()) if cover else set()
    required = {'profile/' + name + '/' + profile for name, profile in pairs}
    required.update('extent_stratum/' + value for value in ('before', 'intersection', 'after'))
    required.update('extent_phase/' + value for value in PHASES)
    required.update(('extent_split/train', 'extent_split/val'))
    optional = {'scaled_context/' + value for value in CONTEXTS}
    optional.update(('scaled_role/primary', 'scaled_role/auxiliary'))
    required.update(optional & available)
    missing = required - available
    if missing:
        raise SmokeContractError('Source lacks required smoke coverage: ' + ', '.join(sorted(missing)))
    selected, labels = [], {}

    def add(ident, label):
        if ident not in entries or ident in labels or ident not in unchanged_ids:
            raise SmokeContractError('Protected control missing, duplicated, or changed: ' + label)
        if variants(metadata[ident], pairs):
            raise SmokeContractError('Protected control unexpectedly contains a scale variant')
        labels[ident] = [label]
        selected.append(ident)

    for ident in sorted(exact_shadow_ids):
        meta = metadata.get(ident, {})
        if (meta.get('spec', {}).get('family') != 'shadow'
                or not meta['spec'].get('shadow_moving')
                or not any(a.get('motion') not in ('static', 'standing') for a in meta.get('runtime_actors', []))):
            raise SmokeContractError('Registered shadow control is not an actual moving source')
        add(ident, 'protected/shadow_exact/' + ident)

    for crosswalk in CROSSWALKS:
        label = 'protected/crosswalk/' + crosswalk
        choices = [ident for ident, meta in metadata.items() if ident in unchanged_ids
            and ident not in labels and meta['spec'].get('family') != 'shadow'
            and not variants(meta, pairs) and any(a.get('mission_crossing_id') == crosswalk
                and a.get('motion') in ('walking', 'running') for a in meta.get('runtime_actors', []))]
        if not choices:
            raise SmokeContractError('Missing unchanged moving crosswalk: ' + crosswalk)
        add(min(choices, key=lambda ident: order_key(label, ident)), label)

    for lower, upper in ((50, 80), (80, 100)):
        for role in ('front', 'rear'):
            label = f'protected/far/{lower}_{upper}m/{role}'
            choices = [ident for ident, meta in metadata.items() if ident in unchanged_ids
                and ident not in labels and meta['spec'].get('family') == 'highway'
                and not variants(meta, pairs) and any(a.get('source_list') == 'vehicleList'
                    and a.get('actual_ego_role') == role
                    and lower <= a.get('planned_center_distance_m', -1) < upper
                    for a in meta.get('runtime_actors', []))]
            if not choices:
                raise SmokeContractError('Missing unchanged far vehicle control: ' + label)
            add(min(choices, key=lambda ident: order_key(label, ident)), label)

    uncovered = required - set().union(*(cover[ident] for ident in selected))
    frequency = Counter(token for value in cover.values() for token in value)
    while uncovered:
        choices = [ident for ident in entries if ident not in labels and cover[ident] & uncovered]
        if not choices or len(selected) >= maximum:
            raise SmokeContractError('Finite smoke cap cannot cover: ' + ', '.join(sorted(uncovered)))
        ident = min(choices, key=lambda ident: (-len(cover[ident] & uncovered),
            -sum((Fraction(1, frequency[token]) for token in cover[ident] & uncovered), Fraction()),
            order_key('set-cover', ident)))
        labels[ident] = ['set_cover']
        selected.append(ident)
        uncovered -= cover[ident]
    # Discard redundant non-protected greedy choices without changing the
    # preregistered candidate pool or looking at live outcomes.
    for ident in selected[8:][::-1]:
        others = [value for value in selected if value != ident]
        if required <= set().union(*(cover[value] for value in others)):
            selected.remove(ident)
            del labels[ident]
    cover_frames = len(selected)
    if target is not None:
        if len(selected) > target:
            raise SmokeContractError('Required coverage exceeds the fixed native target')
        extras = sorted((ident for ident in entries if ident not in labels and variants(metadata[ident], pairs)),
                        key=lambda ident: order_key('fixed-native-fill', ident))
        if len(extras) < target - len(selected):
            raise SmokeContractError('Insufficient additional qualified-scale slots for fixed native target')
        for ident in extras[:target - len(selected)]:
            selected.append(ident)
            labels[ident] = ['fixed_native_count_extra']
    rows = [dict(id=ident, selection_roles=labels[ident], coverage=sorted(cover[ident]),
                 scaled_actors=variants(metadata[ident], pairs)) for ident in selected]
    return rows, dict(required=sorted(required), covered=sorted(set().union(*(cover[i] for i in selected))),
        unavailable_scale_context_or_roles=sorted(optional - available), protected_frames=8,
        maximum_frames=maximum, target_frames=target, frames=len(rows),
        extra_fixed_count_frames=len(rows)-cover_frames, coverage_passed=True,
        native_outcomes_used_for_selection=False)


def source_checks(manifest):
    """Support the explicit new scale audit schema without rewriting history."""
    report_path = manifest.parent / 'preservation_audit.json'
    if not report_path.exists():
        # The existing reuse registrar adds one config member to its source
        # and deliberately does not edit/relabel that source's full audits.
        # Prove this exact overlay rather than accepting a missing audit or
        # adding generic equivalence exceptions to frozen input readers.
        deck = read(manifest)
        configured = deck.get('config', {}).get(reuse.KEY, {})
        reference = configured.get('registration', {})
        registry_path = Path(reference.get('path', ''))
        if (configured.get('policy') != reuse.POLICY or not registry_path.is_absolute()
                or not registry_path.is_file() or sha256(registry_path) != reference.get('sha256')):
            raise SmokeContractError('Missing full audit is not an explicit shadow registry overlay')
        registry = read(registry_path)
        parent_ref = registry.get('source_manifest', {})
        parent_path = Path(parent_ref.get('path', ''))
        if (registry.get('policy') != reuse.POLICY or not parent_path.is_absolute()
                or parent_path.resolve() == manifest.resolve() or not parent_path.is_file()
                or sha256(parent_path) != parent_ref.get('sha256')):
            raise SmokeContractError('Shadow registry parent manifest pin differs')
        parent = read(parent_path)
        if (reuse.KEY in parent.get('config', {}) or parent.get('count') != 5000
                or len(parent.get('scenes', [])) != 5000
                or read(parent_path.parent / 'preservation_audit.json').get('schema')
                    != 'base5_asset_scale_preparation_v1'):
            raise SmokeContractError('Shadow overlay must derive directly from an audited scale full 5K')
        expected = copy.deepcopy(parent)
        expected['config'][reuse.KEY] = copy.deepcopy(configured)
        if deck != expected:
            raise SmokeContractError('Shadow overlay changed fields outside its one registered config member')
        result = source_checks(parent_path)
        result.update({str(registry_path.resolve()): sha256(registry_path),
                       str(parent_path.resolve()): sha256(parent_path)})
        return result
    report = read(report_path)
    if report.get('schema') != 'base5_asset_scale_preparation_v1':
        return source_audit_pins(manifest)
    expected = sha256(manifest)
    if (report.get('passed') is not True or report.get('errors') or report.get('frames') != 5000
            or report.get('manifest_sha256') != expected
            or ('candidate_manifest_sha256' in report and report['candidate_manifest_sha256'] != expected)):
        raise SmokeContractError('Full scale candidate preservation audit failed or stale')
    scenario_path = manifest.parent / 'scenario_audit.json'
    audit = read(scenario_path)
    if (audit.get('status') != 'OFFLINE_PASS' or audit.get('errors')
            or audit.get('scenes') != 5000 or audit.get('unique_payloads') != 5000):
        raise SmokeContractError('Full scale candidate scenario audit failed or incomplete')
    return {str(path): sha256(path) for path in (report_path, scenario_path)}


def prepare(manifest, output, maximum=MAX_FRAMES, target=17):
    manifest, output = Path(manifest).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh preregistration output required')
    import generate_base5_independent as gen
    runtime_dir = gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if (output == manifest.parent or manifest.parent in output.parents
            or output == runtime_dir or runtime_dir in output.parents):
        raise SmokeContractError('Offline smoke output cannot modify source or MORAI runtime')
    checks = source_checks(manifest)
    inputs = pins.collect_inputs(manifest)
    deck = read(manifest)
    entries, metadata, _ = qualification.validate_source(deck)
    config = scale.bind_config(copy.deepcopy(deck['config']))
    policy = config.get(scale.CONFIG_KEY)
    pairs = approved_pairs(policy)
    catalog = {asset['data_id']: asset for asset in read(gen.catalog.DEFAULT_CATALOG)['assets']}
    for ident, meta in metadata.items():
        scale.verify_metadata(read(entries[ident]['scenario']), meta, config, catalog)
    origin = deck.get('provenance', {}).get('asset_scale', {})
    original_path = Path(origin.get('source_manifest', '')).resolve()
    if not original_path.is_file() or sha256(original_path) != origin.get('source_manifest_sha256'):
        raise SmokeContractError('Scale candidate lacks its immutable original full manifest')
    original = read(original_path)
    originals = {entry['id']: entry for entry in original['scenes']}
    if original.get('count') != 5000 or len(originals) != 5000:
        raise SmokeContractError('Scale source ancestor is not a full unique 5K')
    inputs[str(original_path)] = sha256(original_path)
    unchanged = {ident for ident, entry in entries.items() if entry == originals.get(ident)}
    reuse_pins = reuse.validate_manifest(deck)
    inputs.update(reuse_pins)
    registry = read(deck['config'][reuse.KEY]['registration']['path'])
    shadows = [row['source_id'] for row in registry['entries'] if row['mode'] == 'exact_source']
    selected, coverage = select(deck, metadata, pairs, shadows, unchanged, maximum, target)
    actual_audit = audit_base5_scenarios.audit(manifest)
    if actual_audit.get('status') != 'OFFLINE_PASS' or actual_audit.get('errors'):
        raise SmokeContractError('Current full candidate scenario audit failed')
    runtime = pins.collect_runtime(entrypoints=pins.ENTRYPOINTS + (Path(__file__).name,))
    subset = copy.deepcopy(deck)
    subset.update(scenes=[copy.deepcopy(entries[row['id']]) for row in selected], count=len(selected),
        status='REGISTERED_SCALE_EXTENT_NATIVE_SMOKE_ONLY; no bulk or training')
    for row in selected:
        entry = entries[row['id']]
        row.update(scenario_sha256=entry['scenario_sha256'], metadata_sha256=sha256(entry['metadata']),
            source_entry_preserved=True, original_ancestor_entry_preserved=row['id'] in unchanged)
    for mapping in (inputs, checks, runtime):
        pins.verify_hashes(mapping)
    scale.verify_frozen_policy(policy)
    atomic_json(output / 'manifest.json', subset)
    atomic_json(output / 'source_audit.json', actual_audit)
    registration = dict(schema=SCHEMA, policy=POLICY, helper_sha256=sha256(__file__),
        source_manifest=str(manifest), source_manifest_sha256=inputs[str(manifest)],
        subset_manifest_sha256=sha256(output / 'manifest.json'), input_hashes=inputs,
        source_audit_hashes=checks, runtime_hashes=runtime,
        policy_sha256=scale.digest(policy), qualified_profiles=[list(pair) for pair in sorted(pairs)],
        qualification_evidence_hashes={pin['path']: pin['sha256'] for name, profile in pairs
            for pin in policy['assets'][name]['profiles'][profile]['native_qualification']['evidence']},
        selected=selected, coverage=coverage, native_qa_unchanged=True,
        selection='Frozen metadata deterministic set-cover; no native outcomes or post-hoc success ranking',
        bulk_started=False, training_started=False, live_qualified=False)
    atomic_json(output / 'registration.json', registration)
    for mapping in (inputs, checks, runtime):
        pins.verify_hashes(mapping)
    scale.verify_frozen_policy(policy)
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maximum-frames', type=int, default=MAX_FRAMES)
    parser.add_argument('--target-frames', type=int, default=17)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output, args.maximum_frames, args.target_frames))
