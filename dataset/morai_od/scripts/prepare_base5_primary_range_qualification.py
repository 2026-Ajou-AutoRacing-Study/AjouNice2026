#!/usr/bin/env python3
"""Register current-source regressions and all declared v152 range contracts.

Registration is CPU-only and precedes native outcomes. Existing v151 inputs,
raw evidence, releases and the frozen front allocation remain immutable.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path

import base5_obstacle_front_support as front
import base5_primary_obstacle_range as primary
from base5_capture_safety import atomic_json, sha256
from base5_motion_ground import moving_actors
import prepare_base5_front_support_qualification as front_qualification

SCHEMA = 'base5_primary_range_qualification_v152'
POLICY = 'all-declared-primary-range-and-front-regression-v152'


def read(path):
    return json.loads(Path(path).read_text())


def order(ident):
    return primary.digest([POLICY, ident]), ident


def cap_band(cap):
    if cap == 20:
        return '20'
    if cap in (30, 40):
        return '30plus'
    raise primary.RangeContractError('Unknown declared primary cap stratum')


def validate_source(deck):
    entries, metadata, _ = front_qualification.validate_source(deck)
    policy = deck['config'].get(primary.CONFIG_KEY, {})
    if (policy.get('policy') != primary.POLICY
            or policy.get('source_sha256') != primary.implementation_sha256()):
        raise primary.RangeContractError('Current primary-range config/helper required')
    planned = []
    for ident, meta in metadata.items():
        plan = primary.validate_plan(meta['spec'], deck['config'])
        if plan is None:
            continue
        if plan['logical_slot_id'] != ident or moving_actors(meta):
            raise primary.RangeContractError('Range source ID/motion differs from its frozen plan')
        primary.verify_preserved(meta, meta)
        planned.append(ident)
    front_count = sum(bool(meta['spec'].get(front.PLAN_KEY, {}).get('selected'))
                      for meta in metadata.values())
    if not planned or front_count != 338:
        raise primary.RangeContractError('Declared range cohort and original338 front allocation required')
    return entries, metadata, sorted(planned, key=order)


def select_strata(entries, metadata, planned):
    strata = []
    for has_front in (False, True):
        for band in ('20', '30plus'):
            options = [ident for ident in planned
                       if (front.PLAN_KEY in metadata[ident]['spec']) == has_front
                       and cap_band(metadata[ident]['spec'][primary.PLAN_KEY]['cap_m']) == band]
            if not options:
                raise primary.RangeContractError('Range recovery needs front absent/present x two cap bands')
            ident = min(options, key=order)
            meta, entry = metadata[ident], entries[ident]
            plan = meta['spec'][primary.PLAN_KEY]
            strata.append(dict(source_id=ident, has_front_plan=has_front, cap_band=band,
                cap_m=plan['cap_m'], data_id=plan['primary_data_id'], primary_name=plan['primary_name'],
                source_scenario_sha256=entry['scenario_sha256'],
                source_metadata_sha256=sha256(entry['metadata']), range_plan=copy.deepcopy(plan)))
    return strata


def prepare(manifest, previous_registration, output):
    manifest, previous_registration, output = map(Path, (manifest, previous_registration, output))
    if output.exists():
        raise FileExistsError('New v152 qualification output required')
    source_pins = {str(path.resolve()): sha256(path) for path in (manifest, previous_registration)}
    deck, previous = read(manifest), read(previous_registration)
    entries, metadata, planned = validate_source(deck)
    if previous.get('source_manifest_sha256') != deck['config'][primary.CONFIG_KEY].get('source_manifest_sha256'):
        raise primary.RangeContractError('Previous front registration belongs to another v151 source')
    static = [row['id'] for row in previous['static']]
    animated = [row['id'] for row in previous['animated']]
    if len(static) != 64 or len(animated) != 8 or len(set(static + animated)) != 72:
        raise primary.RangeContractError('Original static64 and moving8 identity cohort required')
    old = {row['id']: row for row in previous['static'] + previous['animated']}
    for ident in static + animated:
        if ident not in entries or bool(moving_actors(metadata[ident])) != (ident in animated):
            raise primary.RangeContractError('Regression source identity or actual motion changed')
        if not primary.has_plan(metadata[ident]) and entries[ident]['scenario_sha256'] != old[ident]['sha256']:
            raise primary.RangeContractError('Untargeted regression payload changed')
    strata = select_strata(entries, metadata, planned)
    inventory = [dict(id=entry['id'], scenario_sha256=entry['scenario_sha256'],
                      metadata_sha256=sha256(entry['metadata'])) for entry in deck['scenes']]
    compatible = copy.deepcopy(previous)
    compatible.update(source_manifest=str(manifest.resolve()), source_manifest_sha256=sha256(manifest),
        input_sha256=source_pins, front_helper_sha256=front.implementation_sha256(),
        frozen_specs_sha256=deck['config']['frozen_specs_sha256'], live_qualified=False, bulk_started=False,
        qualification_policy=POLICY, previous_registration_sha256=sha256(previous_registration))
    for name in ('static', 'animated'):
        compatible[name] = [dict(row, sha256=entries[row['id']]['scenario_sha256'],
            metadata_sha256=sha256(entries[row['id']]['metadata'])) for row in previous[name]]
    for name, ids in (('static64', static), ('animated8', animated), ('range_static', planned),
                      ('range_recovery4', [row['source_id'] for row in strata])):
        subset = copy.deepcopy(deck)
        subset.update(count=len(ids), scenes=[entries[ident] for ident in ids],
            status='BOUNDED_V152_CURRENT_SOURCE_QUALIFICATION_PENDING_NATIVE; not training or bulk')
        atomic_json(output / name / 'manifest.json', subset)
    atomic_json(output / 'front_registration.json', compatible)
    cohort = [dict(id=ident, scenario_sha256=entries[ident]['scenario_sha256'],
                   metadata_sha256=sha256(entries[ident]['metadata']),
                   range_plan=copy.deepcopy(metadata[ident]['spec'][primary.PLAN_KEY])) for ident in planned]
    doc = dict(schema=SCHEMA, policy=POLICY, source_manifest=str(manifest.resolve()),
        source_manifest_sha256=sha256(manifest), source_inventory_sha256=primary.digest(inventory),
        range_helper_sha256=primary.implementation_sha256(), front_helper_sha256=front.implementation_sha256(),
        input_sha256=source_pins, front_registration=dict(path=str((output / 'front_registration.json').resolve()),
            sha256=sha256(output / 'front_registration.json')),
        range_static=cohort, range_static_count=len(cohort), strata4=strata,
        front_allocation_selected=338,
        range_cap_counts=dict(Counter(str(row['range_plan']['cap_m']) for row in cohort)),
        range_front_overlap=sum(primary.has_plan(metadata[ident]) and front.PLAN_KEY in metadata[ident]['spec']
                                for ident in planned),
        manifests={name: dict(path=str((output / name / 'manifest.json').resolve()),
                             sha256=sha256(output / name / 'manifest.json'))
                   for name in ('static64', 'animated8', 'range_static', 'range_recovery4')},
        selection='Original front64+moving8 identities, all current range contracts, fixed source-ID hash four strata; no new outcomes used',
        actual_deficit_fabricated=False, live_qualified=False, training_data=False, bulk_started=False)
    for path, digest in source_pins.items():
        if sha256(path) != digest:
            raise primary.RangeContractError('Input changed while registering qualification')
    atomic_json(output / 'registration.json', doc)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'previous-registration', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.previous_registration, args.output))
