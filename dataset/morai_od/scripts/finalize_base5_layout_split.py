#!/usr/bin/env python3
"""Unify selected recovery overlays without editing native metadata/ledgers."""
import argparse
from collections import Counter, defaultdict
import copy
import json
from pathlib import Path

import base5_layout_split as layout
from base5_capture_safety import atomic_json, sha256


def planned_fields_match(planned, actual):
    if isinstance(planned, dict):
        return isinstance(actual, dict) and all(key in actual and planned_fields_match(value, actual[key])
            for key,value in planned.items() if key != 'training_split_overlay')
    if isinstance(planned, list):
        return isinstance(actual,list) and len(planned)==len(actual) and all(
            planned_fields_match(a,b) for a,b in zip(planned,actual))
    return planned==actual


def finalize(source, adopted, selected, output):
    source, adopted, selected, output = map(lambda p: Path(p).resolve(), (source, adopted, selected, output))
    if output.exists():
        raise FileExistsError('Fresh finalized layout split output required')
    source_doc, adopted_doc = json.loads(source.read_text()), json.loads(adopted.read_text())
    layout.validate_manifest_overlay(source_doc)
    base_ref = source_doc['split_overlay']
    base = layout.read_overlay(base_ref['path'])
    layout.verify_source_pins(base)
    from base5_unit_geometry_contract import KEY as UNIT_KEY,load as load_unit
    unit_reference=source_doc['config'].get(UNIT_KEY)
    if unit_reference is not None:load_unit(unit_reference)
    source_entries = {row['id']: row for row in source_doc['scenes']}
    entries = {row['id']: row for row in adopted_doc['scenes']}
    rows = [json.loads(line) for line in selected.read_text().splitlines()]
    if (len(source_entries) != len(source_doc['scenes']) or len(entries) != len(adopted_doc['scenes'])
            or set(entries) != set(source_entries) or set(source_entries) != set(base['assignments'])
            or adopted_doc['config'] != source_doc['config']):
        raise ValueError('Finalization requires one complete adopted selection of the original quota IDs/config')
    lookup, seen_runtime, seen_roots = {}, set(), set()
    for row in rows:
        canonical = row.get('source_cell_id', row['id'])
        root = str(Path(row['capture_root']).resolve())
        if (row.get('status') != 'accepted_candidate' or canonical not in entries or canonical in lookup
                or row['id'] in seen_runtime or root in seen_roots
                or (row['id'] in source_entries and row['id'] != canonical)):
            raise ValueError('Selection/alias collision or unapproved native row')
        lookup[canonical] = row;seen_runtime.add(row['id']);seen_roots.add(root)
    if set(lookup) != set(entries):
        raise ValueError('Native selection and adopted quota IDs differ')
    pins = {str(path): sha256(path) for path in (source, adopted, selected)}
    authorized = {base['overlay_sha256']: dict(path=base_ref['path'], sha256=base_ref['sha256'])}
    loaded = {str(Path(base_ref['path']).resolve()): base}
    records, assignments, bindings, lineage, aliases, cores = [], {}, {}, {}, {}, {}
    for canonical, entry in sorted(entries.items()):
        row, original_entry = lookup[canonical], source_entries[canonical]
        raw = Path(row['capture_root']).resolve()
        metadata_path, scenario_path = raw/'metadata.json', raw/'scenario.json'
        meta = json.loads(metadata_path.read_text())
        if meta.get(UNIT_KEY)!=unit_reference:
            raise ValueError('Selected native unit geometry reference differs from source config')
        original = json.loads(Path(original_entry['metadata']).read_text())
        payload = sha256(scenario_path)
        if (payload != row['scenario_sha256'] or payload != entry['scenario_sha256']
                or sha256(entry['scenario']) != payload or meta['split'] != original['split']):
            raise ValueError('Selected native/adopted payload/source split binding differs')
        audit_path = Path(row.get('audit_path', raw/'audit.json'))
        audit = json.loads(audit_path.read_text())
        if audit.get('review_reasons') or audit.get('accepted_candidate') is False:
            raise ValueError('Reviewed native observation cannot enter finalized training split')
        reference = meta.get('training_split_overlay')
        observation_overlay = base
        if reference:
            path = str(Path(reference['path']).resolve())
            if sha256(path) != reference['sha256']:
                raise ValueError('Selected native overlay reference changed')
            if path not in loaded:
                loaded[path] = layout.read_overlay(path)
                layout.verify_source_pins(loaded[path])
            observation_overlay = loaded[path]
            if observation_overlay['overlay_sha256'] != reference['overlay_sha256']:
                raise ValueError('Selected native overlay content lineage differs')
            authorized[observation_overlay['overlay_sha256']] = dict(path=path, sha256=reference['sha256'])
        observed = layout.resolve(observation_overlay, row['id'], row, meta)
        expected = base['assignments'][canonical]
        if (observed['split'] != expected['split'] or observed['group_id'] != expected['group_id']
                or observation_overlay.get('recovery_aliases', {}).get(row['id'], row.get('source_cell_id', row['id'])) != canonical):
            raise ValueError('Selected recovery changed effective parent split/group or alias target')
        changed = payload != expected['scenario_sha256'] or observed['geometry_sha256'] != expected['geometry_sha256']
        if changed:
            mapping = observation_overlay['audit'].get('recovery_mapping', {})
            declared = meta.get('fresh_completion_source', {})
            if (mapping.get('source_id') != canonical or mapping.get('candidate_id') != row['id']
                    or mapping.get('old_scenario_sha256') != expected['scenario_sha256']
                    or mapping.get('new_scenario_sha256') != payload
                    or declared.get('source_cell_id') != canonical
                    or declared.get('source_scenario_sha256') != expected['scenario_sha256']
                    or declared.get('ordinal') != mapping.get('ordinal')
                    or observation_overlay.get('parent_overlay_sha256') != base['overlay_sha256']
                    or observation_overlay.get('recovery_implementation_sha256') != sha256(layout.__file__)):
                raise ValueError('Recovery mapping is not bound to the authoritative source overlay/implementation')
        planned_path = Path(observation_overlay.get('recovery_bindings', {}).get(canonical, original_entry)['metadata'])
        planned_meta=json.loads(planned_path.read_text())
        if not planned_fields_match(planned_meta, meta):
            raise ValueError('Captured native metadata changed planned non-overlay fields')
        pins[str(planned_path.resolve())]=sha256(planned_path)
        cores[canonical]=dict(path=str(planned_path.resolve()),sha256=pins[str(planned_path.resolve())],
            metadata_content_sha256=layout.digest({k:v for k,v in planned_meta.items() if k!='training_split_overlay'}))
        if row['id'] != canonical:
            if row['id'] in aliases and aliases[row['id']] != canonical:
                raise ValueError('Selected runtime alias collision')
            aliases[row['id']] = canonical
        records.append(dict(id=canonical, meta=meta, scenario_sha256=payload))
        assignments[canonical] = dict(expected, scenario_sha256=payload,
            geometry_sha256=layout.digest(layout.geometry(meta)), source_split=meta['split'])
        bindings[canonical] = dict(metadata=str(metadata_path), scenario=str(scenario_path),
            scenario_sha256=payload, metadata_content_sha256=layout.digest(
                {k:v for k,v in meta.items() if k != 'training_split_overlay'}))
        lineage[canonical] = dict(runtime_id=row['id'], capture_root=str(raw), changed_geometry=changed,
            source_scenario_sha256=expected['scenario_sha256'], selected_scenario_sha256=payload,
            observation_overlay_sha256=observation_overlay['overlay_sha256'],
            prepared_metadata_path=str(planned_path.resolve()),prepared_metadata_sha256=pins[str(planned_path.resolve())],
            raw_metadata_path=str(metadata_path),raw_metadata_sha256=sha256(metadata_path))
        for path in (metadata_path, scenario_path, audit_path):pins[str(path)]=sha256(path)
    # Recompute on the actual selected observations. Independent proposal
    # forks can collide with each other although each passed its own audit.
    group_of, _, _ = layout.groups(records, base['policy'])
    group_splits = defaultdict(set)
    for canonical, group in group_of.items():group_splits[group].add(assignments[canonical]['split'])
    if any(len(splits)>1 for splits in group_splits.values()):
        raise ValueError('Selected recovery geometries leak across train/val')
    for canonical, group in group_of.items():assignments[canonical]['group_id']=group
    audit = layout.audit_assignments(records, assignments, base['policy'])
    hard_groups = defaultdict(set)
    for record in records:
        binding = assignments[record['id']]
        if binding['split']=='train':hard_groups[binding['group_id']].update(layout.features(record['meta'],base['policy'])[0])
    remaining=Counter()
    for values in hard_groups.values():remaining.update(values)
    floors=base['audit'].get('train_floors', {})
    missing=[key for key,floor in floors.items() if remaining[key]<floor]
    if audit['errors'] or missing:
        raise ValueError('Final selected layout coverage/floors failed: '+str(audit['errors']+missing))
    audit.update(status='CPU_PASS',train_floors=floors,train_floor_violations=[],
        finalized_selected_native_frames=len(records),native_semantics_or_qa_changed=False,
        native_input_qualification_still_required=True)
    overlay=copy.deepcopy(base)
    overlay.pop('overlay_sha256',None)
    overlay.update(assignments=assignments,audit=audit,recovery_aliases=aliases,recovery_bindings=bindings,
        authorized_source_overlays=authorized,selected_lineage=lineage,
        prepared_core_sources=cores,
        finalization_source_hashes=pins,parent_overlay_sha256=base['overlay_sha256'],
        finalizer_sha256=sha256(__file__),bulk_ready=False,native_recovery_qualified=False)
    overlay['overlay_sha256']=layout.digest(overlay)
    output.mkdir(parents=True)
    overlay_path=output/'split_overlay.json';atomic_json(overlay_path,overlay)
    overlay_hash=sha256(overlay_path)
    candidate=copy.deepcopy(adopted_doc)
    candidate['split_overlay']=dict(path=str(overlay_path),sha256=overlay_hash,policy=layout.POLICY)
    candidate.update(status='SELECTED_LAYOUT_SPLIT_FINALIZED_CPU_ONLY; native input qualification pending',bulk_ready=False)
    for entry in candidate['scenes']:
        canonical=entry['id'];binding=assignments[canonical]
        # A manifest sidecar describes the immutable prepared capture input.
        # Keep native runtime diagnostics in their original pinned RAW file,
        # not inside this planned core (shadow reuse compares it exactly).
        prepared_metadata=Path(cores[canonical]['path'])
        meta=json.loads(prepared_metadata.read_text())
        meta['training_split_overlay']=dict(path=str(overlay_path),sha256=overlay_hash,
            overlay_sha256=overlay['overlay_sha256'],source_metadata_path=str(prepared_metadata),
            source_metadata_sha256=cores[canonical]['sha256'])
        path=output/'metadata'/(canonical+'.json');atomic_json(path,meta)
        entry.update(metadata=str(path),derived_training_split=binding['split'],derived_layout_group_id=binding['group_id'])
    candidate.setdefault('provenance',{})['selected_layout_finalization']=dict(source_manifest=str(source),
        source_manifest_sha256=pins[str(source)],selected_ledger=str(selected),selected_ledger_sha256=pins[str(selected)],
        raw_preserved=True,finalizer_sha256=sha256(__file__),
        metadata_sidecar_policy='Prepared input core plus split overlay; runtime diagnostics remain in pinned RAW metadata')
    manifest=output/'manifest.json';atomic_json(manifest,candidate)
    derived_selected=output/'derived_selected_ledger.jsonl'
    with derived_selected.open('x') as stream:
        for row in rows:
            stream.write(json.dumps(dict(row,training_split_overlay=dict(path=str(overlay_path),
                sha256=overlay_hash,overlay_sha256=overlay['overlay_sha256'])),ensure_ascii=False)+'\n')
    layout.validate_manifest_overlay(candidate)
    for path,checksum in pins.items():
        if sha256(path)!=checksum:raise ValueError('Finalization source changed: '+path)
    result=dict(manifest=str(manifest),manifest_sha256=sha256(manifest),split_overlay=str(overlay_path),
        split_overlay_sha256=overlay_hash,frames=len(records),splits=audit['splits'],
        derived_selected_ledger=str(derived_selected),derived_selected_ledger_sha256=sha256(derived_selected),
        changed_geometry=sum(row['changed_geometry'] for row in lineage.values()))
    atomic_json(output/'result.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','adopted','selected','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(finalize(args.source,args.adopted,args.selected,args.output),indent=2))
