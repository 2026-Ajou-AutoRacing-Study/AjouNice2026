#!/usr/bin/env python3
"""Attach a versioned review-only native unit reference to a new candidate.

Apply before the derived train/val overlay. Existing source metadata, raw GT,
scenarios, frozen specs and conservative placement envelopes remain unchanged.
"""
import argparse
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
import base5_unit_geometry_contract as unit


def prepare(source,reference_specs,output):
    source,output = Path(source).resolve(strict=True),Path(output).resolve()
    if output.exists() or source.parent == output or source.parent in output.parents:
        raise FileExistsError('Fresh output outside preserved source required')
    deck = json.loads(source.read_text())
    if deck.get('split_overlay') or unit.KEY in deck.get('config',{}):
        raise ValueError('Attach a fresh unit review policy before layout split preparation')
    assets,sources,pins = {},[],{str(source):sha256(source)}
    # Sources are explicit subsets, so an unstable asset cannot silently be
    # supplied by an unrelated inventory or conflicting reference precedence.
    for spec in reference_specs:
        path = Path(spec['path']).resolve(strict=True)
        if sha256(path) != spec['sha256']:
            raise ValueError('Declared reference SHA differs')
        document = json.loads(path.read_text())
        wanted = spec['assets']
        if not isinstance(wanted,list) or not wanted or len(wanted) != len(set(wanted)):
            raise ValueError('Explicit unique nonempty reference subset required')
        sources.append(dict(path=str(path),sha256=spec['sha256'],assets=wanted))
        pins[str(path)] = spec['sha256']
        for name in wanted:
            if name in assets or name not in document['assets']:
                raise ValueError('Duplicate or missing unit reference asset: '+name)
            record = document['assets'][name]
            assets[name] = dict(data_id=record['data_id'],source_list='objectList',raw_class='Obstacle',
                native_size_lwh_m=record['bbox_size_m'],reference_source=sources[-1],
                use='unexpected native unit size review only; not a placement or GT replacement')
    # Bind semantic eligibility to the actual catalog, not arbitrary source
    # class/container labels. Vehicles, pedestrians and composites never enter.
    from build_base5_mr_asset_catalog import DEFAULT_CATALOG
    catalog = json.loads(DEFAULT_CATALOG.read_text())
    catalog_assets = {a['name']:a for a in catalog['assets']}
    for name,record in assets.items():
        actual = catalog_assets[name]
        if (actual['data_id'] != record['data_id'] or actual['source_list'] != 'objectList'
                or actual.get('semantic_class') != 'obstacle'):
            raise ValueError('Unit reference is not a catalog physical obstacle: '+name)
    pins[str(DEFAULT_CATALOG.resolve())] = sha256(DEFAULT_CATALOG)
    policy_path = output/'unit_geometry_review.json'
    atomic_json(policy_path,dict(schema=unit.SCHEMA,assets=assets,sources=sources,
        catalog_reference=dict(path=str(DEFAULT_CATALOG.resolve()),sha256=sha256(DEFAULT_CATALOG)),
        training_gt_modified=False,placement_envelope_modified=False))
    reference = dict(path=str(policy_path),sha256=sha256(policy_path))
    unit.load(reference)
    candidate = copy.deepcopy(deck)
    candidate['config'][unit.KEY] = reference
    for entry in candidate['scenes']:
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('Source scenario changed')
        old = Path(entry['metadata'])
        pins[str(old.resolve())],pins[str(Path(entry['scenario']).resolve())] = sha256(old),entry['scenario_sha256']
        metadata = json.loads(old.read_text())
        if unit.KEY in metadata:
            raise ValueError('Source metadata already has a unit contract')
        metadata[unit.KEY] = reference
        new = output/'metadata'/(entry['id']+'.json')
        atomic_json(new,metadata)
        entry['metadata'] = str(new)
    candidate.setdefault('provenance',{})['unit_geometry_review'] = dict(source_manifest=str(source),
        source_manifest_sha256=pins[str(source)],reference=reference,helper_sha256=sha256(__file__),
        raw_gt_modified=False,scenario_geometry_changed=False)
    atomic_json(output/'manifest.json',candidate)
    if any(sha256(path) != checksum for path,checksum in pins.items()):
        raise RuntimeError('Preserved unit reference/source changed')
    atomic_json(output/'source_pins.json',pins)
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--reference-specs',type=Path,required=True,
        help='JSON list of explicit {path,sha256,assets} reference subsets')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    print(prepare(args.manifest,json.loads(args.reference_specs.read_text()),args.output))
