#!/usr/bin/env python3
"""Prepare a bounded large-scale diagnostic using the existing safe probe path.

This offline-only tool neither registers production profiles nor changes native
QA. The default 39 captures screen all requested assets; barrier yaw 90 is an
explicit follow-up, not an approval silently inferred from the first screen.
"""
import argparse
import copy
import json
import math
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
import prepare_base5_scale_probe as probe


def fresh_output(output, preserved_roots):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh diagnostic output required')
    roots = list(preserved_roots) + [probe.gen.geo.DEFAULT_RUNTIME_ASSET_DIR]
    if any(output == Path(root).resolve() or Path(root).resolve() in output.parents for root in roots):
        raise ValueError('Diagnostic output cannot be inside preserved source/reference or MORAI runtime')
    return output


def plan(phase='initial'):
    if phase == 'barrier-followup':
        return [(name, False, [2., 3.], [90.], None) for name in
                ('PE_Firewall_Orange', 'Steel_Barricade', 'Concrete_Barricade01')]
    if phase != 'initial':
        raise ValueError('Unknown qualification phase')
    return [('CargoBox', False, [1.5, 2., 3.], [0., 90.], None),
            ('CargoBox', True, [1.5], [0., 90.], ['sample_2_3_2']),
            ('WoodBox', False, [1.5, 2., 3.], [0., 90.], None),
            ('YellowBarrel', False, [1.5, 2.], [0., 90.], None),
            ('PE_Drum', False, [1.5, 2.], [0., 90.], None)] + [
                (name, False, [2., 3.], [0.], None) for name in
                ('PE_Firewall_Orange', 'Steel_Barricade', 'Concrete_Barricade01')]


def selected_count(items):
    return sum(len(yaws) * (len(only) if only else (6 if axis else 1 + len(factors)))
               for _, axis, factors, yaws, only in items)


def merge_references(paths, required):
    assets, pins = {}, {}
    required = set(required)
    if not required or not required <= set(probe.WHITELIST):
        raise ValueError('Explicit nonempty whitelisted requested-asset subset required')
    for path in paths:
        path = Path(path).resolve(strict=True)
        document = json.loads(path.read_text())
        if not isinstance(document.get('assets'),dict):
            raise ValueError('Native unit reference assets mapping required')
        # A large reference inventory may contain unrelated assets. They are
        # neither accepted nor copied into this bounded requested-asset deck.
        reference = {name:record for name,record in document['assets'].items() if name in required}
        pins[str(path)] = sha256(path)
        for name, record in reference.items():
            if (not isinstance(record,dict) or record.get('data_id',probe.WHITELIST[name][0]) != probe.WHITELIST[name][0]
                    or record.get('source_list','objectList') != 'objectList'
                    or record.get('scale_xyz',[1.,1.,1.]) != [1.,1.,1.]):
                raise ValueError('Requested native unit identity/container/scale differs: '+name)
            for field in ('bbox_size_m','pivot_to_bbox_center_local_m'):
                vector = record.get(field)
                if (not isinstance(vector,(list,tuple)) or len(vector)!=3 or any(
                        type(value) not in (int,float) or not math.isfinite(value)
                        or (field=='bbox_size_m' and value<=0) for value in vector)):
                    raise ValueError('Finite requested native unit size/pivot required: '+name)
            if name in assets and assets[name] != record:
                raise ValueError('Conflicting unit references: ' + name)
            assets[name] = copy.deepcopy(record)
    missing = set(required) - set(assets)
    if missing:
        raise ValueError('Missing native unit placement reference: ' + ','.join(sorted(missing)))
    return dict(schema='base5_scale_qualification_unit_reference_v1', assets=assets,
                sources=copy.deepcopy(pins), placement_only=True, production_approved=False), pins


def prepare(manifest, output, unit_references, *, phase='initial', distance_m=10.):
    items = plan(phase)
    expected = selected_count(items)
    if not 1 <= expected <= 40:
        raise ValueError('Bounded qualification deck must contain at most 40 captures')
    source = Path(manifest).resolve(strict=True)
    output = fresh_output(output,[source.parent]+[Path(path).resolve().parent for path in unit_references])
    reference, pins = merge_references(unit_references, {row[0] for row in items})
    pins[str(source)] = sha256(source)
    # Validate the bounds before any output writes; prepare() repeats these
    # checks and enforces supported Ego, ground initializer and bbox clearance.
    for name, axis, factors, yaws, _ in items:
        probe.validate_inputs([name], axis, factors, yaws, distance_m)
    reference_path = output / 'unit_reference.json'
    atomic_json(reference_path, reference)
    entries, config = [], None
    for ordinal, (name, axis, factors, yaws, only) in enumerate(items):
        fragment_path = probe.prepare(source, output / 'fragments' / f'{ordinal:02d}_{name}',
            [name], axis=axis, factors=factors, yaws=yaws, distance_m=distance_m,
            unit_reference=reference_path)
        fragment = json.loads(fragment_path.read_text())
        if config is None:
            config = fragment['config']
        elif config != fragment['config']:
            raise ValueError('Fragment safety/capture configuration differs')
        for entry in fragment['scenes']:
            meta = json.loads(Path(entry['metadata']).read_text())
            if only and meta['scale_probe']['profile'] not in only:
                continue
            scene = json.loads(Path(entry['scenario']).read_text())
            new_id = 'qualification_' + f'{len(entries):03d}_' + entry['id'].split('_', 2)[2]
            meta['spec']['index'] = len(entries)
            meta['large_scale_qualification'] = dict(phase=phase, diagnostic_only=True,
                native_mesh_boundary_review_required=True, native_corner_self_consistency_is_mesh_proof=False,
                physical_collider_contact_certified=False,
                barrier_second_yaw_pending=(phase == 'initial' and name in (
                    'PE_Firewall_Orange', 'Steel_Barricade', 'Concrete_Barricade01')))
            sp, mp = (output/'scenarios'/(new_id+suffix) for suffix in ('.json', '_metadata.json'))
            atomic_json(sp, scene)
            atomic_json(mp, meta)
            entries.append(dict(entry, id=new_id, scenario=str(sp), metadata=str(mp), scenario_sha256=sha256(sp)))
    if len(entries) != expected or len({row['id'] for row in entries}) != expected:
        raise ValueError('Qualification inventory differs from the preregistered plan')
    if any(sha256(path) != checksum for path, checksum in pins.items()):
        raise ValueError('Preserved source/reference changed during preparation')
    config['dataset_id'] = 'base5_scale_qualification_261017_' + phase
    deck = dict(schema_version=2, dataset_id=config['dataset_id'], config=config,
        scenes=entries, count=expected, status='DIAGNOSTIC_ONLY; no production profile or bulk approval',
        scale_probe_registration=dict(source_files=pins, source_unchanged=True,
            preparation_script_sha256=sha256(Path(__file__)), maximum_scenes=40,
            requested_pivot_distance_m=distance_m, production_quota_claim=False,
            native_geometry_qualified=False, phase=phase,
            barrier_yaw90_followup_required=(phase == 'initial')))
    path = output/'manifest.json'
    atomic_json(path, deck)
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--unit-reference', type=Path, action='append', required=True)
    parser.add_argument('--phase', choices=['initial', 'barrier-followup'], default='initial')
    parser.add_argument('--distance-m', type=float, default=10.)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output, args.unit_reference,
                  phase=args.phase, distance_m=args.distance_m))
