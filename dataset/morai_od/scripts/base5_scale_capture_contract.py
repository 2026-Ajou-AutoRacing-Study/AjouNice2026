"""Fail-closed scaled-scene capture and native-dimension checks.

Native annotations are preserved. A camera-positive result alone must not admit
a visibly scaled mesh whose 3D annotation remains at its original dimensions.
"""
import json
import math
from pathlib import Path

from base5_asset_scale import WHITELIST, verify_actor

ACTOR_LISTS = ('objectList', 'vehicleList', 'pedestrianList', 'aiPedstrianList')


def vector(value):
    if not isinstance(value,dict) or any(isinstance(value[k],bool) for k in ('x','y','z')):
        raise ValueError('Explicit numeric scenario scale vector required')
    result = [float(value[k]) for k in ('x', 'y', 'z')]
    if not all(math.isfinite(x) and x > 0 for x in result):
        raise ValueError('Finite positive scenario scale required')
    return result


def raw_variants(scene):
    """Inspect raw actors first; metadata absence must never hide a mesh scale."""
    result = {}
    for source in ACTOR_LISTS:
        for actor in scene.get(source, []):
            size = vector(actor.get('scale', dict(x=1., y=1., z=1.)))
            if size == [1., 1., 1.]:continue
            uid = actor['UNIQUEID']
            if source != 'objectList' or uid in result:
                raise ValueError('Nonunit scale requires one uniquely identified physical objectList actor')
            result[uid] = actor
    return result


def _bind_scene(scene, metadata):
    actual = raw_variants(scene)
    requested = {}
    for actor in metadata.get('runtime_actors', []):
        value = (actor.get('asset_scale') or {}).get('scale_xyz')
        probe = metadata.get('scale_probe', {})
        if probe.get('parent_uid') == actor['unique_id']:
            value = probe.get('requested_scale')
        if value is None:continue
        if not isinstance(value,(list,tuple)) or len(value)!=3:
            raise ValueError('Explicit finite three-axis metadata scale required')
        value = vector(dict(zip(('x','y','z'),value)))
        if value == [1.,1.,1.]:continue
        uid = actor['unique_id']
        if uid in requested:raise ValueError('Duplicate scaled runtime actor UID')
        requested[uid] = (actor,value)
    if set(actual)!=set(requested):
        raise ValueError('Captured raw nonunit actors differ from explicit scale metadata')
    for uid,(actor,size) in requested.items():
        raw = actual[uid]
        matches = [a for a in scene.get('objectList',[]) if a.get('UNIQUEID')==uid]
        if (actor['name'] not in WHITELIST or actor['data_id']!=WHITELIST[actor['name']][0]
                or actor['source_list']!='objectList' or actor['motion'] not in ('static','standing')
                or len(matches)!=1 or raw['DataID']!=actor['data_id'] or vector(raw['scale'])!=size):
            raise ValueError('Native corner derivation requires captured nonunit physical-obstacle identity')
    return set(requested)


def validate_entry(entry, manifest):
    scene = json.loads(Path(entry['scenario']).read_text())
    meta = json.loads(Path(entry['metadata']).read_text())
    variants = list(raw_variants(scene).values())
    _bind_scene(scene,meta)
    if not variants:
        return
    diagnostic = manifest['config'].get('capture', {}).get('asset_scale_probe') is True
    if diagnostic:
        probe = meta.get('scale_probe', {})
        if (not str(manifest.get('status', '')).startswith('DIAGNOSTIC_ONLY')
                or not 1 <= len(manifest['scenes']) <= 64 or len(variants) != 1
                or len(meta.get('runtime_actors', [])) != 1):
            raise ValueError('Scale probes must be explicit bounded single-parent diagnostics')
        actor = meta['runtime_actors'][0]
        raw = variants[0]
        if (actor['name'] not in WHITELIST or actor['data_id'] != WHITELIST[actor['name']][0]
                or actor['source_list'] != 'objectList' or actor['motion'] not in ('static', 'standing')
                or raw['UNIQUEID'] != actor['unique_id'] or raw['DataID'] != actor['data_id']
                or probe.get('parent_uid') != actor['unique_id'] or probe.get('asset') != actor['name']
                or probe.get('requested_scale') != vector(raw['scale'])):
            raise ValueError('Unregistered or ineligible scaled diagnostic actor')
        return
    policy = manifest['config'].get('asset_scale_policy')
    for raw in variants:
        actors = [a for a in meta['runtime_actors'] if a['unique_id'] == raw['UNIQUEID']]
        if len(actors) != 1 or not policy or not actors[0].get('asset_scale'):
            raise ValueError('Production scale requires qualified frozen policy and assignment')
        verify_actor(raw, actors[0]['asset_scale'], policy)
        if actors[0]['asset_scale'].get('candidate_only'):
            raise ValueError('Candidate-only scale is not production qualified')
    from base5_asset_scale import verify_metadata
    # Runtime metadata deliberately stores actor identity, not the catalog's
    # semantic/ontology/collection fields. Re-read the authoritative catalog;
    # do not fabricate a semantic class or weaken its eligibility checks.
    from build_base5_mr_asset_catalog import DEFAULT_CATALOG
    catalog={a['data_id']:a for a in json.loads(DEFAULT_CATALOG.read_text())['assets']}
    verify_metadata(scene,meta,manifest['config'],catalog)


def native_size_review(metadata, audit):
    results, reasons = [], []
    for actor in metadata.get('runtime_actors', []):
        assignment = actor.get('asset_scale') or {}
        probe = metadata.get('scale_probe', {})
        scale = assignment.get('scale_xyz')
        if probe.get('parent_uid') == actor['unique_id']:
            scale = probe.get('requested_scale')
        if scale is None or list(scale) == [1., 1., 1.]:
            continue
        components = [c for c in audit['components'] if c['parent_uid'] == actor['unique_id']]
        expected = actor['estimated_size']
        valid = len(components) == 1 and components[0]['raw_class'] == 'Obstacle'
        actual = components[0]['size_m'] if valid else None
        error = [abs(a-b) for a, b in zip(actual, expected)] if actual else None
        tolerance = [max(.03, abs(x)*.03) for x in expected]
        matched = bool(error is not None and all(e <= t for e, t in zip(error, tolerance)))
        if not matched:
            reasons.append('scaled_native_bbox_size_mismatch')
        results.append(dict(parent_uid=actor['unique_id'], scale_xyz=scale,
            native_size_m=actual, expected_placement_size_m=expected, absolute_error_m=error,
            tolerance_m=tolerance, matched=matched,
            diagnostic_only=bool(probe), native_annotation_modified=False))
    return dict(actors=results, review_reasons=sorted(set(reasons)),
                policy='scaled-native-dimensions-v155; raw preserved; no inferred GT repair')


def registered_variants(root, metadata):
    """Bind audit opt-in to the actually captured scenario, never a size guess."""
    path = Path(root)/'scenario.json'
    if not path.is_file():
        # Legacy unit fixtures/archives did not all retain a scenario. A new
        # explicit scale contract never gets that historical exemption.
        if (metadata.get('scale_probe') or 'asset_scale_plan' in metadata.get('spec',{})
                or any(a.get('asset_scale') is not None for a in metadata.get('runtime_actors',[]))):
            raise ValueError('Explicit scale metadata lacks its captured raw scenario')
        return set()
    return _bind_scene(json.loads(path.read_text()),metadata)


def verify_capture(entry, capture, manifest):
    """Rebind completed scale captures without trusting unchanged planned files."""
    capture = Path(capture)
    actual = json.loads((capture/'metadata.json').read_text())
    variants = registered_variants(capture,actual)
    expected = json.loads(Path(entry['metadata']).read_text()) if entry and entry.get('metadata') else {}
    scaled_expected = ('asset_scale_plan' in expected.get('spec',{})
        or any((a.get('asset_scale') or {}).get('scale_xyz',[1.,1.,1.]) != [1.,1.,1.]
               for a in expected.get('runtime_actors',[]))
        or expected.get('scale_probe',{}).get('requested_scale',[1.,1.,1.]) != [1.,1.,1.])
    if not variants and not scaled_expected:return
    if not entry or not entry.get('scenario_sha256'):
        raise ValueError('Completed scaled capture lacks a pinned source entry')
    from base5_capture_safety import sha256
    if sha256(capture/'scenario.json') != entry['scenario_sha256']:
        raise ValueError('Completed scaled scenario differs from frozen source')
    for key in ('spec','runtime_actors','scale_probe'):
        if actual.get(key)!=expected.get(key):
            raise ValueError('Completed scaled metadata differs from frozen source: '+key)
    validate_entry(dict(scenario=str(capture/'scenario.json'),metadata=str(capture/'metadata.json')),manifest)
    from report_base5_compound_evidence import _validate_native_geometry
    audit = json.loads((capture/'audit.json').read_text())
    _validate_native_geometry(capture,audit['components'])
