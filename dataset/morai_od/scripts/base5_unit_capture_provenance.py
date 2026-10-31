"""Read-only completed-capture validation for opt-in native unit-size review.

Runtime diagnostic metadata may be added, but cannot replace planned actor
identity or the saved native unit review. Untagged legacy QA is unchanged.
This module performs no ROS imports, source writes, or acceptance promotion.
"""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from audit_morai_lidar_gt_visibility import parse_instances
import base5_unit_geometry_contract as unit


class UnitCaptureProvenanceError(unit.UnitGeometryContractError):
    """A completed unit-contract capture cannot be trusted; never a deficit."""


def require(condition, reason):
    if not condition:
        raise UnitCaptureProvenanceError(reason)


def read(path, pins):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), 'Exact nonsymlink unit capture input required: '+str(path))
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    require(str(path) not in pins or pins[str(path)] == digest, 'Unit capture input changed while reading')
    pins[str(path)] = digest
    return data


def scale_vector(value):
    require(isinstance(value, (list, tuple)) and len(value) == 3
            and all(type(x) in (int, float) and math.isfinite(x) and x > 0 for x in value),
            'Explicit finite actor scale required')
    return list(value)


def actor_identity(metadata):
    result, seen = [], set()
    actors = metadata.get('runtime_actors')
    require(isinstance(actors, list), 'Completed unit capture lacks runtime actor inventory')
    for actor in actors:
        uid, data_id = actor.get('unique_id'), actor.get('data_id')
        require(type(uid) is int and type(data_id) is int and uid not in seen,
                'Duplicate or invalid completed unit actor identity')
        seen.add(uid)
        assignment = actor.get('asset_scale') or {}
        require(isinstance(assignment, dict), 'Malformed completed unit scale assignment')
        factors = assignment.get('scale_xyz', [1., 1., 1.])
        probe = metadata.get('scale_probe', {})
        if probe.get('parent_uid') == uid:
            factors = probe.get('requested_scale')
        result.append(dict(unique_id=uid, data_id=data_id, name=actor.get('name'),
            source_list=actor.get('source_list'), motion=actor.get('motion'), scale_xyz=scale_vector(factors)))
    return result


def verify_native_components(capture, metadata, audit, scenario, pins):
    """Bind audit components to actual native summary records, not a label guess."""
    actors = {row['unique_id']: row for row in actor_identity(metadata)}
    for uid, actor in actors.items():
        matches = [row for row in scenario.get(actor['source_list'], []) if row.get('UNIQUEID') == uid]
        require(len(matches) == 1 and matches[0].get('DataID') == actor['data_id'],
                'Completed unit actor differs from captured native scenario identity')
        raw_scale = matches[0].get('scale', dict(x=1., y=1., z=1.))
        require(actor['scale_xyz'] == scale_vector([raw_scale[axis] for axis in 'xyz']),
                'Completed unit actor scale differs from captured native scenario')
    annotations = list(capture.glob('LIDAR_*/*_Instance.txt'))
    require(len(annotations) == 1, 'One exact completed unit native annotation required')
    content = read(annotations[0], pins)
    boxes = parse_instances(annotations[0])
    require(len(boxes) == len([line for line in content.splitlines() if line.strip()]),
            'Malformed completed unit native annotation')
    native, ordinals = {}, Counter()
    for box in boxes:
        pair = (box['unique_id'], box['raw_class'])
        key = pair + (ordinals[pair],)
        native[key] = box
        ordinals[pair] += 1
    components = audit.get('components')
    require(isinstance(components, list) and len(components) == len(native),
            'Completed unit audit/native component inventory differs')
    seen = set()
    for component in components:
        uid, raw_class = component['parent_uid'], component['raw_class']
        parts = str(component['component_id']).split(':')
        require(len(parts) == 3 and parts[:2] == [str(uid), raw_class],
                'Completed unit component identifier differs')
        key = (uid, raw_class, int(parts[2]))
        require(uid in actors and key in native and key not in seen,
                'Missing, foreign, or duplicate completed unit native component')
        seen.add(key)
        actor, box = actors[uid], native[key]
        require(component['asset'] == actor['name'] and component['data_id'] == actor['data_id'],
                'Completed unit native component/actor identity differs')
        summary = component.get('native_summary_size_m', component['size_m'])
        for actual, expected in ((summary, box['size']),
                                 (component['center_lidar_m'], box['center']),
                                 (component['rotation_rpy'], box['rotation_rpy'])):
            actual, expected = np.asarray(actual, float), np.asarray(expected, float)
            require(actual.shape == expected.shape == (3,) and np.isfinite(actual).all()
                    and np.allclose(actual, expected, atol=1e-6, rtol=1e-6),
                    'Completed unit saved audit differs from native summary geometry')


def verify_capture(entry, capture, manifest=None, ledger_row=None):
    """Validate an opt-in capture without altering it or upgrading its status.

    ``ledger_row`` is optional for callers without a ledger, but resume callers
    should supply it to bind the on-disk audit to the durable recorded audit.
    Additional live telemetry/capture-safety metadata is intentionally allowed.
    """
    pins, capture = {}, Path(capture)
    try:
        actual = json.loads(read(capture / 'metadata.json', pins))
        expected = json.loads(read(entry['metadata'], pins)) if entry and entry.get('metadata') else {}
        declared = (manifest or {}).get('config', {}).get(unit.KEY)
        expected_ref, actual_ref = expected.get(unit.KEY), actual.get(unit.KEY)
        if declared is None and expected_ref is None and actual_ref is None:
            return None
        require(entry is not None and expected_ref is not None and actual_ref == expected_ref,
                'Completed unit reference changed, removed, or lacks a planned source')
        if manifest is not None:
            require(declared == expected_ref, 'Completed unit reference differs from manifest')
        unit.load(expected_ref)
        require(actor_identity(expected) == actor_identity(actual),
                'Completed unit actor identity/order/motion/scale changed')
        scenario = json.loads(read(capture / 'scenario.json', pins))
        require(pins[str(capture / 'scenario.json')] == entry.get('scenario_sha256'),
                'Completed unit scenario differs from planned payload')
        if entry.get('scenario'):
            read(entry['scenario'], pins)
            require(pins[str(Path(entry['scenario']))] == entry['scenario_sha256'],
                    'Planned unit scenario source changed')
        audit = json.loads(read(capture / 'audit.json', pins))
        if ledger_row is not None:
            require(ledger_row.get('scenario_sha256') == entry['scenario_sha256']
                    and ledger_row.get('audit') == audit,
                    'Completed unit on-disk audit differs from durable ledger')
        verify_native_components(capture, actual, audit, scenario, pins)
        recomputed = unit.review(actual, audit)
        require(audit.get('native_unit_geometry_review') == recomputed,
                'Completed unit review missing or differs from recomputed native QA')
        require(set(recomputed['review_reasons']).issubset(audit.get('review_reasons', [])),
                'Completed unit review reasons removed from capture QA')
        if recomputed['review_reasons']:
            require(audit.get('accepted_candidate') is False
                    and (ledger_row is None or ledger_row.get('status') != 'accepted_candidate'),
                    'Completed unit mismatch incorrectly claims accepted status')
        for path, digest in pins.items():
            require(hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest,
                    'Completed unit input changed during provenance validation')
        return dict(verified=True, enabled=True, reference=expected_ref,
                    actors=len(actual['runtime_actors']), compared=sum(row['compared'] for row in recomputed['actors']),
                    review_reasons=list(recomputed['review_reasons']), ledger_audit_bound=ledger_row is not None,
                    raw_or_status_modified=False)
    except UnitCaptureProvenanceError:
        raise
    except (OSError, ValueError, TypeError, KeyError, IndexError, unit.UnitGeometryContractError) as error:
        raise UnitCaptureProvenanceError('Malformed or changed completed unit capture: '+str(error)) from error
