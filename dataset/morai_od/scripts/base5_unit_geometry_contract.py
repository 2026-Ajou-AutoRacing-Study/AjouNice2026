"""Opt-in native unit-size change detection, never GT/reference replacement.

Only registered single-component static physical obstacles are compared. The
reference is an expected local native size, not a conservative spacing envelope
or mesh/collider certificate. A mismatch is review, never a label correction.
"""
import json
import hashlib
import math
from pathlib import Path

from base5_capture_safety import sha256


KEY = 'native_unit_geometry_reference'
SCHEMA = 'base5_native_unit_geometry_review_v1'


class UnitGeometryContractError(RuntimeError):
    """Reference/provenance failure stops capture; it is not a scene deficit."""


def load(reference):
    if (not isinstance(reference,dict) or set(reference) != {'path','sha256'}
            or not Path(reference['path']).is_absolute()):
        raise UnitGeometryContractError('Explicit absolute pinned unit geometry reference required')
    path = Path(reference['path'])
    if path.is_symlink() or not path.is_file():
        raise UnitGeometryContractError('Unit geometry review reference changed')
    data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=reference['sha256']:
        raise UnitGeometryContractError('Unit geometry review reference changed')
    value = json.loads(data)
    if (value.get('schema') != SCHEMA or not isinstance(value.get('assets'),dict)
            or not value['assets'] or value.get('training_gt_modified') is not False):
        raise UnitGeometryContractError('Explicit review-only unit geometry policy required')
    for name,record in value['assets'].items():
        size = record.get('native_size_lwh_m')
        if (not isinstance(name,str) or not name or type(record.get('data_id')) is not int
                or record.get('source_list') != 'objectList' or record.get('raw_class') != 'Obstacle'
                or not isinstance(size,list) or len(size) != 3
                or any(type(x) not in (int,float) or not math.isfinite(x) or x <= 0 for x in size)):
            raise UnitGeometryContractError('Explicit physical obstacle identity and native local size required')
    sources=value.get('sources')
    catalog_ref=value.get('catalog_reference')
    if not isinstance(sources,list) or not sources or not isinstance(catalog_ref,dict):
        raise UnitGeometryContractError('Pinned native sources and catalog required')
    catalog_path=Path(catalog_ref['path'])
    if catalog_path.is_symlink() or not catalog_path.is_file():
        raise UnitGeometryContractError('Unit geometry catalog changed')
    catalog_bytes=catalog_path.read_bytes()
    if hashlib.sha256(catalog_bytes).hexdigest()!=catalog_ref['sha256']:
        raise UnitGeometryContractError('Unit geometry catalog changed')
    catalog={a['name']:a for a in json.loads(catalog_bytes)['assets']}
    expected_records={}
    for source in sources:
        source_path=Path(source['path'])
        if source_path.is_symlink() or not source_path.is_file():
            raise UnitGeometryContractError('Unit geometry review source changed')
        data=source_path.read_bytes()
        if hashlib.sha256(data).hexdigest()!=source['sha256']:
            raise UnitGeometryContractError('Unit geometry review source changed')
        document=json.loads(data)
        names=source.get('assets')
        if not isinstance(names,list) or not names or len(names)!=len(set(names)):
            raise UnitGeometryContractError('Explicit unique source asset subset required')
        for name in names:
            if name in expected_records or name not in document['assets']:
                raise UnitGeometryContractError('Native unit source asset missing or duplicated')
            expected_records[name]=(source,document['assets'][name])
    if set(expected_records)!=set(value['assets']):
        raise UnitGeometryContractError('Unit source and review asset sets differ')
    for name,record in value['assets'].items():
        actual=catalog.get(name,{})
        source,original=expected_records[name]
        if (actual.get('data_id')!=record['data_id'] or actual.get('source_list')!='objectList'
                or actual.get('semantic_class')!='obstacle'
                or original.get('data_id')!=record['data_id']
                or original.get('scale_xyz',[1.,1.,1.])!=[1.,1.,1.]
                or original.get('source_list','objectList')!='objectList'
                or record.get('reference_source')!=source
                or original.get('bbox_size_m')!=record['native_size_lwh_m']):
            raise UnitGeometryContractError('Native unit source/catalog identity or dimensions differ')
    return value


def validate_actor_identities(metadata,policy):
    """A known physical DataID cannot evade review through a renamed actor.

    A genuinely different asset remains outside this opt-in policy. Validate
    before capture as well as during native QA; do not infer identity from the
    spacing envelope or from a training class.
    """
    actors=metadata.get('runtime_actors')
    if not isinstance(actors,list):
        raise UnitGeometryContractError('Unit reference requires an explicit runtime actor inventory')
    names_by_id={record['data_id']:name for name,record in policy['assets'].items()}
    for actor in actors:
        name=actor.get('name')
        record=policy['assets'].get(name)
        known_name=names_by_id.get(actor.get('data_id'))
        if known_name is not None and name!=known_name:
            raise UnitGeometryContractError('Known unit DataID/name identity differs')
        if record is not None and (actor.get('data_id')!=record['data_id']
                or actor.get('source_list')!=record['source_list']):
            raise UnitGeometryContractError('Unit reference actor identity/container differs')


def review(metadata,audit):
    reference = metadata.get(KEY)
    if reference is None:
        return dict(enabled=False,actors=[],review_reasons=[])
    policy = load(reference)
    validate_actor_identities(metadata,policy)
    rows, reasons = [], []
    for actor in metadata.get('runtime_actors',[]):
        name = actor['name']
        record = policy['assets'].get(name)
        if record is None:
            continue
        factors = (actor.get('asset_scale') or {}).get('scale_xyz',[1.,1.,1.])
        probe = metadata.get('scale_probe',{})
        if probe.get('parent_uid') == actor['unique_id']:
            factors = probe['requested_scale']
        if list(factors) != [1.,1.,1.] or actor.get('motion') not in ('static','standing'):
            continue
        components = [c for c in audit['components'] if c['parent_uid'] == actor['unique_id']]
        if len(components) != 1 or components[0]['raw_class'] != 'Obstacle':
            # Never reduce a composite parent to one envelope, even if the
            # asset unexpectedly changes its native component representation.
            rows.append(dict(parent_uid=actor['unique_id'],name=name,compared=False,
                reason='not_single_physical_obstacle_component'))
            reasons.append('unit_native_component_contract_changed')
            continue
        expected, actual = record['native_size_lwh_m'], components[0]['size_m']
        if (len(actual) != 3 or any(type(x) not in (int,float) or not math.isfinite(x) or x <= 0 for x in actual)):
            raise UnitGeometryContractError('Invalid native unit component size')
        errors = [abs(a-b) for a,b in zip(actual,expected)]
        tolerance = [max(.03,.03*x) for x in expected]
        matched = all(error <= limit for error,limit in zip(errors,tolerance))
        if not matched:
            reasons.append('unit_native_bbox_size_mismatch')
        rows.append(dict(parent_uid=actor['unique_id'],name=name,compared=True,matched=matched,
            native_size_lwh_m=actual,expected_native_unit_size_lwh_m=expected,
            absolute_error_m=errors,tolerance_m=tolerance,reference=reference,
            raw_annotation_modified=False,mesh_geometry_certified=False))
    return dict(enabled=True,policy=SCHEMA,reference=reference,actors=rows,
                review_reasons=sorted(set(reasons)),training_gt_modified=False)


def validate_entry(entry,manifest):
    metadata = json.loads(Path(entry['metadata']).read_text())
    expected = manifest.get('config',{}).get(KEY)
    actual = metadata.get(KEY)
    if expected != actual:
        raise UnitGeometryContractError('Manifest/metadata unit reference changed or removed')
    if expected is not None:
        validate_actor_identities(metadata,load(expected))
