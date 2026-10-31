#!/usr/bin/env python3
"""새 출력 전용: compound 최종 방향의 정적 reference와 넓은 독립 지면 지원.

원형 quality/campaign/collector는 수정하지 않는다. 원형 prepare 호출 세 곳만
메모리 AST로 연결하며 기존 공간 샘플링, 보충 상한, native QA는 유지한다.
추가 reference도 calibration-only이고 학습 quota를 채우지 않는다.
"""
import argparse
import ast
from collections import Counter
from contextlib import contextmanager
import copy
import fcntl
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import MultiPoint

import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_motion_ground as ground
import base5_native_box_corners as native
import base5_unit_capture_provenance as unit_capture
from base5_capture_safety import atomic_json, collision_window, sha256
from base5_ground_probe import ego_to_world, plane_height, supported, surface_at, world_cloud
import run_base5_capture_campaign as campaign
import run_base5_capture_campaign_spatial as spatial
import run_base5_quality_loop as quality


SCHEMA = 'base5_final_direction_reference_runtime_v1'
POLICY = 'opt-in-final-direction-original-pivot-compound-support-envelope-v1'
GUARD = 'direction_reference_runtime_contract.json'
RECORDS = 'direction_reference_records.json'
SOURCE = Path(quality.__file__).resolve()
EXTRA_SUPPORT_M = 1.0
MAXIMUM_TARGET_TOLERANCE_M = .4
MAXIMUM_CELLS = 4


class DirectionReferenceContractError(RuntimeError):
    """계약 손상은 일반 부족분이 아니며 모든 후속 load를 중단한다."""


class CompoundEnvelopeUnsupported(ValueError):
    def __init__(self, details):
        # 원형 bounded near-terrain 복구의 기존 분기를 그대로 사용한다.
        super().__init__('Static pivot terrain unsupported')
        self.details = details


def require(value, message):
    if not value:
        raise DirectionReferenceContractError(message)


def read(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError,ValueError,TypeError) as error:
        raise DirectionReferenceContractError('Direction evidence missing or malformed: '+str(path)) from error


def verify_hashes(values):
    try:
        pins.verify_hashes(values)
    except (OSError,ValueError,TypeError) as error:
        raise DirectionReferenceContractError('Direction immutable input changed: '+str(error)) from error


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def runtime_hashes():
    return pins.collect_runtime(entrypoints=(Path(__file__).name, spatial.__file__.split('/')[-1]))


def file_inventory(root):
    root = Path(root).resolve(strict=True)
    result = {}
    for path in sorted(root.rglob('*')):
        require(not path.is_symlink(), 'Immutable direction evidence cannot contain symlinks')
        if path.is_file():
            stat = path.stat()
            result[str(path.resolve())] = dict(sha256=sha256(path), size=stat.st_size,
                inode=stat.st_ino, mtime_ns=stat.st_mtime_ns)
    require(result, 'Nonempty direction evidence required')
    return result


def input_pins(manifest):
    """Subset에도 full frozen_specs가 남을 수 있어 count 재해석 없이 실제 참조를 핀한다."""
    manifest = Path(manifest).resolve(strict=True)
    deck = read(manifest)
    result = {str(manifest):sha256(manifest)}
    pending = []

    def add(path, expected=None, inspect=False):
        path = pins._path(path, manifest.parent)
        actual = sha256(path)
        require(expected is None or actual == expected, 'Direction input SHA differs: '+str(path))
        require(str(path) not in result or result[str(path)] == actual, 'Direction input changed')
        new = str(path) not in result
        result[str(path)] = actual
        if inspect and new and path.suffix == '.json':
            pending.append(path)

    require(deck.get('count') == len(deck.get('scenes', [])) and deck['scenes'], 'Complete nonempty source deck required')
    seen = set()
    for entry in deck['scenes']:
        require(entry['id'] not in seen, 'Duplicate direction source ID')
        seen.add(entry['id'])
        add(entry['scenario'], entry['scenario_sha256'])
        add(entry['metadata'], inspect=True)
    for path, expected in pins._references(deck.get('config', {})):
        add(path, expected, True)
    for path, expected in pins._references(deck.get('split_overlay', {})):
        add(path, expected, True)
    for path, expected in pins._default_references(deck).items():
        add(path, expected)
    while pending:
        for path, expected in pins._references(read(pending.pop())):
            add(path, expected, True)
    verify_hashes(result)
    return dict(sorted(result.items()))


def single(manifest):
    deck = read(manifest)
    require(deck.get('count') == len(deck.get('scenes', [])) == 1, 'Single-cell direction preparation required')
    entry = deck['scenes'][0]
    require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Direction source payload changed')
    return deck, entry, read(entry['metadata']), read(entry['scenario'])


def calibration_session(session, manifest=None):
    """실제 complete native, collision, Park, unit와 exact launch를 읽기 전용 대조한다."""
    session = Path(session).resolve(strict=True)
    before = file_inventory(session)
    content = (session/'ledger.jsonl').read_bytes()
    require(content and content.endswith(b'\n'), 'Complete calibration ledger required')
    try:
        rows = [json.loads(line) for line in content.splitlines() if line.strip()]
    except (ValueError,TypeError) as error:
        raise DirectionReferenceContractError('Malformed immutable calibration ledger') from error
    require(len(rows) == 1 and rows[0].get('status') in ('accepted_candidate', 'review'), 'One completed calibration observation required')
    row = rows[0];root = Path(row['capture_root']).resolve(strict=True)
    require(root.parent == session/'captures', 'Calibration root escaped its actual session')
    metadata, audit = read(root/'metadata.json'), read(root/'audit.json')
    require(metadata.get('calibration_only') is True and row.get('audit') == audit
        and sha256(root/'scenario.json') == row['scenario_sha256'], 'Calibration native/audit/payload binding changed')
    require(quality.scene_rejection(row) is None, 'Unsafe calibration contact/support')
    require(not any('collision' in r or 'ego_vertical' in r for r in audit.get('review_reasons', [])), 'Unsafe calibration reasons')
    require(read(session/'shutdown_safety.json').get('park_verified') is True, 'Calibration final Park is unverified')
    evidence = read(root/'collision_history.json');summary = evidence['summary']
    actual = collision_window(evidence['samples'], summary['window_start_monotonic'], summary['window_end_monotonic'])
    require(summary.get('observed') is True and summary.get('no_collision_verified') is True
        and summary.get('non_ego_uids') == [] and actual['observed'] and actual['no_collision_verified'], 'Calibration collision evidence missing')
    contract = read(session/'session_contract.json')
    require(contract.get('sensor_sha256') == sha256(root/'sensor_reference.json'), 'Calibration sensor binding changed')
    if manifest is not None:
        deck, entry, planned, _ = single(manifest)
        require(contract.get('manifest_sha256') == sha256(manifest) and row['id'] == entry['id']
            and row['scenario_sha256'] == entry['scenario_sha256'], 'Calibration exact launch differs')
        from finalize_base5_layout_split import planned_fields_match
        require(planned_fields_match(planned, metadata), 'Calibration planned metadata differs')
        unit_capture.verify_capture(entry, root, deck, row)
    else:
        quality.native_payloads([dict(capture_root=str(session), kind='direction_reference')])
    require(file_inventory(session) == before, 'Calibration changed during validation')
    return dict(root=str(root), session=str(session), row=row, inventory=before)


def corners_by_uid(root):
    root = Path(root)
    files = list(root.glob('LIDAR_*/*_Instance.txt'))
    require(len(files) == 1, 'One exact native annotation required for reference envelope')
    records = native.load_matching_corners(files[0])
    meta, telemetry = read(root/'metadata.json'), read(root/'telemetry.json')
    transform = np.asarray(meta['lidar_to_ego'], float)
    result = {}
    for (uid, _, _), record in records.items():
        points = np.asarray(record['corners_lidar_m'], float)
        require(points.shape == (8, 3) and np.isfinite(points).all(), 'Eight actual finite corners required')
        # Same world convention as the unchanged independent-ground runtime QA.
        ego = (np.c_[points, np.ones(8)] @ transform.T)[:, :3]
        result.setdefault(uid, []).append(ego_to_world(ego, telemetry['ego_before']))
    return result


def angle_difference(a, b):
    return abs((float(a)-float(b)+180.) % 360.-180.)


def selected_yaws(original_meta, preliminary_meta, preliminary_scene):
    result = {}
    for actor in ground.moving_actors(original_meta):
        uid = actor['unique_id']
        planned = next(a for a in preliminary_meta['runtime_actors'] if a['unique_id'] == uid)
        path = planned['motion_path'];points = path['points']
        yaw = math.degrees(math.atan2(points[-1]['y']-points[0]['y'], points[-1]['x']-points[0]['x']))
        raw = next(a for a in preliminary_scene[actor['source_list']] if a['UNIQUEID'] == uid)
        require(angle_difference(yaw, raw['rot']['yaw']) <= .0011, 'Selected path and raw final yaw differ')
        change = path['ground_path_evidence']['terrain_selected_direction_change_deg']
        require(change in (0, 90, -90, 45, -45, 135, -135, 180), 'Unknown bounded motion direction')
        result[uid] = dict(yaw_deg=yaw, direction_change_deg=change,
            original_target=copy.deepcopy(actor['motion_path']['target_position']))
    return result


def write_direction_reference(source, preliminary, output):
    """최종 yaw만 채택한다. 이미 보정된 preview Z를 original pivot에 섞지 않는다."""
    output = Path(output)
    require(not output.exists(), 'Fresh final-yaw reference output required')
    _, _, original_meta, _ = single(source)
    _, _, preview_meta, preview_scene = single(preliminary)
    yaws = selected_yaws(original_meta, preview_meta, preview_scene)
    base = ground.static_reference(Path(source), output/'original_static')
    deck, entry, meta, scene = single(base)
    for actor in meta['runtime_actors']:
        if actor['unique_id'] not in yaws:
            continue
        chosen = yaws[actor['unique_id']];target = chosen['original_target']
        raw = next(a for a in scene[actor['source_list']] if a['UNIQUEID'] == actor['unique_id'])
        ground.gen.geo.set_actor_world_position(raw, target, chosen['yaw_deg'])
        actor['position'] = copy.deepcopy(target)
        actor['yaw'] = chosen['yaw_deg']
        require(actor['position'] == target and angle_difference(raw['rot']['yaw'], actor['yaw']) <= .0011,
            'Final reference original pivot/meta/raw yaw differs')
        require(all(abs(float(raw['pos'][axis])-target[axis]) <= .00051 for axis in 'xyz'), 'Serialized reference pivot differs')
    meta['final_direction_reference'] = dict(policy=POLICY, original_source_sha256=sha256(source),
        preliminary_manifest_sha256=sha256(preliminary), selected_yaws={str(k):v for k,v in yaws.items()},
        original_target_xyz_restored=True, native_or_gt_modified=False, calibration_only=True)
    sid = entry['id']+'_finaldirection';sp=output/'scenario.json';mp=output/'metadata.json'
    atomic_json(sp, scene);atomic_json(mp, meta)
    entry.update(id=sid, scenario=str(sp.resolve()), metadata=str(mp.resolve()), scenario_sha256=sha256(sp))
    deck.update(scenes=[entry], count=1, status='FINAL_DIRECTION_ORIGINAL_PIVOT_CALIBRATION_ONLY')
    path=output/'manifest.json';atomic_json(path, deck)
    return path


def envelope_points(corners, margin):
    require(math.isfinite(margin) and 1. <= margin <= 1.4+1e-9, 'Bounded preregistered envelope margin required')
    polygon = MultiPoint(np.asarray(corners, float)[:, :2]).convex_hull.buffer(margin, resolution=16)
    return np.asarray(polygon.exterior.coords, float)


def expand_compound_support(prepared_manifest, reference, terrain_sessions, output):
    """더 큰 support를 요구해서 기존 surface_at의 3/5/8m 순서를 그대로 사용한다."""
    output=Path(output);require(not output.exists(), 'Fresh support-envelope deck required')
    deck, entry, meta, scene = single(prepared_manifest)
    evidence = calibration_session(reference)
    by_uid = corners_by_uid(evidence['root'])
    compound = {uid:np.concatenate(values) for uid,values in by_uid.items() if len(values) > 1}
    clouds=[];terrain_evidence=[]
    for session in terrain_sessions:
        checked=calibration_session(session);root=Path(checked['root'])
        tm=read(root/'metadata.json');require(not tm.get('runtime_actors'), 'Independent terrain must have no actors')
        clouds.append(world_cloud(root,tm,read(root/'telemetry.json')));terrain_evidence.append(checked)
    cloud=np.concatenate(clouds);changes=[]
    for actor in meta['runtime_actors']:
        uid=actor['unique_id']
        if uid not in compound:
            continue
        tolerance = actor.get('motion_path',{}).get('target_tolerance_m',0.)
        require(type(tolerance) in (int,float) and math.isfinite(tolerance)
            and 0 <= tolerance <= MAXIMUM_TARGET_TOLERANCE_M, 'Arrival tolerance exceeds bounded envelope policy')
        corners=compound[uid];margin=EXTRA_SUPPORT_M+tolerance
        targets=envelope_points(corners,margin)
        target=actor.get('motion_path',{}).get('target_position',actor['position'])
        plane=surface_at(cloud,np.asarray([target['x'],target['y']]),support_xy=targets)
        if plane is None or not all(supported(plane,p) for p in targets):
            raise CompoundEnvelopeUnsupported(dict(uid=uid,reason='compound_support_envelope_terrain_unsupported',
                support_margin_m=margin,support_envelope_xy=targets.tolist(),terrain_sessions=list(map(str,terrain_sessions))))
        reference_row=next(r for r in meta['ground_probe_references'] if r['uid'] == uid)
        old=copy.deepcopy(reference_row['plane_world'])
        require(max(abs(plane_height(old,p)-plane_height(plane,p)) for p in corners[:,:2]) <= .10,
            'Wider independently fitted terrain disagrees with original support level')
        gap=min(corners[:,2]-np.asarray([plane_height(plane,p) for p in corners[:,:2]]))
        if not -.30 <= gap <= .75:
            raise ValueError('Static placement too far from ground; review')
        reference_row['plane_world']=plane
        changes.append(dict(uid=uid, original_plane=old, plane_world=copy.deepcopy(plane),
            static_native_component_count=len(by_uid[uid]), actual_native_corner_count=len(corners),
            target_tolerance_m=tolerance, extra_support_m=EXTRA_SUPPORT_M, support_margin_m=margin,
            support_envelope_xy=targets.tolist(), static_lowest_ground_gap_m=float(gap)))
    require(changes, 'Compound support adapter requires actual multi-component native reference')
    evidence_pins={}
    for item in [evidence,*terrain_evidence]:
        evidence_pins.update({path:record['sha256'] for path,record in item['inventory'].items()})
    marker=dict(policy=POLICY, changes=changes, input_hashes=evidence_pins,
        source_prepared_manifest=str(Path(prepared_manifest).resolve()),source_prepared_manifest_sha256=sha256(prepared_manifest),
        strict_surface_at_radii_m=[3.,5.,8.], support_extrapolation_limit_m=.75,
        original_runtime_ground_qa_unchanged=True, raw_native_or_gt_modified=False)
    meta['compound_support_envelope']=marker
    for actor in ground.moving_actors(meta):
        files=actor['motion_path']['ground_path_evidence']['files']
        known={v['path'] for v in files}
        files.extend(dict(path=p,sha256=h) for p,h in evidence_pins.items() if p not in known)
    ground.validate_motion_ground(meta,scene)
    output.mkdir(parents=True)
    sp=output/'scenario.json';mp=output/'metadata.json'
    atomic_json(sp,scene);atomic_json(mp,meta)
    require(sha256(sp)==entry['scenario_sha256'], 'Support evidence cannot change moving payload')
    entry.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()))
    deck.update(scenes=[entry],status='COMPOUND_ENVELOPE_PREPARED; UNCHANGED_NATIVE_QA_REQUIRED')
    path=output/'manifest.json';atomic_json(path,deck)
    verify_hashes(evidence_pins)
    return path


def verify_record(path):
    record=read(path)
    require(record.get('schema')==SCHEMA and record.get('policy')==POLICY, 'Direction record schema changed')
    verify_hashes(record['input_hashes']);verify_hashes(record['runtime_hashes'])
    for root,before in record.get('session_inventories',{}).items():
        require(file_inventory(root)==before, 'Direction native calibration inventory changed')
    if record.get('output_hashes'):
        pins.verify_hashes(record['output_hashes'])
    return record


def record(path, value, contract):
    """삭제된 recipe 전체도 resume에서 잡히도록 append-only 색인을 별도로 보관한다."""
    path=Path(path).resolve();output=Path(contract['capture_output'])
    require(output in path.parents and not path.exists(),'New campaign-local record required')
    atomic_json(path,value)
    index=output/RECORDS;document=read(index)
    require(document['contract_sha256']==digest(contract),'Direction record index belongs to another campaign')
    require(str(path) not in document['records'],'Direction record cannot be rewritten')
    document['records'][str(path)]=sha256(path)
    atomic_json(index,document)


def direction_prepare(source,terrain,reference,output,additional_terrain=None,*,
                      prepared_callback,capture_callback,cell,contract):
    verify_hashes(contract['runtime_hashes'])
    source,terrain,reference,output=map(Path,(source,terrain,reference,output))
    sessions=[terrain,*list(additional_terrain or [])]
    require(len(sessions)<=ground.MAX_TERRAIN_SESSIONS, 'Existing terrain witness bound exceeded')
    ref_evidence=calibration_session(reference)
    compound={uid for uid,values in corners_by_uid(ref_evidence['root']).items() if len(values)>1}
    if not compound:
        return ground.prepare(source,terrain,reference,output,additional_terrain=additional_terrain)
    _,_,original_meta,_=single(source)
    snapshots={str(reference.resolve()):ref_evidence['inventory']}
    for session in sessions:
        snapshots[str(session.resolve())]=calibration_session(session)['inventory']
    recipe=dict(source=str(source.resolve()),source_sha256=sha256(source),source_inputs=input_pins(source),
        terrain_sessions=[str(p.resolve()) for p in sessions],reference_session=str(reference.resolve()),
        session_inventories=snapshots,extra_support_m=EXTRA_SUPPORT_M,
        maximum_target_tolerance_m=MAXIMUM_TARGET_TOLERANCE_M)
    container=output.parent/'direction_reference_preparations'/digest(recipe)[:24]
    registration=container/'recipe.json'
    if container.exists():
        saved=verify_record(registration);require(saved['recipe']==recipe, 'Direction recipe cache mismatch')
    else:
        container.mkdir(parents=True)
        record(registration,dict(schema=SCHEMA,policy=POLICY,recipe=recipe,
            input_hashes=recipe['source_inputs'],runtime_hashes=contract['runtime_hashes'],
            session_inventories=snapshots,calibration_only=True,bulk_ready=False),contract)
    preliminary=container/'preliminary'/'manifest.json'
    preview_receipt=container/'preliminary_receipt.json'
    if preliminary.exists():
        require(preview_receipt.exists(),'Incomplete preview cache; use fresh campaign')
        verify_record(preview_receipt)
    else:
        preliminary=ground.prepare(source,terrain,reference,preliminary.parent,additional_terrain=additional_terrain)
        record(preview_receipt,dict(schema=SCHEMA,policy=POLICY,input_hashes=recipe['source_inputs'],
            runtime_hashes=contract['runtime_hashes'],session_inventories=snapshots,
            output_hashes={p:r['sha256'] for p,r in file_inventory(preliminary.parent).items()}),contract)
    _,_,preview_meta,preview_scene=single(preliminary)
    yaws=selected_yaws(original_meta,preview_meta,preview_scene)
    changed=any(uid in compound and choice['direction_change_deg']!=0 for uid,choice in yaws.items())
    final_reference=reference
    if changed:
        reference_recipe=dict(source_sha256=sha256(source),source_inputs=recipe['source_inputs'],
            selected_yaws={str(k):v for k,v in yaws.items()},policy=POLICY)
        bank=output.parent/'final_direction_references'/digest(reference_recipe)[:24]
        bank_receipt=bank/'reference_receipt.json'
        if bank_receipt.exists():
            cached=verify_record(bank_receipt)
            require(cached['reference_recipe']==reference_recipe,'Final direction cache belongs to another source/pivot/yaw')
            final_reference=Path(cached['reference_session'])
            calibration_session(final_reference,Path(cached['reference_manifest']))
        else:
            require(not bank.exists(),'Incomplete final-direction reference; fresh campaign required')
            bank.mkdir(parents=True)
            raw=bank/'deck'/'manifest.json'
            matched=prepared_callback(raw,lambda:write_direction_reference(source,preliminary,raw.parent))
            result=capture_callback(matched,bank/'native',cell,'motion_final_direction_static_reference')
            require(quality.scene_rejection(result) is None,'Final-direction calibration contact/support failed')
            native_evidence=calibration_session(bank/'native',matched)
            final_reference=bank/'native'
            record(bank_receipt,dict(schema=SCHEMA,policy=POLICY,reference_recipe=reference_recipe,
                input_hashes=dict(recipe['source_inputs'],**{str(preliminary.resolve()):sha256(preliminary)}),
                runtime_hashes=contract['runtime_hashes'],reference_manifest=str(Path(matched).resolve()),
                reference_session=str(final_reference.resolve()),
                session_inventories={str(final_reference.resolve()):native_evidence['inventory']},
                output_hashes={p:r['sha256'] for p,r in file_inventory(bank).items()}),contract)
    pivot=container/'final_pivot'/'manifest.json'
    pivot_receipt=container/'final_pivot_receipt.json'
    if pivot.exists():
        require(pivot_receipt.exists(),'Incomplete pivot preparation; fresh campaign required')
        verify_record(pivot_receipt)
    else:
        pivot=ground.prepare(source,terrain,final_reference,pivot.parent,additional_terrain=additional_terrain)
        _,_,final_meta,final_scene=single(pivot)
        require(selected_yaws(original_meta,final_meta,final_scene)==yaws,'Final preparation changed selected direction/pivot recipe')
        record(pivot_receipt,dict(schema=SCHEMA,policy=POLICY,input_hashes=recipe['source_inputs'],
            runtime_hashes=contract['runtime_hashes'],session_inventories={**snapshots,
                str(final_reference.resolve()):calibration_session(final_reference)['inventory']},
            output_hashes={p:r['sha256'] for p,r in file_inventory(pivot.parent).items()}),contract)
    try:
        result=expand_compound_support(pivot,final_reference,sessions,output)
    except CompoundEnvelopeUnsupported as error:
        deficit=container/'envelope_deficit_receipt.json'
        if not deficit.exists():
            record(deficit,dict(schema=SCHEMA,policy=POLICY,input_hashes=recipe['source_inputs'],
                runtime_hashes=contract['runtime_hashes'],session_inventories=snapshots,
                details=error.details,existing_near_probe_recovery_retained=True),contract)
        else:
            verify_record(deficit)
        raise
    final_receipt=container/'final_receipt.json'
    record(final_receipt,dict(schema=SCHEMA,policy=POLICY,input_hashes=recipe['source_inputs'],
        runtime_hashes=contract['runtime_hashes'],session_inventories={**snapshots,
            str(final_reference.resolve()):calibration_session(final_reference)['inventory']},
        output_hashes={p:r['sha256'] for p,r in file_inventory(output).items()},
        final_manifest=str(result.resolve()),native_qa_unchanged=True,bulk_ready=False),contract)
    verify_record(final_receipt)
    return result


def compiled_execute(contract,source=SOURCE):
    require(sha256(source)==contract['quality_source_sha256'],'Frozen quality source changed')
    tree=ast.parse(Path(source).read_text());functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute']
    require(len(functions)==1,'Exactly one frozen quality execute required')
    count=0
    class Adapter(ast.NodeTransformer):
        def visit_Call(self,node):
            nonlocal count
            self.generic_visit(node)
            if isinstance(node.func,ast.Attribute) and isinstance(node.func.value,ast.Name) and node.func.value.id=='motion_ground' and node.func.attr=='prepare':
                require(len(node.args)==4 and all(k.arg=='additional_terrain' for k in node.keywords),'Frozen prepare call signature changed')
                count+=1;node.func=ast.Name(id='_direction_prepare',ctx=ast.Load())
                node.keywords.extend(ast.keyword(arg=name,value=ast.Name(id=value,ctx=ast.Load()))
                    for name,value in [('prepared_callback','prepared'),('capture_callback','capture'),('cell','cell')])
            return node
    function=Adapter().visit(copy.deepcopy(functions[0]))
    require(count==3,'Exactly three original prepare hooks required; found '+str(count))
    namespace=dict(quality.__dict__)
    namespace['_direction_prepare']=lambda *a,**kw:direction_prepare(*a,**kw,contract=contract)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),str(source),'exec'),namespace)
    require(sha256(source)==contract['quality_source_sha256'],'Quality source changed during AST compilation')
    return namespace['execute']


@contextmanager
def installed(contract):
    original=quality.execute
    quality.execute=compiled_execute(contract)
    try:
        yield
    finally:
        quality.execute=original


def configuration(manifest,indices,rounds,seed=None):
    return dict(schema=SCHEMA,policy=POLICY,manifest=str(Path(manifest).resolve()),
        manifest_sha256=sha256(manifest),indices=list(indices),quota_rounds=rounds,
        seed_capture=None if seed is None else dict(path=str(Path(seed).resolve()),sha256=sha256(seed)),
        quality_source_sha256=sha256(SOURCE),runtime_hashes=runtime_hashes(),input_hashes=input_pins(manifest),
        original_spatial_campaign=campaign.contract(manifest,indices,rounds),maximum_cells=MAXIMUM_CELLS,
        extra_compound_support_m=EXTRA_SUPPORT_M,maximum_target_tolerance_m=MAXIMUM_TARGET_TOLERANCE_M,
        original_native_qa_unchanged=True,bulk_ready=False,bulk_authorized=False)


def verify_campaign(output,manifest,indices=None):
    """종료/partial/completed resume 공통 read-only 검사. 실패는 RuntimeError다."""
    output=Path(output).resolve(strict=True);contract=read(output/GUARD)
    require(contract.get('schema')==SCHEMA and contract.get('policy')==POLICY
        and contract.get('capture_output')==str(output)
        and contract.get('manifest')==str(Path(manifest).resolve())
        and contract.get('manifest_sha256')==sha256(manifest),'Direction campaign source/profile changed')
    if indices is not None:
        require(contract['indices']==list(indices),'Direction campaign indices changed')
    require(contract.get('bulk_ready') is False and contract.get('bulk_authorized') is False,'Prototype direction adapter cannot claim bulk authorization')
    require(contract.get('quality_source_sha256')==sha256(SOURCE)
        and contract.get('maximum_cells')==MAXIMUM_CELLS
        and contract.get('extra_compound_support_m')==EXTRA_SUPPORT_M
        and contract.get('maximum_target_tolerance_m')==MAXIMUM_TARGET_TOLERANCE_M
        and contract.get('original_native_qa_unchanged') is True,'Direction preparation recipe/profile changed')
    seed=contract.get('seed_capture')
    if seed is not None:
        require(isinstance(seed,dict) and sha256(seed['path'])==seed['sha256'],'Preserved seed capture changed')
    require(contract['runtime_hashes']==runtime_hashes(),'Direction runtime closure changed')
    verify_hashes(contract['runtime_hashes']);verify_hashes(contract['input_hashes'])
    require(contract['original_spatial_campaign']==campaign.contract(manifest,contract['indices'],contract['quota_rounds']),
        'Original spatial/campaign/source contract changed')
    index=read(output/RECORDS)
    require(index.get('schema')==SCHEMA and index.get('contract_sha256')==digest(contract),'Direction recipe index changed')
    observed={}
    for name in ('recipe.json','preliminary_receipt.json','final_pivot_receipt.json','final_receipt.json','reference_receipt.json','envelope_deficit_receipt.json'):
        for path in output.rglob(name):
            if 'direction_reference_preparations' in path.parts or 'final_direction_references' in path.parts:
                verify_record(path);observed[str(path.resolve())]=sha256(path)
    require(observed==index.get('records'),'Direction recipe deleted, replaced or never durably indexed')
    return dict(passed=True,policy=POLICY,verified_records=len(observed),bulk_ready=False,native_qa_unchanged=True)


def verify_completed_campaign(source_manifest,index,campaign_root):
    """최종 승격기가 사용하는 실제 단일 quota 증거 API. 기존 review는 고치지 않는다."""
    root=Path(campaign_root).resolve(strict=True)
    verify_campaign(root,source_manifest)
    registered=read(root/GUARD)['indices']
    require(index in registered,'Requested completed proof is outside its exact campaign')
    source=read(source_manifest);original=source['scenes'][index];old=read(original['metadata'])
    require(read(root/'status.json').get('stage')=='complete' and read(root/'quota.json').get('accepted')==len(registered)
        and read(root/'quota.json').get('deficit')==0,'One completed approved quota required')
    selection=root/'selected_ledger.jsonl';payload=selection.read_bytes()
    require(payload and payload.endswith(b'\n'),'Complete selected ledger required')
    rows=[json.loads(line) for line in payload.splitlines() if line.strip()]
    require(len(rows)==len(registered) and len({r.get('source_cell_id',r['id']) for r in rows})==len(rows),
        'One unique native selection per exact campaign quota required')
    matched=[r for r in rows if r.get('source_cell_id',r['id'])==original['id']]
    require(len(matched)==1,'Exactly one selected native observation of requested source required')
    row=matched[0];raw=Path(row['capture_root']).resolve(strict=True);session=raw.parents[1]
    require(row.get('status')=='accepted_candidate' and row.get('source_cell_id',row['id'])==original['id']
        and row.get('audit',{}).get('accepted_candidate') is True and not row['audit'].get('review_reasons'),
        'Selected moving capture is not an approval of the explicit source quota')
    require(root in raw.parents,'Selected native observation escaped its campaign')
    snapshot=file_inventory(session)
    contract=read(session/'session_contract.json')
    manifests=[p for p in session.parent.rglob('manifest.json') if sha256(p)==contract['manifest_sha256']]
    require(manifests,'Actual moving launch manifest missing')
    prepared=manifests[0];deck,entry,meta,scene=single(prepared)
    require(entry['id']==row['id'] and entry['scenario_sha256']==row['scenario_sha256']
        and sha256(raw/'scenario.json')==entry['scenario_sha256'] and read(raw/'audit.json')==row['audit'],
        'Selected native payload/audit/launch differs')
    from finalize_base5_layout_split import planned_fields_match
    require(planned_fields_match(meta,read(raw/'metadata.json')),'Selected native metadata differs from its exact launch')
    identity=lambda m:[tuple(a.get(k) for k in ('unique_id','data_id','name','source_list','motion')) for a in m['runtime_actors']]
    require(meta['spec']==old['spec'] and meta['split']==old['split']==original['split']
        and identity(meta)==identity(old)
        and meta.get('native_unit_geometry_reference')==old.get('native_unit_geometry_reference')
        and meta.get('fresh_environment_provenance')==old.get('fresh_environment_provenance'),
        'Selected native changed original spec/ordered actor/unit/environment quota')
    require(not meta.get('calibration_only'),'Calibration cannot fill a training quota')
    ground.validate_motion_ground(meta,scene)
    unit_capture.verify_capture(entry,raw,deck,row)
    require(read(session/'shutdown_safety.json').get('park_verified') is True,'Selected final Park is unverified')
    evidence=read(raw/'collision_history.json');summary=evidence['summary']
    window=collision_window(evidence['samples'],summary['window_start_monotonic'],summary['window_end_monotonic'])
    require(window['observed'] and window['no_collision_verified'] and summary.get('no_collision_verified') is True,
        'Selected collision evidence is incomplete')
    for report in (root/original['id']).glob('round_*/report.json'):
        data=read(report);require(data.get('stage')=='complete','Incomplete preserved quality round')
        campaign.validate_selected(data)
    require(file_inventory(session)==snapshot,'Selected native session changed during verification')
    result={str(Path(source_manifest).resolve()):sha256(source_manifest),str(selection):sha256(selection)}
    for p,recorded in file_inventory(root).items():
        result[p]=recorded['sha256']
    result.update(input_pins(prepared));result.update(runtime_hashes())
    verify_hashes(result)
    return dict(passed=True,source_id=original['id'],prepared_manifest=str(prepared),
        selected_ledger=str(selection),accepted_row=row,input_hashes=result,
        native_qa_unchanged=True,bulk_ready=False,policy=POLICY)


def run(args):
    require(args.enable_final_direction_reference,'Explicit --enable-final-direction-reference is required')
    require(not args.approve_bulk_capture,'Direction reference prototype cannot authorize bulk')
    deck=read(args.manifest)
    indices=[int(v) for v in args.indices.split(',')] if args.indices else list(range(min(args.limit,len(deck['scenes']))))
    require(1<=len(indices)<=MAXIMUM_CELLS and len(set(indices))==len(indices)
        and all(0<=i<len(deck['scenes']) for i in indices),'Direction prototype requires 1..4 unique valid cells')
    require(1<=args.quota_rounds<=3,'Keep original 1..3 quota rounds')
    output=Path(args.output).resolve();manifest=Path(args.manifest).resolve(strict=True)
    require(output!=manifest.parent and manifest.parent not in output.parents,'Output cannot be inside immutable source')
    runtime=ground.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    require(output!=runtime and runtime not in output.parents,'Output cannot be inside simulator runtime')
    require(not output.exists() or args.resume,'Fresh output or explicit resume required')
    expected=configuration(manifest,indices,args.quota_rounds,args.seed_capture)
    expected['capture_output']=str(output)
    if output.exists():
        require((output/GUARD).is_file(),'Cannot attach direction adapter to legacy campaign')
        require(read(output/GUARD)==expected,'Direction resume contract differs')
        verify_campaign(output,manifest,indices)
    else:
        output.mkdir(parents=True,exist_ok=False);atomic_json(output/GUARD,expected)
        atomic_json(output/RECORDS,dict(schema=SCHEMA,contract_sha256=digest(expected),records={}))
    with (output/'.direction_reference.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            with installed(expected):
                result=spatial.run(argparse.Namespace(manifest=manifest,output=output,indices=','.join(map(str,indices)),
                    limit=len(indices),resume=True,quota_rounds=args.quota_rounds,
                    approve_bulk_capture=False,seed_capture=args.seed_capture))
            verify_campaign(output,manifest,indices)
            return result
        finally:
            verify_hashes(expected['runtime_hashes']);verify_hashes(expected['input_hashes'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enable-final-direction-reference',action='store_true')
    parser.add_argument('--manifest',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--indices');parser.add_argument('--limit',type=int,default=1)
    parser.add_argument('--resume',action='store_true');parser.add_argument('--quota-rounds',type=int,default=2)
    parser.add_argument('--seed-capture',type=Path);parser.add_argument('--approve-bulk-capture',action='store_true')
    raise SystemExit(run(parser.parse_args()))


if __name__=='__main__':
    main()
