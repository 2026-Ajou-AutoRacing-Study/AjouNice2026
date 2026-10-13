"""Transfer a verified nearby terrain witness for the exact same static placement.

Full relative rotation comes from the same rigid annotation, not yaw-only Ego
poses. This is capture QA, never a model input or rewritten training annotation.
"""
import hashlib
import json
import math
import copy
import os
from pathlib import Path
import numpy as np
from base5_capture_safety import sha256,atomic_json


class ReferenceQualityError(ValueError):
    """Observed placement/view mismatch: quality deficit, not damaged evidence."""


COMPOSITE_ANCHORS={'BabyStroller1':'Obstacle','BabyStroller2':'Obstacle',
    'BabyStroller3':'Obstacle','Bike3':'Vehicle','Scooter1':'Vehicle'}


def select_anchor(boxes,actor):
    if len(boxes)==1:return boxes[0]
    label=COMPOSITE_ANCHORS.get(actor.get('name'))
    candidates=[b for b in boxes if label and b['raw_class']==label]
    if len(candidates)!=1:raise ReferenceQualityError('No unique known rigid component anchor')
    return candidates[0]


def validate_components(near,far,nt,ft,R,t):
    """Match by class/size/geometry, never Instance row order or parent collapse."""
    from base5_capture_audit import rotation_matrix
    if len(near)!=len(far):raise ReferenceQualityError('Reference component count changed')
    remaining=list(near);matched=[]
    for b in far:
        fc=(ft@np.r_[b['center'],1.])[:3];fr=ft[:3,:3]@rotation_matrix(b['rotation_rpy'])
        candidates=[]
        for i,a in enumerate(remaining):
            if a['raw_class']!=b['raw_class'] or not np.allclose(a['size'],b['size'],atol=.001,rtol=0):continue
            nc=(nt@np.r_[a['center'],1.])[:3];nr=nt[:3,:3]@rotation_matrix(a['rotation_rpy'])
            error=float(np.linalg.norm(nc-(R@fc+t)))
            angle=math.degrees(math.acos(float(np.clip((np.trace(nr.T@R@fr)-1)/2,-1,1))))
            if error<=.02 and angle<=1.:candidates.append((i,error,angle))
        if len(candidates)!=1:raise ReferenceQualityError('Component pose/size correspondence changed or ambiguous')
        i,error,angle=candidates[0];remaining.pop(i)
        matched.append(dict(raw_class=b['raw_class'],center_error_m=error,rotation_error_deg=angle))
    return matched


def actor_payload(scene,container,uid):
    actor=next(a for a in scene[container] if a['UNIQUEID']==uid)
    return hashlib.sha256(json.dumps(actor,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def object_status(objects,uid):
    return next(a for k in ('npc_list','pedestrian_list','obstacle_list') for a in objects.get(k,[]) if a['unique_id']==uid)


def matching_status(a,b):
    if a['type']!=b['type']:raise ReferenceQualityError('Reference object type changed')
    if np.linalg.norm([a['position'][k]-b['position'][k] for k in ('x','y','z')])>.01:
        raise ReferenceQualityError('Reference physical position changed')
    if abs((a['heading']-b['heading']+180)%360-180)>.05:raise ReferenceQualityError('Reference heading changed')
    if max(abs(a['size'][k]-b['size'][k]) for k in ('x','y','z'))>.001:raise ReferenceQualityError('Reference size changed')
    if max(np.linalg.norm([s['velocity'][k] for k in ('x','y','z')]) for s in (a,b))>.05:
        raise ReferenceQualityError('Reference object not stationary')


def prepare(source,reference_session,output):
    if output.exists():raise FileExistsError('New static reference retry output required')
    safety=json.loads((reference_session/'shutdown_safety.json').read_text())
    if not safety.get('park_verified'):raise ValueError('Reference Park unverified')
    row=json.loads((reference_session/'ledger.jsonl').read_text().splitlines()[-1])
    if row['status']!='accepted_candidate':raise ValueError('Nearby reference not accepted')
    root=Path(row['capture_root']);meta=json.loads((root/'metadata.json').read_text())
    if not meta.get('calibration_only'):raise ValueError('Reference must be calibration-only')
    audit=json.loads((root/'audit.json').read_text())
    if audit.get('review_reasons'):raise ValueError('Reference audit has deficits')
    deck=json.loads(source.read_text());entry=deck['scenes'][0]
    if meta['static_height_reference']['source_scenario_sha256']!=entry['scenario_sha256']:
        raise ValueError('Nearby reference source mismatch')
    current=json.loads(Path(entry['metadata']).read_text());witnesses=[]
    for actor in current['runtime_actors']:
        if actor['motion'] not in ('static','standing'):raise ValueError('Moving reference prohibited')
        g=next((a for a in audit['ground_support']['actors'] if a['uid']==actor['unique_id']),None)
        if not g or not g['verified'] or not g.get('plane'):continue
        from audit_morai_lidar_gt_visibility import parse_instances
        components=[b for b in parse_instances(next(root.glob('LIDAR_*/*_Instance.txt'))) if b['unique_id']==actor['unique_id']]
        try:select_anchor(components,actor)
        except ReferenceQualityError:continue
        files=[root/'metadata.json',root/'telemetry.json',root/'scenario.json',root/'audit.json',root/'collision_history.json',
            reference_session/'shutdown_safety.json',next(root.glob('LIDAR_*/*_Instance.txt')),
            next(root.glob('LIDAR_*/*_Intensity.bin'))]
        witnesses.append(dict(uid=actor['unique_id'],capture_root=str(root),
            sha256={str(p):sha256(p) for p in files}))
    if not witnesses:raise ValueError('Reference has no verified local plane')
    updated={w['uid']:w for w in current.get('static_ground_references',[])}
    updated.update({w['uid']:w for w in witnesses})
    current['static_ground_references']=list(updated.values());sid=entry['id']+'_rigidground';output.mkdir(parents=True)
    sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
    atomic_json(sp,json.loads(Path(entry['scenario']).read_text()));atomic_json(mp,current)
    deck.update(count=1,scenes=[dict(entry,id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp))])
    atomic_json(output/'manifest.json',deck);return output/'manifest.json'


def transfer_plane(near_box,far_box,near_transform,far_transform,plane):
    from base5_capture_audit import rotation_matrix
    nt=np.asarray(near_transform);ft=np.asarray(far_transform)
    R=(nt[:3,:3]@rotation_matrix(near_box['rotation_rpy']))@(ft[:3,:3]@rotation_matrix(far_box['rotation_rpy'])).T
    nc=(nt@np.r_[near_box['center'],1.])[:3];fc=(ft@np.r_[far_box['center'],1.])[:3]
    t=nc-R@fc;n=np.array([*plane['coefficients'][:2],-1.]);d=plane['coefficients'][2]
    normal=R.T@n;constant=n@t+d
    if abs(normal[2])<.8:raise ValueError('Transferred terrain not approximately horizontal')
    coefficients=[-normal[0]/normal[2],-normal[1]/normal[2],-constant/normal[2]]
    polygon=np.asarray(plane['support_polygon_xy']);z=polygon@np.asarray(plane['coefficients'][:2])+d
    points=(np.c_[polygon,z]-t)@R
    return dict(coefficients=coefficients,support_polygon_xy=points[:,:2].tolist()),R,t


def validate_view_transform(R,t,near_ego,far_ego):
    expected_yaw=far_ego['heading']-near_ego['heading']
    measured_yaw=math.degrees(math.atan2(R[1,0],R[0,0]))
    if abs((measured_yaw-expected_yaw+180)%360-180)>.2:
        raise ReferenceQualityError('Rigid view transform conflicts with Ego headings')
    yaw=math.radians(near_ego['heading']-1.347)
    world=np.array([far_ego[k]-near_ego[k] for k in ('x','y','z')])
    expected=np.array([[math.cos(yaw),math.sin(yaw),0],[-math.sin(yaw),math.cos(yaw),0],[0,0,1]])@world
    if np.linalg.norm(t[:2]-expected[:2])>.25:
        raise ReferenceQualityError('Rigid view transform conflicts with Ego horizontal position')
    if abs(R[2,2])<math.cos(math.radians(3.)):
        raise ReferenceQualityError('Rigid view transform has excessive tilt')


def apply_reference(ground,boxes,root,metadata):
    from audit_morai_lidar_gt_visibility import parse_instances
    from base5_capture_audit import rotation_matrix
    from base5_ground_probe import supported,plane_height
    for witness in metadata.get('static_ground_references',[]):
        uid=witness['uid'];g=next((a for a in ground['actors'] if a['uid']==uid),None)
        if not g or g.get('reason')!='actor_ground_plane_unverified':continue
        for path,digest in witness['sha256'].items():
            if sha256(Path(path))!=digest:raise RuntimeError('Static ground witness changed')
        nr=Path(witness['capture_root']);nm=json.loads((nr/'metadata.json').read_text())
        actor=next(a for a in metadata['runtime_actors'] if a['unique_id']==uid)
        if actor['motion'] not in ('static','standing') or not nm.get('calibration_only'):
            raise ValueError('Reference must be a static calibration')
        container=actor['source_list']
        if actor_payload(json.loads((root/'scenario.json').read_text()),container,uid)!=actor_payload(json.loads((nr/'scenario.json').read_text()),container,uid):
            raise ValueError('Reference actor payload changed')
        tele=json.loads((nr/'telemetry.json').read_text())
        matching_status(object_status(tele['objects_before'],uid),object_status(tele['objects_after'],uid))
        for objects in metadata['capture_runtime_objects']:
            matching_status(object_status(tele['objects_before'],uid),object_status(objects,uid))
        nb=[b for b in parse_instances(next(nr.glob('LIDAR_*/*_Instance.txt'))) if b['unique_id']==uid]
        fb=[b for b in boxes if b['unique_id']==uid]
        na=select_anchor(nb,actor);fa=select_anchor(fb,actor)
        if not np.allclose(na['size'],fa['size'],atol=.001,rtol=0):raise ReferenceQualityError('Rigid annotation size changed')
        ng=next(a for a in json.loads((nr/'audit.json').read_text())['ground_support']['actors'] if a['uid']==uid)
        if not ng['verified'] or not ng.get('plane'):raise ValueError('Reference local plane unverified')
        plane,R,t=transfer_plane(na,fa,nm['lidar_to_ego'],metadata['lidar_to_ego'],ng['plane'])
        validate_view_transform(R,t,tele['ego_before'],metadata['capture_runtime_ego'])
        matched=validate_components(nb,fb,np.asarray(nm['lidar_to_ego']),np.asarray(metadata['lidar_to_ego']),R,t)
        signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        corners=np.concatenate([signs*b['size']/2@rotation_matrix(b['rotation_rpy']).T+b['center'] for b in fb])
        corners=(np.c_[corners,np.ones(len(corners))]@np.asarray(metadata['lidar_to_ego']).T)[:,:3]
        if not all(supported(plane,c[:2]) for c in corners):raise ReferenceQualityError('Transferred footprint outside observed terrain')
        gap=float(np.min(corners[:,2]-[plane_height(plane,c[:2]) for c in corners]))
        if not -.30<=gap<=ground['maximum_gap_m']:raise ReferenceQualityError('Transferred ground gap outside bounds')
        g.update(verified=True,reason=None,minimum_annotation_ground_gap_m=gap,suggested_z_shift_m=0.,
            static_rigid_reference=dict(capture_root=str(nr),rotation=R.tolist(),translation=t.tolist(),
                plane=plane,anchor_raw_class=fa['raw_class'],component_matches=matched,
                policy='exact-static-payload-and-observed-pose; full-relative-rigid-component-rotation',
                limitation='Same static placement QA only; not Ego attitude measurement or mesh/collider certification'))
    ground['review_reasons']=sorted({a['reason'] for a in ground['actors'] if a.get('required') and a.get('reason')})


def validate_reaudit_output(output):
    """QA views are not capture sessions; verify immutable raw/proofs directly."""
    from base5_capture_audit import audit_capture
    meta=json.loads((output/'metadata.json').read_text());proof=meta['static_ground_reaudit']
    source=Path(proof['source_capture_root']);result=json.loads((output/'reaudit_result.json').read_text())
    if result['raw_source_capture']!=str(source):raise RuntimeError('Re-audit source changed')
    if json.loads((source/'metadata.json').read_text()).get('static_ground_reaudit'):
        validate_reaudit_output(source)
    for name,key in (('metadata.json','source_metadata_sha256'),('audit.json','source_audit_sha256')):
        if sha256(source/name)!=proof[key]:raise RuntimeError('Re-audit original proof changed')
    if sha256(source/'scenario.json')!=result['scenario_sha256']:raise RuntimeError('Re-audit source scenario changed')
    for p,digest in proof['raw_sha256'].items():
        if sha256(output/p)!=digest or sha256(source/p)!=digest:raise RuntimeError('Re-audit raw changed')
    audit=json.loads(json.dumps(audit_capture(output,meta)))
    if audit!=result['audit'] or json.loads((output/'audit.json').read_text())!=audit:
        raise RuntimeError('Re-audit verification changed')
    status='review' if audit['review_reasons'] else 'accepted_candidate'
    if result['status']!=status:raise RuntimeError('Re-audit status disagrees with QA')
    return result


def reaudit_original(row,manifest,output):
    """Reuse immutable sensor bytes; write a separate QA view, never overwrite raw."""
    if row['status']!='review' or row['audit']['review_reasons']!=['actor_ground_plane_unverified']:
        raise ValueError('Only isolated missing-ground review may reuse raw')
    source=Path(row['capture_root']);entry=json.loads(manifest.read_text())['scenes'][0]
    if json.loads((source/'audit.json').read_text())!=row['audit']:
        raise RuntimeError('Original audit differs from authoritative row')
    if sha256(source/'scenario.json')!=entry['scenario_sha256']:raise RuntimeError('Re-audit actor/scenario payload changed')
    metadata=json.loads((source/'metadata.json').read_text())
    requested=json.loads(Path(entry['metadata']).read_text())
    metadata['static_ground_references']=requested['static_ground_references']
    if metadata.get('calibration_only'):raise ValueError('Calibration cannot fill training quota')
    telemetry=json.loads((source/'telemetry.json').read_text())
    metadata['capture_runtime_objects']=[telemetry['objects_before'],telemetry['objects_after']]
    metadata['capture_runtime_ego']=telemetry['ego_before']
    if output.exists():
        result=json.loads((output/'reaudit_result.json').read_text())
        stored=json.loads((output/'metadata.json').read_text())
        if (result['raw_source_capture']!=str(source) or result['scenario_sha256']!=row['scenario_sha256']
                or stored['static_ground_references']!=metadata['static_ground_references']):
            raise RuntimeError('Existing re-audit source/reference changed')
        for p,digest in stored['static_ground_reaudit']['raw_sha256'].items():
            if sha256(output/p)!=digest or sha256(source/p)!=digest:raise RuntimeError('Re-audit raw changed')
        from base5_capture_audit import audit_capture
        # JSON changes Shapely hull tuples to lists. Compare persisted values,
        # not Python container types; never weaken numeric/proof comparisons.
        actual=json.loads(json.dumps(audit_capture(output,stored)))
        if actual!=result['audit']:raise RuntimeError('Re-audit verification changed')
        return result
    metadata['static_ground_reaudit']=dict(source_capture_root=str(source),
        source_metadata_sha256=sha256(source/'metadata.json'),source_audit_sha256=sha256(source/'audit.json'),
        raw_sha256={str(p.relative_to(source)):sha256(p) for p in source.rglob('*')
            if p.is_file() and p.name not in ('metadata.json','audit.json','reaudit_result.json')},
        policy='same immutable sensor bytes; new proof/audit view; no second sensor capture')
    output.mkdir(parents=True)
    for p in source.rglob('*'):
        relative=p.relative_to(source)
        if not p.is_file() or relative in (Path('metadata.json'),Path('audit.json'),Path('reaudit_result.json')):continue
        dest=output/relative;dest.parent.mkdir(parents=True,exist_ok=True)
        os.link(p,dest)
    from base5_capture_audit import audit_capture
    # Ground hull coordinates may be tuples in memory but lists on disk.
    # Return the same canonical JSON value that the next chained QA verifies.
    audit=json.loads(json.dumps(audit_capture(output,metadata)))
    atomic_json(output/'metadata.json',metadata);atomic_json(output/'audit.json',audit)
    result=copy.deepcopy(row);result.update(id=entry['id'],capture_root=str(output.resolve()),audit=audit,
        status='accepted_candidate' if not audit['review_reasons'] else 'review',reused_sensor_capture=True,
        raw_source_capture=str(source),original_sensor_scenario_sha256=row['scenario_sha256'])
    # The scene bytes are unchanged; retain their original hash and telemetry.
    atomic_json(output/'reaudit_result.json',result)
    return result


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--reference-session',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(prepare(a.manifest,a.reference_session,a.output))
