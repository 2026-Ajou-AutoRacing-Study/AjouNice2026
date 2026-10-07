"""Independent terrain-supported motion paths. Input edits only; raw GT stays raw."""
import copy
import math
from pathlib import Path
import numpy as np
from base5_capture_safety import atomic_json, sha256
from base5_ground_probe import world_cloud, surface_at, supported, plane_height, ego_to_world
from base5_capture_audit import rotation_matrix
from audit_morai_lidar_gt_visibility import parse_instances
import generate_base5_independent as gen


def moving_actors(meta):
    return [a for a in meta.get('runtime_actors',[]) if a.get('motion') not in ('static','standing')]


def validate_motion_ground(meta,scene):
    """Refuse legacy unverified moving payloads before ScenarioLoad, not after capture."""
    for actor in moving_actors(meta):
        path=actor.get('motion_path',{})
        proof=path.get('ground_path_evidence',{})
        if proof.get('policy')!='independent-terrain-sampled-v46':
            raise ValueError('Dynamic path lacks independent ground preparation: '+actor['name'])
        for item in proof.get('files',[]):
            if sha256(Path(item['path']))!=item['sha256']:
                raise ValueError('Motion ground evidence checksum mismatch')
        if not proof.get('files') or len(proof.get('samples',[]))<2:
            raise ValueError('Incomplete motion ground witness')
        if len(path['points'])!=len(proof['samples']):raise ValueError('Motion sample count mismatch')
        for point,sample in zip(path['points'],proof['samples']):
            if (abs(point['x']-sample['x'])>1e-8 or abs(point['y']-sample['y'])>1e-8 or
                    abs(point['z']-sample['z']-proof['asset_pivot_offset_m'])>1e-8):
                raise ValueError('Motion point differs from witnessed ground height')
        raw=next(p for p in scene['waypointDataList'] if p['pathName'].endswith(' - '+str(actor['unique_id'])))
        expected=[gen.geo.world_to_morai_waypoint(p) for p in path['points']]
        if [p['pos'] for p in raw['waypointData']]!=expected:
            raise ValueError('Ground-prepared waypoint/metadata mismatch')
        if actor['position']!=path['points'][0]:
            raise ValueError('Ground-prepared departure mismatch')


def static_reference(source,output):
    """Same target XY/asset, speed zero; calibration only, never a training negative."""
    if output.exists():raise FileExistsError('New static reference directory required')
    m=gen.geo.load_json(source)
    if len(m['scenes'])!=1:raise ValueError('Static reference is single-cell')
    old=m['scenes'][0];scene=gen.geo.load_json(Path(old['scenario']));meta=gen.geo.load_json(Path(old['metadata']))
    for record in moving_actors(meta):
        position=copy.deepcopy(record['motion_path']['target_position'])
        actor=next(a for a in scene[record['source_list']] if a['UNIQUEID']==record['unique_id'])
        gen.geo.set_actor_world_position(actor,position,record.get('yaw',0.) if 'yaw' in record else float(actor['rot']['yaw']))
        actor.update(isWaypointMode=False,waypointEventList=[])
        actor['moveSpeed' if record['source_list']=='objectList' else 'speed']=0.
        record.update(position=position,motion='standing')
        record.pop('motion_path',None)
    scene['waypointDataList']=[]
    meta.update(calibration_only=True,static_motion_reference=True)
    output.mkdir(parents=True)
    sid=old['id']+'_staticref';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
    atomic_json(sp,scene);atomic_json(mp,meta)
    row=copy.deepcopy(old);row.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
        scenario_sha256=sha256(sp),motion='standing')
    m.update(scenes=[row],count=1,status='STATIC_MOTION_REFERENCE_CALIBRATION_ONLY')
    atomic_json(output/'manifest.json',m);return output/'manifest.json'


def terrain_samples(cloud,start,end,spacing=.5):
    n=max(1,int(math.ceil(math.hypot(end['x']-start['x'],end['y']-start['y'])/spacing)))
    result=[]
    for i in range(n+1):
        t=i/n;xy=np.array([start['x']+(end['x']-start['x'])*t,start['y']+(end['y']-start['y'])*t])
        plane=surface_at(cloud,xy)
        if plane is None or not supported(plane,xy):raise ValueError('Unsupported motion terrain; no extrapolation')
        z=plane_height(plane,xy)
        if result and abs(z-result[-1]['z'])>.10:raise ValueError('Motion terrain step exceeds 10cm; reject path')
        result.append(dict(x=float(xy[0]),y=float(xy[1]),z=float(z),plane=plane))
    return result


def supported_motion_direction(cloud,path,record,meta):
    """Same target pivot/speed/budget; choose a measured safe straight departure direction."""
    from shapely.geometry import LineString,Point
    target=path['target_position'];start=path['points'][0];end=path['points'][-1]
    direction=np.array([end['x']-start['x'],end['y']-start['y']],float)
    direction/=np.linalg.norm(direction)
    departure=math.hypot(start['x']-target['x'],start['y']-target['y'])
    tail=math.hypot(end['x']-target['x'],end['y']-target['y'])
    ego=meta['ego'];yaw=math.radians(ego['yaw'])
    ego_box=gen.rectangle([ego['x']+1.35*math.cos(yaw),ego['y']+1.35*math.sin(yaw)],[4.8,2.],ego['yaw'])
    radius=math.hypot(*record['estimated_size'][:2])/2
    offset=math.hypot(record['geometric_center_estimate'][0]-target['x'],record['geometric_center_estimate'][1]-target['y'])
    other_boxes=[]
    for other in meta['runtime_actors']:
        if other['unique_id']==record['unique_id']:continue
        other_radius=math.hypot(*other['estimated_size'][:2])/2
        other_path=other.get('motion_path')
        if other_path:
            p=other_path['target_position']
            other_offset=math.hypot(other['geometric_center_estimate'][0]-p['x'],other['geometric_center_estimate'][1]-p['y'])
            other_boxes.append(LineString([(p['x'],p['y']) for p in other_path['points']]).buffer(other_radius+other_offset))
        else:other_boxes.append(Point(other['geometric_center_estimate']).buffer(other_radius))
    # Do not reorder the first candidate: paired evidence retains its original direction if supported.
    for degrees in (0,90,-90,45,-45,135,-135,180):
        angle=math.radians(degrees);c,s=math.cos(angle),math.sin(angle)
        v=np.array([c*direction[0]-s*direction[1],s*direction[0]+c*direction[1]])
        first=dict(x=target['x']-v[0]*departure,y=target['y']-v[1]*departure,z=target['z'])
        last=dict(x=target['x']+v[0]*tail,y=target['y']+v[1]*tail,z=target['z'])
        corridor=LineString([(first['x'],first['y']),(last['x'],last['y'])]).buffer(radius+offset)
        if corridor.distance(ego_box)<.3 or any(corridor.distance(b)<.2 for b in other_boxes):continue
        try:
            gen.validate_natural_path_roi([first,last],target,record['geometric_center_estimate'],ego,meta.get('base',ego)['yaw'])
            samples=terrain_samples(cloud,first,last)
        except ValueError:continue
        return samples,float(math.degrees(math.atan2(v[1],v[0]))),degrees
    raise ValueError('No independently supported safe motion direction in same target cell')


def prepare(source,terrain,reference,output,additional_terrain=None):
    """Derive asset pivot offset at target, then sample actual empty terrain along path."""
    if output.exists():raise FileExistsError('Grounded motion output overwrite refused')
    m=gen.geo.load_json(source)
    if len(m['scenes'])!=1:raise ValueError('Ground preparation is single-cell')
    old=m['scenes'][0];scene=gen.geo.load_json(Path(old['scenario']));meta=gen.geo.load_json(Path(old['metadata']))
    roots=[]
    terrain_sessions=[terrain]+list(additional_terrain or [])
    if len(terrain_sessions)>4:raise ValueError('Motion terrain witnesses bounded to four sessions')
    for session in terrain_sessions+[reference]:
        if not gen.geo.load_json(session/'shutdown_safety.json').get('park_verified'):raise ValueError('Unsafe calibration exit')
        ledger=[__import__('json').loads(l) for l in (session/'ledger.jsonl').read_text().splitlines() if l]
        if len(ledger)!=1 or ledger[0]['status'] not in ('accepted_candidate','review'):raise ValueError('Incomplete calibration')
        root=Path(ledger[0]['capture_root']);audit=gen.geo.load_json(root/'audit.json')
        if any('collision' in r or 'ego_vertical' in r for r in audit.get('review_reasons',[])):
            raise ValueError('Unsafe calibration contact/support')
        if not gen.geo.load_json(root/'metadata.json').get('calibration_only'):raise ValueError('Calibration provenance missing')
        roots.append(root)
    empty_roots,ref=roots[:-1],roots[-1];empty=empty_roots[0]
    clouds=[];files=[]
    for empty_root in empty_roots:
        em=gen.geo.load_json(empty_root/'metadata.json');et=gen.geo.load_json(empty_root/'telemetry.json')
        if em.get('runtime_actors'):raise ValueError('Terrain must have no actors')
        if em['ground_probe_source']['scenario_sha256']!=old['scenario_sha256']:raise ValueError('Terrain from different source')
        clouds.append(world_cloud(empty_root,em,et))
        files.extend(dict(path=str(p.resolve()),sha256=sha256(p)) for p in
            (empty_root/'metadata.json',empty_root/'telemetry.json',next(empty_root.glob('LIDAR_*/*_Intensity.bin'))))
    rm=gen.geo.load_json(ref/'metadata.json');rt=gen.geo.load_json(ref/'telemetry.json')
    if rm['spec']!=meta['spec']:raise ValueError('Static reference quota mismatch')
    cloud=np.concatenate(clouds)
    boxfile=next(ref.glob('LIDAR_*/*_Instance.txt'));boxes=parse_instances(boxfile)
    files.extend(dict(path=str(p.resolve()),sha256=sha256(p)) for p in
        (ref/'metadata.json',ref/'telemetry.json',boxfile))
    refs=[]
    for record in moving_actors(meta):
        uid=record['unique_id'];path=record['motion_path'];target=path['target_position'];corners=[]
        for box in boxes:
            if box['unique_id']!=uid:continue
            signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
            lidar=signs*box['size']/2@rotation_matrix(box['rotation_rpy']).T+box['center']
            ego=(np.c_[lidar,np.ones(8)]@np.array(rm['lidar_to_ego']).T)[:,:3]
            corners.extend(ego_to_world(ego,rt['ego_before']))
        if not corners:raise ValueError('Missing static asset annotation')
        cs=np.array(corners);plane=surface_at(cloud,[target['x'],target['y']],support_xy=cs[:,:2])
        if plane is None or not all(supported(plane,c[:2]) for c in cs):raise ValueError('Static pivot terrain unsupported')
        gap=float(np.min(cs[:,2]-[plane_height(plane,c[:2]) for c in cs]))
        if not -.30<=gap<=.75:raise ValueError('Static placement too far from ground; review')
        # Preserve asset-specific pivot height. OBB includes margins; no universal z=ground rule.
        pivot_offset=target['z']-plane_height(plane,[target['x'],target['y']])-max(0.,gap+.03)
        samples,yaw,direction_change=supported_motion_direction(cloud,path,record,meta)
        points=[dict(x=s['x'],y=s['y'],z=s['z']+pivot_offset) for s in samples]
        actor=next(a for a in scene[record['source_list']] if a['UNIQUEID']==uid)
        gen.geo.set_actor_world_position(actor,points[0],yaw)
        raw=next(p for p in scene['waypointDataList'] if p['pathName'].endswith(' - '+str(uid)))
        raw['waypointData']=[dict(pos=gen.geo.world_to_morai_waypoint(p),rot=gen.geo.formatted_rotation(0.)) for p in points]
        actor['waypointEventList']=[dict(index=-1,waypointEventData=[]) for _ in points]
        target['z']=plane_height(plane,[target['x'],target['y']])+pivot_offset
        path.update(points=points,ground_path_required=True,ground_path_evidence=dict(
            policy='independent-terrain-sampled-v46',files=files,samples=samples,
            asset_pivot_offset_m=pivot_offset,static_annotation_ground_gap_m=gap,
            terrain_selected_direction_change_deg=direction_change,
            limitation='LiDAR local plane + static annotation witness; not continuous mesh contact proof. Capture audit required.'))
        record['position']=copy.deepcopy(points[0])
        refs.append(dict(uid=uid,plane_world=plane,probe_capture=str(empty),
            probe_captures=[str(p) for p in empty_roots],MR_heading_correction_deg=-1.347))
    meta['ground_probe_references']=refs
    meta['ground_support_check']=dict(enabled=True,max_range_m=20.,maximum_gap_m=.15,
        required_uids=[a['unique_id'] for a in meta['runtime_actors']],
        policy='all-actors-observed-ground-v1')
    validate_motion_ground(meta,scene)
    output.mkdir(parents=True)
    sid=old['id']+'_groundpath';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
    atomic_json(sp,scene);atomic_json(mp,meta)
    row=copy.deepcopy(old);row.update(id=sid,source_cell_id=old.get('source_cell_id',old['id']),
        scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp))
    m.update(count=1,scenes=[row],status='GROUND_PREPARED_MOTION; CAPTURE_AUDIT_REQUIRED')
    atomic_json(output/'manifest.json',m);return output/'manifest.json'
