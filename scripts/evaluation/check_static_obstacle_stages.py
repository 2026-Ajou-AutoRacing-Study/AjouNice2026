"""Check safety invariants on every published planning stage (recorded-ego replay)."""
import argparse,bisect,json,math,yaml
from pathlib import Path
from collections import Counter
import numpy as np
from shapely.geometry import Polygon
from shapely.prepared import prep

p=argparse.ArgumentParser();p.add_argument('result',type=Path);a=p.parse_args();r=json.loads(a.result.read_text())
scenes=sorted(r['scenes'],key=lambda x:x['time']);scene_times=[s['time'] for s in scenes]
source_scene={round(s['time'],6):s for s in scenes}
source_scene_ns={s['time_ns']:s for s in scenes if 'time_ns' in s}
available_scenes=sorted((s for s in scenes if 'wall' in s),key=lambda s:s['wall'])
available_times=[s['wall'] for s in available_scenes]
ego=sorted(r['odometry'],key=lambda x:x['time']);ego_times=[e['time'] for e in ego]
planning_ego={round(e['time'],6):e for e in r.get('planning_odometry',[])}
roads={round(b['time'],6):b for b in r['records']}
prepared_roads={}
scene_checks={(int(s['publication_index']),int(s['trajectory_crc32'])):s
              for s in r.get('scene_checks',[])}

def footprint(x,y,yaw):
 c=math.cos(yaw);s=math.sin(yaw)
 return Polygon([(x+c*dx-s*dy,y+s*dx+c*dy) for dx,dy in [(3.845,.945),(3.845,-.945),(-.79,-.945),(-.79,.945)]])

def check(trajectories, use_snapshots=False):
 result=Counter();failures=[]
 for tr in trajectories:
  t=tr['time']
  if t<r['bag_start'] or not tr['points']:continue
  evaluation_time=max(t,tr.get('received',t))
  scene=source_scene.get(round(t,6))
  if available_scenes and 'wall' in tr:
   available_idx=bisect.bisect_right(available_times,tr['wall'])-1
   if available_idx>=0:scene=available_scenes[available_idx]
  elif evaluation_time-t>=.05:
   # Legacy recordings lack receipt ordering: this is only an approximation.
   scene=scenes[max(0,bisect.bisect_right(scene_times,evaluation_time)-1)]
  if scene is None:continue
  e=planning_ego.get(round(evaluation_time,6),ego[max(0,bisect.bisect_right(ego_times,evaluation_time)-1)])
  if use_snapshots:
   snapshot=scene_checks.get((tr.get('sequence'),tr.get('crc32')))
   if snapshot is None or 'ego_x' not in snapshot:
    result['missing_validation_snapshot']+=1;continue
   stamp_ns=int(snapshot['scene_stamp_ns'])
   if source_scene_ns:
    scene=source_scene_ns.get(stamp_ns)
   else:
    # Match rospy.Time.to_sec() arithmetic for recordings predating integer stamps.
    secs,nsecs=divmod(stamp_ns,1000000000)
    scene=source_scene.get(round(secs+float(nsecs)/1000000000,6))
   if scene is None:
    result['missing_snapshot_scene']+=1;continue
   e={key:float(snapshot['ego_'+key]) for key in ['x','y','yaw','v']}
  pts=np.asarray(tr['points']);idx=int(np.argmin(np.hypot(pts[:,0]-e['x'],pts[:,1]-e['y'])))
  result['frames']+=1
  result['stale_header_frames']+=evaluation_time-t>.05
  # Positive velocity needs a forward-facing vehicle pose. A global pi flip can
  # remain collision-free yet be impossible for the controller to follow.
  delta=np.diff(pts[idx:,:2],axis=0)
  lengths=np.linalg.norm(delta,axis=1)
  offsets=np.concatenate(([0.],np.cumsum(lengths)))
  forward=pts[idx:,3]>.01
  stop=np.flatnonzero(~forward)
  end=int(stop[0]) if len(stop) else len(forward)
  eligible=(lengths>1e-6)&(offsets[:-1]<47.5)&(np.arange(len(lengths))<end)
  direction=delta[:,0]*np.cos(pts[idx:-1,2])+delta[:,1]*np.sin(pts[idx:-1,2])
  result['moving_inconsistent_heading_frames']+=bool(np.any(eligible&(direction<0)))
  # Check the actual rectangular footprint over the optimized 47.5 m horizon.
  # The longer tail is an unoptimized route extension, not a PO solution.
  road_key=round(scene['time'],6)
  if road_key in roads:
   if road_key not in prepared_roads:
    b=roads[road_key];road=Polygon(b['left']+b['right'][::-1])
    prepared_roads[road_key]=prep(road.buffer(.001)) if road.is_valid else None
   road=prepared_roads[road_key]
   if road is not None:
    distance=0.;outside=False
    for j in range(idx,len(pts)):
     if j>idx:distance+=float(np.linalg.norm(pts[j,:2]-pts[j-1,:2]))
     if distance>47.5 or abs(pts[j,3])<.01:break
     if not road.covers(footprint(*pts[j,:3])):outside=True;break
    result['moving_outside_road_frames']+=outside
  if not scene['obstacles']:continue
  result['active_frames']+=1
  polygons=[Polygon(o['polygon']).buffer(o['hard']) for o in scene['obstacles']]
  boxes=[poly.bounds for poly in polygons]
  measured_collision=any(footprint(e['x'],e['y'],e['yaw']).intersects(poly) for poly in polygons)
  result['measured_margin_overlap']+=measured_collision
  if pts[idx,3]<.01:
   result['zero_at_ego']+=1
   result['zero_with_measured_margin_overlap']+=measured_collision
  bad=False
  for x,y,yaw,v,*_ in pts[idx:]:
   if abs(v)<.01:break
   nearby=[j for j,b in enumerate(boxes) if b[0]-4.0<x<b[2]+4.0 and b[1]-4.0<y<b[3]+4.0]
   if not nearby:continue
   poly=footprint(x,y,yaw)
   if any(poly.intersects(polygons[j]) for j in nearby):bad=True;break
  result['moving_collision_frames']+=bad
  if bad:failures.append({'time':t-r['bag_start'],'received':tr['received']-r['bag_start'] if 'received' in tr else None,
    'sequence':tr.get('sequence'),'wall':tr.get('wall'),'scene_time':scene['time']-r['bag_start'],
    'scene_wall':scene.get('wall')})
 return {'counts':dict(result),'failures':failures}

stages={'PO':r['trajectories'],**r.get('stages',{})}
results={name:check(trajs) for name,trajs in stages.items()}
vehicle=yaml.safe_load((Path(__file__).resolve().parents[2]/'configs/planning/vehicle_info.yaml').read_text())
curvature_limit=math.tan(vehicle['max_steer_angle'])/vehicle['wheel_base']
kinematic_failures=[]
for tr in r['trajectories']:
 t=round(tr['time'],6);e=planning_ego.get(t);scene=source_scene.get(t)
 if tr['time']<r['bag_start'] or not e or not scene or not scene['obstacles']:continue
 pts=np.asarray(tr['points'])
 if len(pts)<3:continue
 distances=np.hypot(pts[:,0]-e['x'],pts[:,1]-e['y']);idx=int(np.argmin(distances))
 if pts[idx,3]<.01:continue
 yaw_gap=abs(math.atan2(math.sin(pts[idx,2]-e['yaw']),math.cos(pts[idx,2]-e['yaw'])))
 if distances[idx]>1.0+1e-6 or yaw_gap>.35+1e-6:
  kinematic_failures.append({'time':tr['time']-r['bag_start'],'kind':'initial_pose',
                            'gap':float(distances[idx]),'yaw_gap':yaw_gap})
 distance=0.
 for i in range(max(idx,1),len(pts)-1):
  if i>idx:distance+=float(np.linalg.norm(pts[i,:2]-pts[i-1,:2]))
  if distance>10.0 or pts[i,3]<.01:break
  u=pts[i,:2]-pts[i-1,:2];v=pts[i+1,:2]-pts[i,:2]
  denominator=float(np.linalg.norm(u)*np.linalg.norm(v)*np.linalg.norm(u+v))
  if denominator<=1e-8:continue
  curvature=2*abs(float(u[0]*v[1]-u[1]*v[0]))/denominator
  if curvature>curvature_limit+1e-3:
   kinematic_failures.append({'time':tr['time']-r['bag_start'],'kind':'local_curvature',
                             'curvature':curvature,'distance':distance});break
results['PO_initial_and_near_curvature_failures']=kinematic_failures
results['PO_near_curvature_check']={'horizon_m':10.0,'limit_per_m':curvature_limit,'tolerance_per_m':1e-3}
if r.get('scene_checks') or any('crc32' in tr for tr in stages.get('/planning/trajectory', [])):
 final='/planning/trajectory'
 results['final_against_latest_observer_scene']=results[final]
 results[final]=check(stages[final],use_snapshots=True)
 observer_failures=results['final_against_latest_observer_scene']['failures']
 bad_sequences={f['sequence'] for f in observer_failures}
 responses=[]
 for failure in observer_failures:
  later=[tr for tr in stages[final] if tr.get('wall',0)>failure['wall'] and
         tr.get('sequence') not in bad_sequences and
         (tr.get('sequence'),tr.get('crc32')) in scene_checks and
         int(scene_checks[(tr['sequence'],tr['crc32'])]['scene_stamp_ns'])*1e-9 >=
           r['bag_start']+failure['scene_time']-1e-6]
  next_output=min(later,key=lambda tr:tr['wall']) if later else None
  responses.append({**failure,'safe_output_delay_ms':
                    (next_output['wall']-failure['scene_wall'])*1000 if next_output else None})
 results['new_scene_responses']=responses
invalid=Counter()
for v in r.get('validation',[]):
 if v['stamp']>=r['bag_start']:invalid.update(k for k,val in v.items() if k.startswith('is_valid') and not val)
results['validator_invalid_fields']=dict(invalid)
results['invalid_scene_polygons']=sum(not Polygon(o['polygon']).is_valid for s in scenes for o in s['obstacles'])
times=r.get('po_processing_ms',[])
results['po_processing_ms_p50_p95_max']=np.percentile(times,[50,95,100]).tolist() if times else None
cycles=r.get('cycle_wall',[])
results['coverage']={'incomplete_cycles':sum(not c['complete'] for c in cycles),
 'cycles':len(cycles),'po_frames':results['PO']['counts'].get('frames',0),
 'missing_scene_frames':sum(round(t['time'],6) not in source_scene for t in r['trajectories'] if t['time']>=r['bag_start']),
 'fatal_logs':sum(level>=16 for _,level,_ in r['logs'])}
(a.result.parent/'stage_metrics.json').write_text(json.dumps(results,indent=2));print(a.result.parent.name,json.dumps(results))
