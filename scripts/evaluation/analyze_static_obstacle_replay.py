"""Independent, recorded-ego replay metrics. This is not a closed-loop driving test."""
import argparse,json,math,re
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon

p=argparse.ArgumentParser();p.add_argument('result',type=Path);a=p.parse_args()
r=json.loads(a.result.read_text());ego=r['odometry'];et=np.array([e['time'] for e in ego]);scenes={round(s['time'],6):s for s in r['scenes']}
planning_ego={round(e['time'],6):e for e in r.get('planning_odometry',[])}
rows=[]
for tr in r['trajectories']:
 t=tr['time']
 if t < r['bag_start']:continue
 pts=np.array(tr['points']); s=scenes.get(round(t,6));e=ego[max(0,np.searchsorted(et,t,side='right')-1)]
 e=planning_ego.get(round(t,6),e)
 if not len(pts):continue
 distances=np.hypot(pts[:,0]-e['x'],pts[:,1]-e['y']);idx=int(np.argmin(distances));near=pts[idx]
 row=dict(t=round(t-r['bag_start'],3),objects=len(s['obstacles']) if s else None,
  ego_v=e['v'],target_v=float(near[3]),ego_gap=float(distances[idx]),latency=tr['received']-t,
  collision=False,first_collision_distance=None)
 if s and s['obstacles']:
  poly=[Polygon(o['polygon']).buffer(o['hard']) for o in s['obstacles']]
  boxes=[o.bounds for o in poly]
  dist=0.0
  for i in range(idx,len(pts)):
   if i>idx:dist+=np.linalg.norm(pts[i,:2]-pts[i-1,:2])
   x,y,yaw,v=pts[i]
   # Geometry beyond a commanded stop is not a commanded traversal.
   if abs(v)<0.01:break
   nearby=[j for j,b in enumerate(boxes) if b[0]-4.0<x<b[2]+4.0 and b[1]-4.0<y<b[3]+4.0]
   if not nearby:continue
   c=math.cos(yaw);sn=math.sin(yaw)
   footprint=Polygon([(x+c*dx-sn*dy,y+sn*dx+c*dy) for dx,dy in [(3.845,.945),(3.845,-.945),(-.79,-.945),(-.79,.945)]])
   if any(footprint.intersects(poly[j]) for j in nearby):
    row['collision']=True;row['first_collision_distance']=dist;break
 rows.append(row)
active=[x for x in rows if x['objects']]
logs=[msg for t,l,msg in r['logs']]
published=[tr for tr in r['trajectories'] if tr['time']>=r['bag_start'] and tr['points']]
cadence_key='wall' if all('wall' in tr for tr in published) else 'received'
published.sort(key=lambda tr:tr[cadence_key])
receipt_gap=max((b[cadence_key]-a[cadence_key] for a,b in zip(published,published[1:])),default=None)
publication_gap=None
missing_publications=0
if 'po_publications' in r:
 evidence={(x['sequence'],x['crc32']):x for x in r['po_publications']}
 matched=[evidence.get((tr.get('sequence'),tr.get('crc32'))) for tr in published]
 missing_publications=sum(x is None for x in matched)
 ordered=sorted((x for x in matched if x is not None),key=lambda x:x['before_ns'])
 # Publish occurs between before_ns and after_ns. This is a conservative upper
 # bound on each actual publication gap, independent of the recorder callback.
 publication_gap=max(((b['after_ns']-a['before_ns'])/1e9 for a,b in zip(ordered,ordered[1:])),default=None)
summary=dict(bag=r['bag'],bpp_frames=len(r['records']),scene_frames=len(r['scenes']),po_frames=len(rows),
 active_frames=len(active),moving_collision_frames=sum(x['collision'] for x in rows),
 active_zero_at_ego=sum(abs(x['target_v'])<.01 for x in active),
 active_max_ego_gap=max((x['ego_gap'] for x in active),default=None),
 no_candidate_logs=sum('selected=0' in x for x in logs),
 missing_scene=sum('Missing synchronized' in x for x in logs),
 optimizer_fail_logs=sum('could not solve qp' in x for x in logs),
 max_latency=max((x['latency'] for x in rows),default=None),
 max_input_stamp_gap=max((b['time']-a['time'] for a,b in zip(published,published[1:])),default=None),
 output_gap_clock='PO publish call upper bound' if 'po_publications' in r else cadence_key,
 missing_publication_evidence=missing_publications,
 max_recorder_output_gap=receipt_gap,
 max_output_gap=publication_gap if 'po_publications' in r else receipt_gap)
(a.result.parent/'metrics.json').write_text(json.dumps({'summary':summary,'frames':rows},indent=2))
print(json.dumps(summary,indent=2))
