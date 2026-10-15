"""Attribute nonoverlap stops to observed pipeline stages, with matched validator snapshots."""
import argparse,json,math,bisect
from pathlib import Path
from collections import Counter
from shapely.geometry import Polygon
parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
run=args.run;allrows=[]
for summary in json.loads((run/'summary.json').read_text()):
 name=Path(summary['bag']).stem;p=run/name;r=json.loads((p/'result.json').read_text());bs=r['bag_start']
 scenes=sorted(r['scenes'],key=lambda s:s['wall']);st=[s['wall'] for s in scenes]
 ego=sorted(r['odometry'],key=lambda e:e['time']);et=[e['time'] for e in ego]
 pe={round(e['time'],6):e for e in r['planning_odometry']}
 snapshots={(int(c['publication_index']),int(c['trajectory_crc32'])):c for c in r.get('scene_checks',[])}
 scene_ns={s['time_ns']:s for s in scenes if 'time_ns' in s}
 upstream={'PO':r['trajectories'],'MVP':r['stages']['/planning/scenario_planning/lane_driving/trajectory'],'VS':r['stages']['/planning/scenario_planning/velocity_smoother/trajectory']}
 def near(pts,e):return min(range(len(pts)),key=lambda i:math.hypot(pts[i][0]-e['x'],pts[i][1]-e['y']))
 for tr in r['stages']['/planning/trajectory']:
  if tr['time']<bs or not tr['points']:continue
  scene=scenes[max(0,bisect.bisect_right(st,tr['wall'])-1)]
  t=max(tr['time'],tr['received']);e=pe.get(round(t,6),ego[max(0,bisect.bisect_right(et,t)-1)])
  if snapshots:
   c=snapshots[(tr['sequence'],tr['crc32'])];scene=scene_ns[int(c['scene_stamp_ns'])]
   e={k:float(c['ego_'+k]) for k in ['x','y','yaw','v']}
  if not scene['obstacles']:continue
  i=near(tr['points'],e)
  if tr['points'][i][3]>=.01:continue
  c,s=math.cos(e['yaw']),math.sin(e['yaw']);foot=Polygon([(e['x']+c*a-s*b,e['y']+s*a+c*b) for a,b in [(3.845,.945),(3.845,-.945),(-.79,-.945),(-.79,.945)]])
  polys=[Polygon(o['polygon']).buffer(o['hard']) for o in scene['obstacles']]
  if any(foot.intersects(poly) for poly in polys):continue
  speeds={}
  for stage,trs in upstream.items():
   candidates=[x for x in trs if abs(x['time']-tr['time'])<1e-6 and x['points']]
   if len(candidates)>1:
    candidates=[x for x in candidates if x['wall']<=tr['wall']]
   if candidates:
    x=max(candidates,key=lambda x:x['wall']);speeds[stage]=x['points'][near(x['points'],e)][3]
  stage=next((k for k,v in speeds.items() if abs(v)<.01),'validator' if len(speeds)==len(upstream) else 'unresolved_upstream')
  warnings=[{'t':round(ts-bs,3),'message':msg} for ts,l,msg in r['logs'] if tr['received']-2<=ts<=tr['received']+.01 and l>=4 and ('[MPT]' in msg or '[Static scene]' in msg)]
  allrows.append({'bag':name[-6:],'received':round(tr['received']-bs,3),'header':round(tr['time']-bs,3),'scene':round(scene['time']-bs,3),'ego_v':e['v'],'gap':math.hypot(tr['points'][i][0]-e['x'],tr['points'][i][1]-e['y']),'clearance':min(foot.distance(poly) for poly in polys),'first_zero_stage_at_current_pose':stage,'upstream_velocity_at_current_pose':speeds,'warnings_nearby':warnings})
(run/'nonoverlap_stops.json').write_text(json.dumps(allrows,indent=2))
lines=['# 관측 마진과 겹치지 않은 최종 정지 프레임','', '최종 출력의 검증 기록이 있으면 실제 검증 장면과 자차 위치를 사용하며, 같은 header를 가진 선행 출력들을 비교했다. 같은 header의 선행 출력이 하나이면 기록기 수신 순서가 뒤바뀌어도 연결한다. 최초 0 속도 단계는 관측값이며, 로그가 throttle되므로 인접 로그만으로 모든 프레임의 원인을 단정하지 않는다.','', '| 백 | 발행 / 경로 시각 s | 입력 속도 m/s | 차체-마진 거리 m | 경로와의 거리 m | 최초 0 속도 단계 |','|---|---:|---:|---:|---:|---|']
for x in allrows:lines.append(f"| {x['bag']} | {x['received']:.3f} / {x['header']:.3f} | {x['ego_v']:.2f} | {x['clearance']:.4f} | {x['gap']:.3f} | {x['first_zero_stage_at_current_pose']} |")
(run/'nonoverlap_stops.md').write_text('\n'.join(lines)+'\n')
print('TOTAL',len(allrows),dict(Counter(x['first_zero_stage_at_current_pose'] for x in allrows)))
for x in allrows:print(x['bag'],x['received'],x['first_zero_stage_at_current_pose'],round(x['clearance'],4),round(x['gap'],3),x['warnings_nearby'])
