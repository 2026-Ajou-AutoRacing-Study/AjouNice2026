#!/usr/bin/env python3
"""Requires the sourced AW149 workspace and its configured mission-planner build."""
from pathlib import Path
import os,socket,time,subprocess,xmlrpc.client,math,json,sys
import yaml
import argparse,shlex
parser=argparse.ArgumentParser(description='Audit v87 A/B using the installed production plugin on a private ROS master.')
parser.add_argument('--output-dir',type=Path,default=Path('planner_logs/analysis/kcity_v87_route_audit'))
parser.add_argument('--report',type=Path,default=None)
args=parser.parse_args()
root=Path(__file__).resolve().parents[3];out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
# Reuse the workspace's resolved include/link flags; never rebuild or replace the production plugin.
build=root/'planning_ws/build_isolated/autoware_mission_planner_universe/CMakeFiles/default_planner_plugin_oracle.dir'
flags=next(s.split('=',1)[1] for s in (build/'flags.make').read_text().splitlines() if s.startswith('CXX_INCLUDES ='))
link=shlex.split((build/'link.txt').read_text());libraries=link[link.index('-o')+2:]
subprocess.run(['/usr/bin/c++','-std=c++17','-O1']+shlex.split(flags)+[str(Path(__file__).with_name('probe_v87_native_routes.cpp')),'-o',str(out/'native_route_probe')]+libraries+['-lyaml-cpp'],check=True)
sys.path.insert(0,str(root/'base_ws/src/planning_addons/competition_route_requester/src'))
from competition_route_requester import policy
routes=yaml.safe_load((root/'base_ws/src/planning_addons/competition_route_requester/config/kcity_v87_competition_route.yaml').read_text())
params=yaml.safe_load((root/'base_ws/src/planning_addons/ajounice_planning_integration/config/mission_planner_kcity_v87.yaml').read_text())
params.update(yaml.safe_load((root/'configs/planning/vehicle_info.yaml').read_text()))
def coords(p):return [float(p['x']),float(p['y']),float(p.get('z',0)),math.radians(p['yaw_deg'])]
cases=[]
for key in ['route_a','route_b']:
 start=routes['start']['map_pose'] if key=='route_a' else routes['poses']['wp_200178']
 cases.append({'name':key+'_full','points':[coords(start)]+[coords(routes['poses'][r]) for r in routes[key]['waypoint_refs']+[routes[key]['goal_ref']]]})
# Exercise actual B requests after the trigger and at downstream startup points.
for ref in ['wp_200258','wp_200311','wp_201473']:
 p=routes['poses'][ref];refs=policy.remaining_waypoint_refs(routes,'route_b',p['lanelet_id'])
 cases.append({'name':'route_b_from_'+ref,'points':[coords(p)]+[coords(routes['poses'][r]) for r in refs+[routes['route_b']['goal_ref']]]})
# Exhaust all allowed Route B startup lanelets using a midpoint on their explicit centerline.
from competition_route_requester.geometry import Geometry
geom=Geometry(str(root),routes)
bseq=routes['route_b']['waypoint_path_lanelet_ids']
start=bseq.index(routes['poses'][routes['startup_route_b_trigger']['pose_ref']]['lanelet_id'])
end=bseq.index(routes['startup_route_b_trigger']['end_lanelet_id'])
for lane in bseq[start:end+1]:
 from shapely.geometry import LineString
 line=LineString(geom.lanes[lane][0]);point=line.interpolate(line.length/2);a=line.interpolate(max(0,line.length/2-.1));b=line.interpolate(min(line.length,line.length/2+.1))
 p={'x':point.x,'y':point.y,'z':28.3,'yaw_deg':math.degrees(math.atan2(b.y-a.y,b.x-a.x))}
 refs=policy.remaining_waypoint_refs(routes,'route_b',lane)
 cases.append({'name':'route_b_startup_'+str(lane),'points':[coords(p)]+[coords(routes['poses'][r]) for r in refs+[routes['route_b']['goal_ref']]]})
params.update(route_policy_cases=cases,output_file=str(out/'native_routes.yaml'))
(out/'native_params.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
s=socket.socket();s.bind(('127.0.0.1',0));port=s.getsockname()[1];s.close()
env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_HOME=str(out/'private_ros'))
processes=[];logs=[]
def start(cmd,name):
 f=open(out/(name+'.log'),'w');logs.append(f);p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
try:
 start(['roscore','-p',str(port)],'master')
 master=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
 for _ in range(100):
  try:
   if master.getPid('/v87_audit')[0]==1:break
  except Exception:pass
  time.sleep(.1)
 else:raise RuntimeError('private master timeout')
 start(['roslaunch',str(root/'base_ws/src/planning_addons/ajounice_planning_integration/launch/map.launch')],'map')
 subprocess.run(['rosparam','load',str(out/'native_params.yaml'),'/v87_native_route_probe'],env=env,check=True)
 probe=start([str(out/'native_route_probe')],'probe');code=probe.wait(timeout=180)
 result=yaml.safe_load((out/'native_routes.yaml').read_text());result['private_master_uri']=env['ROS_MASTER_URI'];result['exit_code']=code
 (out/'native_routes_raw.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({'status':result['status'],'cases':[dict({k:c[k] for k in ['name','nonempty']},sections=len(c.get('sections',[]))) for c in result.get('cases',[])],'error':result.get('error')},indent=2))
 if code:sys.exit(code)
finally:
 import signal
 for p in reversed(processes):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in processes:
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
 for f in logs:f.close()

import ctypes,hashlib
ctypes.CDLL(str(root/'planning_ws/install_isolated/lib/libautoware_lanelet2_extension_lib.so'),mode=ctypes.RTLD_GLOBAL)
import lanelet2
m=lanelet2.io.load(str(root/'kcity_map/lanelet_map/lanelet2_map_v87.osm'),lanelet2.projection.UtmProjector(lanelet2.io.Origin(37.2380038082,126.7730819386)))
g=lanelet2.routing.RoutingGraph(m,lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,lanelet2.traffic_rules.Participants.Vehicle))
conf=yaml.safe_load((root/'base_ws/src/planning_addons/competition_route_requester/config/kcity_v87_competition_route.yaml').read_text())
raw=json.loads((out/'native_routes_raw.json').read_text())
result=[]
for c in raw['cases']:
 assert c['nonempty'],c['name']
 seq=[s['preferred'] for s in c['sections']]
 key='route_a' if c['name'].startswith('route_a') else ('fallback_route' if c['name'].startswith('fallback') else 'route_b')
 assert seq in policy.accepted_sequences(conf,key,'waypoint_path_lanelet_ids' if key!='fallback_route' else 'expected_path_lanelet_ids'),(c['name'],seq)
 sets=[set(s['primitives']) for s in c['sections']]
 for i,s in enumerate(c['sections']):
  assert s['preferred'] in sets[i]
  assert all(m.laneletLayer.exists(x) for x in sets[i])
 def lateral_closure(ids,allowed):
  reached=set(ids)
  while True:
   added=set()
   for x in reached:
    for y in [g.left(m.laneletLayer[x]),g.right(m.laneletLayer[x])]:
     if y and y.id in allowed:added.add(y.id)
   if added<=reached:return reached
   reached|=added
 reached=lateral_closure(sets[0],sets[0]);transitions=[]
 for i in range(1,len(sets)):
  following={y.id for x in reached for y in g.following(m.laneletLayer[x])}
  entry=following&sets[i]
  assert entry,('section gap',c['name'],i,sorted(reached),sorted(sets[i]))
  reached=lateral_closure(entry,sets[i]);transitions.append({'from_section':i-1,'to_section':i,'reachable_lanelets':sorted(reached)})
 assert seq[-1] in reached,(c['name'],'goal not reachable')
 result.append({'name':c['name'],'sections':len(seq),'preferred_sequence_matches_config_or_accepted_suffix':True,'all_section_transitions_connected':True,'actual_preferred_lanelet_ids':seq,'transitions':transitions})
out={'status':'PASS','map_sha256':hashlib.sha256((root/'kcity_map/lanelet_map/lanelet2_map_v87.osm').read_bytes()).hexdigest(),'method':'Installed production DefaultPlanner on private ROS master, production footprint and original OSM speeds (no overrides); Lanelet2 graph checks emitted section-by-section continuity','cases':result,'live_closed_loop':False}
(args.report or out/'route_audit.json').write_text(json.dumps(out,indent=2)+'\n')
print('PASS',len(result),'native cases,',sum(x['sections']-1 for x in result),'section transitions')
