"""Run a frozen bag manifest serially, each with a fresh private master."""
import argparse,json,subprocess,sys,os,signal
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--full-pipeline',action='store_true');p.add_argument('--port',type=int,default=11473);p.add_argument('--lockstep',action='store_true');p.add_argument('--native-player',action='store_true');a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
results=[]
failed=False
for item in json.loads(a.manifest.read_text()):
 if not item['selected']:continue
 bag=Path(item['bag']);out=a.output/bag.stem;print('START',bag.name,flush=True)
 if not (out/'result.json').exists():
  with (a.output/(bag.stem+'.log')).open('w') as log:
   child=subprocess.Popen([sys.executable,'scripts/evaluation/replay_static_obstacle_scene.py','--bag',str(bag),'--output',str(out),'--port',str(a.port)]+(['--full-pipeline'] if a.full_pipeline else [])+(['--lockstep'] if a.lockstep else [])+(['--native-player'] if a.native_player else []),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   try:rc=child.wait()
   except KeyboardInterrupt:
    child.send_signal(signal.SIGINT)
    child.wait(timeout=30)
    raise
  if rc:
   failed=True
   results.append({'bag':str(bag),'execution_failure':rc});print('EXECUTION FAILED',bag.name,flush=True);continue
 rc=subprocess.call([sys.executable,'scripts/evaluation/analyze_static_obstacle_replay.py',str(out/'result.json')])
 if rc: raise RuntimeError('metric calculation failed')
 result=json.loads((out/'metrics.json').read_text())['summary']
 if a.full_pipeline:
  subprocess.run([sys.executable,'scripts/evaluation/check_static_obstacle_stages.py',str(out/'result.json')],check=True,stdout=subprocess.DEVNULL)
  stages=json.loads((out/'stage_metrics.json').read_text())
  coverage=stages['coverage'];final=stages['/planning/trajectory']['counts']
  errors=[]
  for key in ['incomplete_cycles','missing_scene_frames','fatal_logs']:
   if coverage[key]:errors.append(key)
  if final.get('frames',0)<.8*item['duration']*10:errors.append('insufficient_final_coverage')
  if result.get('missing_publication_evidence',0):errors.append('missing_PO_publication_evidence')
  if result.get('max_output_gap') is None or result['max_output_gap']>.5:errors.append('PO_output_gap_over_500ms')
  if a.lockstep and coverage['po_frames']!=coverage['cycles']:errors.append('missing_PO_cycle')
  if stages['PO']['counts'].get('moving_inconsistent_heading_frames',0):
   errors.append('PO_inconsistent_forward_heading')
  if stages.get('PO_initial_and_near_curvature_failures'):
   errors.append('PO_initial_pose_or_local_curvature')
  for key in ['moving_collision_frames','moving_outside_road_frames','moving_inconsistent_heading_frames']:
   if final.get(key,0):errors.append(key)
  for key in ['missing_validation_snapshot','missing_snapshot_scene']:
   if final.get(key,0):errors.append(key)
  if any(x['safe_output_delay_ms'] is None for x in stages.get('new_scene_responses',[])):
   errors.append('unresolved_new_scene_output')
  if stages['invalid_scene_polygons']:errors.append('invalid_scene_polygons')
  for log in out.glob('*.log'):
   contents=log.read_text(errors='replace')
   if any(token in contents for token in ['process has died','Segmentation fault','terminate called after']):errors.append('node_crash:'+log.name)
  result['safety_coverage_gate_errors']=errors
  failed=failed or bool(errors)
  print('GATE',bag.name,errors or 'PASS',flush=True)
 results.append(result)
 (a.output/'summary.json').write_text(json.dumps(results,indent=2))
print('FINISHED',len(results),flush=True)
(a.output/'summary.json').write_text(json.dumps(results,indent=2))
sys.exit(1 if failed else 0)
