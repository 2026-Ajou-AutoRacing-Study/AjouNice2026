#!/usr/bin/env python3
"""Run a retained oracle against its configured prefix and owned private ROS master."""
from pathlib import Path
import os, subprocess, sys, time, yaml
binary,prefix=map(Path,sys.argv[1:3]);mode=sys.argv[3] if len(sys.argv)>3 else ''
if os.environ.get('AW149_RETAINED_PREFIX_ENV')!=str(prefix):
    env=dict(os.environ,AW149_RETAINED_PREFIX_ENV=str(prefix))
    raise SystemExit(subprocess.call([str(prefix/'env.sh'),sys.executable,__file__,*sys.argv[1:]],env=env))
home=Path(os.environ['ROS_HOME'])
assert home.parent.name.startswith('private-test-'),'This fixture requires the private test owner'
if binary.name!='planning_validator_node_interface_oracle':
    for kind in ['intersection_collision','trajectory']:
        package='autoware_planning_validator_'+kind+'_checker'
        subprocess.run(['rosparam','load',str(prefix/'share'/package/'config'/(kind+'_checker.ros1.yaml')),'/aw149_full_ingress'],check=True)
    raise SystemExit(subprocess.call([str(binary)]))
test_share=Path(subprocess.check_output(['rospack','find','autoware_test_utils'],text=True).strip())
vehicle=yaml.safe_load((test_share/'config/test_vehicle_info.param.yaml').read_text())['/**']['ros__parameters']
vehicle_file=home/'retained-validator-vehicle.yaml';vehicle_file.write_text(yaml.safe_dump(vehicle))
args=['vehicle_param_path:='+str(vehicle_file)]
if mode=='--expect-no-diagnostics':args+=['publish_diag:=false']
if mode=='--exercise-diagnostic-threshold':args+=['diag_error_count_threshold:=2']
log_path=home/('retained-validator-'+(mode or 'normal')+'.log')
with log_path.open('w') as log:
    process=subprocess.Popen(['roslaunch','autoware_planning_validator','planning_validator_candidate_ros1.launch',*args],stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            assert process.poll() is None,log_path.read_text()
            listed=subprocess.check_output(['rosnode','list'],text=True)
            if '/planning_validator_candidate' in listed.splitlines():break
            time.sleep(.1)
        else:raise RuntimeError(log_path.read_text())
        subprocess.run([str(binary)]+([mode] if mode else []),check=True,timeout=120)
    finally:
        process.terminate()
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
