"""Read-only bag replay on an isolated ROS master; never publishes to the live master."""
import argparse, json, os, signal, subprocess, time, xmlrpc.client, math, copy, io, zlib
from pathlib import Path
import rosbag, rosparam, rospy, yaml
from rosgraph_msgs.msg import Clock, Log
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from autoware_planning_validator.msg import PlanningValidatorStatus
from autoware_internal_debug_msgs.msg import Float64Stamped, StringStamped
from autoware_internal_planning_msgs.msg import Scenario, PathWithLaneId
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Path as PlanningPath, Trajectory, StaticObstacleScene

root = Path('/home/jihun/AjouNice2026')
parser = argparse.ArgumentParser()
parser.add_argument('--bag', type=Path, default=root/'planner_logs/bags/planning_rviz_20260919_031426.bag')
parser.add_argument('--duration', type=float, default=1000.0)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--rate', type=float, default=1.0)
parser.add_argument('--port', type=int, default=11473)
parser.add_argument('--full-pipeline', action='store_true')
parser.add_argument('--warmup', type=float, default=5.0)
parser.add_argument('--lockstep', action='store_true')
parser.add_argument('--native-player', action='store_true')
args = parser.parse_args()
if args.lockstep and args.native_player: parser.error("native-player and lockstep are mutually exclusive")
out = args.output
out.mkdir(parents=True, exist_ok=False)
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:'+str(args.port)
os.environ['ROS_LOG_DIR'] = str(out / 'ros')
procs, handles, logs, records, trajectories, scenes, odometry, observed, planning_odometry = [], [], [], [], [], [], [], [], []
def point(p):
    q=p.pose.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return [p.pose.position.x,p.pose.position.y,yaw,p.longitudinal_velocity_mps]
def spawn(args, name):
    h = open(out / (name + '.log'), 'w'); handles.append(h)
    p = subprocess.Popen(args, stdout=h, stderr=subprocess.STDOUT, start_new_session=True)
    procs.append(p); return p
try:
    # Refuse to reuse an existing private master.
    import socket
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', args.port)) == 0:
            raise RuntimeError('private replay port is occupied')
    spawn(['roscore', '-p', str(args.port)], 'master')
    for _ in range(100):
        try:
            if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/audit')[0] == 1: break
        except OSError: pass
        time.sleep(.1)
    rospy.set_param('/use_sim_time', True)
    rospy.init_node('soa_boundary_replay', disable_signals=True)
    ns = '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'
    for f in ['configs/planning/vehicle_info.yaml',
        'planning_ws/src/autoware_launch/config/ros1/planning/behavior_common.yaml',
        'planning_ws/src/autoware_behavior_path_planner/config/ros1/behavior_path_planner.yaml',
        'planning_ws/src/autoware_behavior_path_planner/config/ros1/drivable_area_expansion.yaml',
        'planning_ws/src/autoware_behavior_path_planner/config/ros1/scene_module_manager.yaml',
        'planning_ws/src/autoware_launch/config/ros1/planning/behavior_path_modules.yaml']:
        rosparam.upload_params(ns, yaml.safe_load((root/f).read_text()))
    rospy.set_param(ns+'/use_occupancy_grid', False)
    rospy.set_param(ns+'/forward_path_length', 400.0)
    rospy.set_param(ns+'/avoidance/target_filtering/unstable_classification_time', 0.0)
    rospy.set_param(ns+'/launch_modules', ['autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager'])
    clock = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
    scenario = rospy.Publisher('/planning/scenario_planning/scenario', Scenario, queue_size=1, latch=True)
    mode = rospy.Publisher('/system/operation_mode/state', OperationModeState, queue_size=1, latch=True)
    bpp_topic = '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id' if args.full_pipeline else '/replay/bpp_path'
    po_topic = '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory' if args.full_pipeline else '/replay/po_output'
    po_node = '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer' if args.full_pipeline else '/motion_planning/path_optimizer'
    stages = {}
    validation = []
    scene_checks = []
    processing_times = []
    po_publications = []
    def receive_validation(m):
        validation.append({key: (getattr(m,key).to_sec() if key=="stamp" else getattr(m,key)) for key in m.__slots__})
    subs_validation = rospy.Subscriber("/planning/validation_status",PlanningValidatorStatus,receive_validation,queue_size=100)
    def receive_scene_check(m):
        for status in m.status:
            if status.name == 'static_scene_check':
                values = {v.key:v.value for v in status.values}
                values['received_wall'] = time.monotonic()
                scene_checks.append(values)
    sub_scene_checks = rospy.Subscriber('/planning_validator/debug/static_scene_check',
      DiagnosticArray,receive_scene_check,queue_size=100)
    po_input = rospy.Publisher('/replay/po_input', PlanningPath, queue_size=1)
    def receive(m):
        records.append({'time':m.header.stamp.to_sec(), 'wall':time.monotonic(),
          'points':[point(p.point) for p in m.points],
          'left':[[p.x,p.y] for p in m.left_bound], 'right':[[p.x,p.y] for p in m.right_bound]})
        p = PlanningPath(header=m.header,points=[x.point for x in m.points],left_bound=m.left_bound,right_bound=m.right_bound)
        if not args.full_pipeline: po_input.publish(p)
    def receive_po(m):
        normalized=copy.copy(m);normalized.header=copy.copy(m.header);normalized.header.seq=0
        buffer=io.BytesIO();normalized.serialize(buffer)
        trajectories.append({'time':m.header.stamp.to_sec(),
          'sequence':m.header.seq,'crc32':zlib.crc32(buffer.getvalue()),
          'received':rospy.Time.now().to_sec(), 'wall':time.monotonic(),
          'points':[point(p) for p in m.points]})
    subs = [rospy.Subscriber(bpp_topic,PathWithLaneId,receive,queue_size=100),
      rospy.Subscriber('/rosout',Log,lambda m: logs.append((m.header.stamp.to_sec(),m.level,m.msg)),queue_size=10000)]
    subs.append(rospy.Subscriber(po_topic,Trajectory,receive_po,queue_size=100))
    subs.append(rospy.Subscriber(po_node+"/debug/processing_time_ms",Float64Stamped,lambda m: processing_times.append(m.data),queue_size=100))
    def receive_po_publication(m):
        sequence,crc32,before,after=map(int,m.data.split())
        po_publications.append({'sequence':sequence,'crc32':crc32,
          'time_ns':m.stamp.sec*1000000000+m.stamp.nanosec,'before_ns':before,'after_ns':after,
          'received_wall':time.monotonic()})
    subs.append(rospy.Subscriber(po_node+'/debug/publication_evidence',
      StringStamped,receive_po_publication,queue_size=100))
    def receive_planning_odometry(m):
        q=m.pose.pose.orientation
        planning_odometry.append({'time':m.header.stamp.to_sec(),'x':m.pose.pose.position.x,
          'y':m.pose.pose.position.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
          'v':m.twist.twist.linear.x})
    subs.append(rospy.Subscriber(po_node+'/debug/planning_odometry',
      Odometry,receive_planning_odometry,queue_size=100))
    def receive_scene(m):
        scenes.append({'time':m.header.stamp.to_sec(), 'time_ns':m.header.stamp.to_nsec(),
          'received':rospy.Time.now().to_sec(),
          'wall':time.monotonic(), 'frame':m.header.frame_id,
          'obstacles':[{'id':bytes(o.id.uuid).hex(),'polygon':[[p.x,p.y] for p in o.polygon.points],
            'hard':o.hard_margin,'soft':o.soft_margin} for o in m.obstacles]})
    subs.append(rospy.Subscriber('/planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene',
      StaticObstacleScene,receive_scene,queue_size=100))
    if not args.full_pipeline:
        spawn(['roslaunch',str(root/'planning_ws/install_isolated/share/tier4_planning_launch/launch/scenario_planning/motion_path.launch'),
          'config_root:='+str(root/'planning_ws/src/autoware_launch/config/ros1/planning'),
          'vehicle_param_path:='+str(root/'configs/planning/vehicle_info.yaml'),
          'input_path:=/replay/po_input','output_trajectory:=/replay/po_output'], 'motion_path')
    remaps = {'input/route':'/planning/mission_planning/route','input/vector_map':'/map/vector_map',
      'input/odometry':'/localization/kinematic_state','input/accel':'/localization/acceleration',
      'input/perception':'/perception/object_recognition/objects','input/scenario':'/planning/scenario_planning/scenario',
      'output/path':'/replay/bpp_path'}
    if not args.full_pipeline:
        node = spawn([str(root/'planning_ws/install_isolated/lib/autoware_behavior_path_planner/behavior_path_planner_full_node'),
          '__name:=behavior_path_planner','__ns:=/planning/scenario_planning/lane_driving/behavior_planning']+
          ['behavior_path_planner/'+k+':='+v for k,v in remaps.items()], 'bpp')
    else:
        import xml.etree.ElementTree as ET
        launch = ET.parse(root/'base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch').getroot()
        vals = {arg.get('name'):arg.get('value') for inc in launch.findall('include') for arg in inc.findall('arg')}
        defaults = {arg.get('name'):arg.get('default') for arg in launch.findall('arg')}
        vals['behavior_velocity_launch_modules'] = defaults['behavior_velocity_launch_modules_with_traffic_light']
        node = spawn(['roslaunch',str(root/'scripts/evaluation/static_obstacle_replay.launch'),
          'config_root:='+str(root/'planning_ws/src/autoware_launch/config/ros1/planning'),
          'vehicle_param_path:='+str(root/'configs/planning/vehicle_info.yaml')]+
          [key+':='+vals[key] for key in ['behavior_path_launch_modules','behavior_velocity_launch_modules','motion_velocity_launch_modules']], 'full_pipeline')
        for topic, message_type in [
                ('/planning/scenario_planning/lane_driving/behavior_planning/path', PlanningPath),
                ('/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path', PlanningPath),
                ('/planning/scenario_planning/lane_driving/trajectory', Trajectory),
                ('/planning/scenario_planning/velocity_smoother/trajectory', Trajectory),
                ('/planning/trajectory', Trajectory)]:
            stages[topic] = []
            def receive_stage(m, topic=topic):
                record = {'time':m.header.stamp.to_sec(),'received':rospy.Time.now().to_sec(),
                    'wall':time.monotonic(),'sequence':m.header.seq,
                    'points':[point(p) for p in m.points],
                    'acceleration':[getattr(p,'acceleration_mps2',0.0) for p in m.points]}
                if topic == '/planning/trajectory':
                    normalized = copy.copy(m)
                    normalized.header = copy.copy(m.header)
                    normalized.header.seq = 0
                    buf = io.BytesIO()
                    normalized.serialize(buf)
                    record['crc32'] = zlib.crc32(buf.getvalue())
                stages[topic].append(record)
            subs.append(rospy.Subscriber(topic,message_type,receive_stage,queue_size=100))
    events, event_times_ns, pubs = [], [], {}
    with rosbag.Bag(str(args.bag)) as bag:
        start = bag.get_start_time()
        end_time = min(bag.get_end_time(), start + args.duration)
        for topic,m,t in bag.read_messages(topics=list(remaps.values())[:5]+(['/tf','/tf_static','/perception/traffic_light_recognition/traffic_signals','/vehicle/status/velocity_status','/vehicle/status/steering_status','/system/operation_mode/state'] if args.full_pipeline else [])):
            if topic not in pubs: pubs[topic] = rospy.Publisher(topic,type(m),queue_size=20,latch=True)
            if t.to_sec()-start <= args.duration:
                events.append((topic,m,t.to_sec()))
                event_times_ns.append(t.to_nsec())
                if topic.endswith('kinematic_state'):
                    q=m.pose.pose.orientation
                    odometry.append({'time':t.to_sec(),'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
                      'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)), 'v':m.twist.twist.linear.x})
                if topic.endswith('/objects'):
                    observed.append({'time':t.to_sec(),'objects':[{'id':bytes(o.object_id.uuid).hex(),
                      'x':o.kinematics.initial_pose_with_covariance.pose.position.x,
                      'y':o.kinematics.initial_pose_with_covariance.pose.position.y,
                      'length':o.shape.dimensions.x,'width':o.shape.dimensions.y} for o in m.objects]})
    time.sleep(2)
    (out/'bpp_parameters.json').write_text(json.dumps(rospy.get_param(ns),indent=2))
    scenario_msg=Scenario(current_scenario=Scenario.LANEDRIVING,activating_scenarios=[Scenario.LANEDRIVING])
    mode_msg=OperationModeState(mode=OperationModeState.AUTONOMOUS,is_autoware_control_enabled=True)
    # Prime route/map/plugin caches with the first recorded snapshot. The bag normally starts
    # after the live nodes have already initialized. No later obstacle observation is used.
    first={}
    for topic,m,t in events:
        if topic not in first: first[topic]=m
    start_ns=rospy.Time.from_sec(start).to_nsec()
    def step_stamp(step):
        ns=start_ns+step*100000000
        return rospy.Time(ns//1000000000,ns%1000000000)
    warmup_steps=int(args.warmup*10)
    for i in range(warmup_steps):
        target=stages.get('/planning/trajectory',[]) if args.full_pipeline else trajectories
        prior_outputs=len(target)
        stamp=step_stamp(i-warmup_steps)
        clock.publish(Clock(stamp));scenario.publish(scenario_msg);mode.publish(mode_msg)
        for topic,m in first.items():
            if i and topic in ['/map/vector_map','/planning/mission_planning/route']:continue
            msg=copy.deepcopy(m)
            if hasattr(msg,'header'):msg.header.stamp=stamp
            if topic in ['/tf','/tf_static']:
                for tr in msg.transforms:tr.header.stamp=stamp
            pubs[topic].publish(msg)
        # A fixed delay can leave route/plugin initialization and old outputs
        # queued into the measured replay. Drain each initial snapshot through
        # the complete pipeline before starting the next simulated warmup tick.
        deadline=time.monotonic()+3.0
        while time.monotonic()<deadline:
            if len(target)>prior_outputs and target[-1]['received']>=stamp.to_sec()-.001:break
            if node.poll() is not None:raise RuntimeError('planning exited during warmup')
            time.sleep(.005)
    time.sleep(.25)  # finish the last warmup callback through the final validator
    records.clear();trajectories.clear();scenes.clear();planning_odometry.clear();logs.clear()
    for records_for_stage in stages.values():records_for_stage.clear()
    if not args.native_player: clock.publish(Clock(step_stamp(0)))
    scenario.publish(scenario_msg);mode.publish(mode_msg)
    if args.lockstep:
        cycle_wall=[]
        index=0
        end=events[-1][2]
        tick=start
        step=0
        while tick <= end+.001:
            began=time.monotonic()
            target=stages.get('/planning/trajectory',[]) if args.full_pipeline else trajectories
            prior_outputs=len(target)
            while index<len(events) and events[index][2]<=tick:
                topic,m,t=events[index];index+=1
                if args.warmup and topic in ['/map/vector_map','/planning/mission_planning/route'] and m is first[topic]:continue
                pubs[topic].publish(m)
            time.sleep(.015)  # drain subscriber input queues before triggering the planning timer
            clock.publish(Clock(step_stamp(step)))
            deadline=time.monotonic()+3.0
            while time.monotonic()<deadline:
                complete=len(target)>prior_outputs and target[-1]['received']>=tick-.001
                if complete:break
                if node.poll() is not None:raise RuntimeError('planning process exited')
                time.sleep(.002)
            cycle_wall.append({'time':tick,'wall_seconds':time.monotonic()-began,
                               'complete':len(target)>prior_outputs and target[-1]['received']>=tick-.001})
            step+=1
            tick=start+step*.1
    elif args.native_player:
        # Keep the input clock/player outside the recorder's Python GIL. Heavy
        # output callbacks must not stall /clock and collapse BPP timer ticks.
        playback_bag = out / 'input_replay.bag'
        with rosbag.Bag(str(playback_bag), 'w') as selected:
            selected.write('/replay/time_anchor', Clock(rospy.Time.from_sec(start)),
                           rospy.Time.from_sec(start))
            for (topic,m,t), stamp_ns in zip(events,event_times_ns):
                if args.warmup and topic in ['/map/vector_map','/planning/mission_planning/route'] and m is first[topic]:continue
                selected.write(topic,m,rospy.Time(stamp_ns//1000000000,stamp_ns%1000000000))
            selected.write('/replay/time_anchor', Clock(rospy.Time.from_sec(end_time)),
                           rospy.Time.from_sec(end_time))
        player = spawn(['rosbag','play','--quiet','--clock','--hz=200',
                        '--delay=0.2','--rate='+str(args.rate),str(playback_bag)],'native_player')
        if player.wait(timeout=(end_time-start)/args.rate+30):
            raise RuntimeError('native rosbag player failed')
    else:
        wall = time.monotonic()
        for topic,m,t in events:
            if args.warmup and topic in ['/map/vector_map','/planning/mission_planning/route'] and m is first[topic]:continue
            time.sleep(max(0, wall+(t-start)/args.rate-time.monotonic()))
            clock.publish(Clock(rospy.Time.from_sec(t))); pubs[topic].publish(m)
            if node.poll() is not None: raise RuntimeError('BPP exited')
    time.sleep(1)
    (out/'result.json').write_text(json.dumps({'bag_start':start,'records':records,'logs':logs,'trajectories':trajectories,'scenes':scenes,'odometry':odometry,'observed':observed,'bag':str(args.bag),'planning_odometry':planning_odometry,'stages':stages,'full_pipeline':args.full_pipeline,'validation':validation,'scene_checks':scene_checks,'lockstep':args.lockstep,'native_player':args.native_player,'cycle_wall':cycle_wall if args.lockstep else [],'po_processing_ms':processing_times,'po_publications':po_publications}))
    print('RESULT', out, 'outputs', len(records))
    for t,level,msg in logs:
        if 'SOA boundary' in msg or 'SOA rejected' in msg or level>=8: print(round(t-start,2),msg)
finally:
    # Complete cleanup even if the batch runner is interrupted a second time.
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    for p in reversed(procs):
        if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
    for p in reversed(procs):
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait()
    for h in handles: h.close()
