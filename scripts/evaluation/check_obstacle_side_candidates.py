"""Deterministic PO integration cases, isolated from the vehicle ROS master."""
import argparse,json,os,signal,socket,subprocess,time,xmlrpc.client,math,copy,io,zlib
from pathlib import Path
import rospy,rosparam,yaml
from rosgraph_msgs.msg import Clock
from autoware_internal_debug_msgs.msg import StringStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Point32
from autoware_planning_msgs.msg import Path as PlanningPath,PathPoint,StaticObstacle,StaticObstacleScene,Trajectory
root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=root/'planner_logs/obstacle_scene_regression/synthetic');parser.add_argument('--search-budget-ms',type=float);parser.add_argument('--require-stable-free-side',action='store_true');parser.add_argument('--only-free',action='store_true');args=parser.parse_args()
out=args.output;out.mkdir(parents=True,exist_ok=True)
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11475';os.environ['ROS_LOG_DIR']=str(out/'ros')
with socket.socket() as sock:
 if sock.connect_ex(('127.0.0.1',11475))==0:raise RuntimeError('port occupied')
procs=[];handles=[]
def spawn(cmd,name):
 h=(out/(name+'.log')).open('w');handles.append(h);p=subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
try:
 spawn(['roscore','-p','11475'],'master')
 for _ in range(100):
  try:
   if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/check')[0]==1:break
  except OSError:pass
  time.sleep(.05)
 rospy.set_param('/use_sim_time',True);rospy.init_node('candidate_check',disable_signals=True)
 ns='/optimizer'
 for f in ['configs/planning/vehicle_info.yaml','planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/common/nearest_search.param.yaml','planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml']:
  rosparam.upload_params(ns,yaml.safe_load((root/f).read_text()))
 rospy.set_param(ns+'/require_obstacle_scene',True)
 if args.search_budget_ms is not None:rospy.set_param(ns+'/obstacle_search_time_budget_ms',args.search_budget_ms)
 spawn(['rosrun','autoware_path_optimizer','path_optimizer_node','__name:=optimizer'],'po')
 clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
 odom=rospy.Publisher(ns+'/input/odometry',Odometry,queue_size=1,latch=True)
 path=rospy.Publisher(ns+'/input/path',PlanningPath,queue_size=1)
 scene=rospy.Publisher('/planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene',StaticObstacleScene,queue_size=1)
 received=[];sub=rospy.Subscriber(ns+'/output/path',Trajectory,lambda m:received.append(m),queue_size=100)
 publication_evidence={}
 def receive_evidence(m):
  seq,crc,before_ns,after_ns=map(int,m.data.split())
  publication_evidence[(seq,crc)]=(before_ns,after_ns,m.stamp.sec*1000000000+m.stamp.nanosec)
 evidence_sub=rospy.Subscriber(ns+'/debug/publication_evidence',StringStamped,receive_evidence,queue_size=100)
 time.sleep(1)
 results=[];previous_points=None
 for case,left,right,expected in [('both_open',4.5,-4.5,'either'),('left_blocked',1.8,-4.5,'right'),('right_blocked',4.5,-1.8,'left'),('fully_blocked',1.8,-1.8,'stop'),('small_unknown',4.5,-4.5,'small'),('grown_car',4.5,-4.5,'either'),('clock_reset',4.5,-4.5,'either')]:
  if args.only_free and case!='both_open':continue
  if case=='clock_reset':
   for i in range(110):
    preload=StaticObstacleScene();preload.header.frame_id='map';preload.header.stamp=rospy.Time.from_sec(500+i*.1);scene.publish(preload);time.sleep(.005)
   time.sleep(.1)
  before=len(received)
  for frame in range(12):
   # Reset before the independent UNKNOWN -> CAR growth scenario.
   base_time={'small_unknown':10,'grown_car':13,'clock_reset':1}.get(case,100+len(results)*3)
   stamp=rospy.Time.from_sec(base_time+frame*.1);clock.publish(Clock(stamp))
   ego=Odometry();ego.header.frame_id='map';ego.pose.pose.orientation.w=1;ego.twist.twist.linear.x=2;odom.publish(ego)
   msg=PlanningPath();msg.header.stamp=stamp;msg.header.frame_id='map'
   for i in range(111):
    p=PathPoint();p.pose.position.x=-5+i*.5;p.pose.orientation.w=1;p.longitudinal_velocity_mps=5;msg.points.append(p)
   msg.left_bound=[Point(x=-10,y=left),Point(x=60,y=left)];msg.right_bound=[Point(x=-10,y=right),Point(x=60,y=right)]
   obstacle=StaticObstacle();obstacle.hard_margin=.1;obstacle.soft_margin=.2
   half_width=.2 if case=='small_unknown' else 1.2
   obstacle.polygon.points=[Point32(x=x,y=y) for x,y in [(18,-half_width),(23,-half_width),(23,half_width),(18,half_width)]]
   obstacles=StaticObstacleScene(header=msg.header,obstacles=[obstacle]);scene.publish(obstacles);path.publish(msg);time.sleep(.12)
  time.sleep(.2)
  if len(received)==before:raise AssertionError(case+' no output')
  points=received[-1].points;near=min(points,key=lambda p:p.pose.position.x*p.pose.position.x+p.pose.position.y*p.pose.position.y)
  around=min(points,key=lambda p:abs(p.pose.position.x-20));y=around.pose.position.y
  stops=[p.pose.position.x for p in points if p.pose.position.x>=0 and abs(p.longitudinal_velocity_mps)<.01]
  stop_x=min(stops) if stops else None
  stopped=stop_x is not None and stop_x < 14.0
  passed=stopped if expected=='stop' else (not stopped and (1.1<abs(y)<2.2 if expected=='small' else abs(y)>2.2 if expected=='either' else y>2.2 if expected=='left' else y< -2.2))
  if case=='fully_blocked' and previous_points:
   # A failed optimization must keep the previously published spatial curve.
   previous_near=min(previous_points,key=lambda p:abs(p.pose.position.x-20))
   passed=passed and math.hypot(around.pose.position.x-previous_near.pose.position.x,
                               around.pose.position.y-previous_near.pose.position.y)<1e-6
  side_history=[1 if min(m.points,key=lambda p:abs(p.pose.position.x-20)).pose.position.y>0 else -1 for m in received[before:] if m.points and max(p.longitudinal_velocity_mps for p in m.points)>0.1]
  side_switches=sum(a!=b for a,b in zip(side_history,side_history[1:]))
  if args.require_stable_free_side and case=='both_open':passed=passed and side_switches==0
  matched=0
  for output in received[before:]:
   normalized=copy.copy(output);normalized.header=copy.copy(output.header);normalized.header.seq=0
   buffer=io.BytesIO();normalized.serialize(buffer)
   evidence=publication_evidence.get((output.header.seq,zlib.crc32(buffer.getvalue())))
   if evidence and evidence[0]<=evidence[1] and evidence[2]==output.header.stamp.to_nsec():matched+=1
  passed=passed and matched==len(received)-before
  results.append(dict(case=case,expected=expected,passed=passed,target_speed=near.longitudinal_velocity_mps,y_at_obstacle=y,stop_x=stop_x,side_switches=side_switches,publication_matches=matched,outputs=len(received)-before))
  previous_points=points
 (out/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
 if not all(r['passed'] for r in results):raise AssertionError('candidate integration failed')
finally:
 for p in reversed(procs):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in reversed(procs):
  try:p.wait(timeout=4)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for h in handles:h.close()
