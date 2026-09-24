"""Exercise installed PO while ego advances on its published path; ideal tracking, not MORAI."""
import argparse,json,math,os,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
import rosparam,rospy,yaml
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Point32
from autoware_planning_msgs.msg import Path as PlanningPath,PathPoint,StaticObstacle,StaticObstacleScene,Trajectory
root=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11475';os.environ['ROS_LOG_DIR']=str(a.output/'ros')
with socket.socket() as sock:assert sock.connect_ex(('127.0.0.1',11475))!=0,'private master occupied'
procs=[];handles=[];rows=[]
def spawn(command,name):
 h=(a.output/(name+'.log')).open('w');handles.append(h);procs.append(subprocess.Popen(command,stdout=h,stderr=h,start_new_session=True))
try:
 spawn(['roscore','-p','11475'],'master')
 for _ in range(100):
  try:
   if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/progression')[0]==1:break
  except OSError:pass
  time.sleep(.05)
 rospy.set_param('/use_sim_time',True);rospy.init_node('path_progression',disable_signals=True)
 for f in ['configs/planning/vehicle_info.yaml','planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/common/nearest_search.param.yaml','planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml']:
  rosparam.upload_params('/optimizer',yaml.safe_load((root/f).read_text()))
 rospy.set_param('/optimizer/require_obstacle_scene',True)
 spawn(['rosrun','autoware_path_optimizer','path_optimizer_node','__name:=optimizer'],'po')
 clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True);odom=rospy.Publisher('/optimizer/input/odometry',Odometry,queue_size=1,latch=True);path=rospy.Publisher('/optimizer/input/path',PlanningPath,queue_size=1);scene=rospy.Publisher('/planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene',StaticObstacleScene,queue_size=1)
 received=[];sub=rospy.Subscriber('/optimizer/output/path',Trajectory,received.append,queue_size=10)
 deadline=time.monotonic()+10
 while path.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.05)
 assert path.get_num_connections()>0
 x=y=yaw=0.0;last_stamp=0
 for tick in range(180):
  stamp=rospy.Time.from_sec(100+tick*.1);clock.publish(Clock(stamp));ego=Odometry();ego.header.stamp=stamp;ego.header.frame_id='map';ego.pose.pose.position.x=x;ego.pose.pose.position.y=y;ego.pose.pose.orientation.z=math.sin(yaw/2);ego.pose.pose.orientation.w=math.cos(yaw/2);ego.twist.twist.linear.x=3.5;odom.publish(ego);time.sleep(.015)
  msg=PlanningPath();msg.header.stamp=stamp;msg.header.frame_id='map'
  for i in range(241):
   q=PathPoint();q.pose.position.x=-10+i*.5;q.pose.orientation.w=1;q.longitudinal_velocity_mps=5;msg.points.append(q)
  msg.left_bound=[Point(x=-10,y=4.5),Point(x=110,y=4.5)];msg.right_bound=[Point(x=-10,y=-4.5),Point(x=110,y=-4.5)]
  obs=StaticObstacle();obs.hard_margin=.1;obs.soft_margin=.2;obs.polygon.points=[Point32(x=px,y=py) for px,py in [(18,-1.2),(23,-1.2),(23,1.2),(18,1.2)]]
  scene.publish(StaticObstacleScene(header=msg.header,obstacles=[obs] if x<32 else []));time.sleep(.01);before=len(received);begin=time.monotonic();path.publish(msg)
  while len(received)==before and time.monotonic()-begin<3:time.sleep(.005)
  assert len(received)>before,'missing output'
  tr=received[-1];assert tr.header.stamp==stamp,'stale output'
  pts=tr.points;i=min(range(len(pts)),key=lambda j:(pts[j].pose.position.x-x)**2+(pts[j].pose.position.y-y)**2);v=pts[i].longitudinal_velocity_mps
  rows.append(dict(tick=tick,x=x,y=y,yaw=yaw,target_v=v,wall_ms=(time.monotonic()-begin)*1000,obstacle_active=x<32))
  assert v>.01,'unexpected stop at '+str(rows[-1])
  if x>=50:break
  distance=0.;j=i
  while j+1<len(pts) and distance<.35:
   first=pts[j].pose.position;second=pts[j+1].pose.position;ds=math.hypot(second.x-first.x,second.y-first.y)
   if ds>1e-6 and distance+ds>=.35:
    ratio=(.35-distance)/ds;x=first.x+ratio*(second.x-first.x);y=first.y+ratio*(second.y-first.y);yaw=math.atan2(second.y-first.y,second.x-first.x);break
   distance+=ds;j+=1
  else:raise AssertionError('trajectory too short')
 assert x>=50 and abs(y)<.3,'did not finish return to reference'
 near=[r for r in rows if 18<=r['x']<=23];assert near and all(abs(r['y'])>2.245 for r in near),'insufficient obstacle clearance'
 print(json.dumps(dict(passed=True,frames=len(rows),end_x=x,end_y=y,stops=sum(r['target_v']<.01 for r in rows)),indent=2))
finally:
 (a.output/'frames.json').write_text(json.dumps(rows,indent=2))
 for p in reversed(procs):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for h in handles:h.close()
