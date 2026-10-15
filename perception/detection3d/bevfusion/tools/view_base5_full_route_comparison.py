#!/usr/bin/env python3
"""Visual-only whole-route comparison on immutable, shared sensor frames.

This tool does not create accuracy GT from missing annotations, run planning,
or change production ROS. A private ROS master owns all visualization nodes.
"""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import signal
import socket
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / 'docker/bevfusion')]


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(8 * 1024**2), b''):
            h.update(chunk)
    return h.hexdigest()


def audit(args):
    import rosbag
    if args.output.exists():
        raise FileExistsError(args.output)
    records = []
    for path, scale in zip(args.bags, args.velocity_scales):
        with rosbag.Bag(str(path)) as bag:
            metadata = bag.get_type_and_topic_info().topics
            ego = []; count = nonempty = maximum = 0
            for topic, m, t in bag.read_messages(topics=['/Ego_topic', '/Object_topic']):
                if topic == '/Ego_topic':
                    ego.append([t.to_sec(), m.position.x, m.position.y,
                                m.velocity.x * scale, m.velocity.y * scale])
                else:
                    n = len(m.npc_list) + len(m.pedestrian_list) + len(m.obstacle_list)
                    count += 1; nonempty += n > 0; maximum = max(maximum, n)
            a = np.asarray(ego)
            dt = np.diff(a[:, 0]); ds = np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1)
            speed = np.linalg.norm(a[:, 3:5], axis=1)
            speed = (speed[1:] + speed[:-1]) / 2
            valid = (dt > 0) & (dt < .25) & (ds < 5) & (speed > 1)
            records.append(dict(path=str(path.resolve()), bag_sha256=digest(path),
                duration_sec=bag.get_end_time()-bag.get_start_time(),
                ego_hz=(len(a)-1)/(a[-1, 0]-a[0, 0]),
                ego_interval_ms=dict(zip(('p50', 'p95', 'p99', 'max'),
                    (np.percentile(dt, [50, 95, 99, 100])*1000).tolist())),
                gaps_over100ms=int((dt > .1).sum()),
                trajectory_distance_m=float(ds[ds < 5].sum()),
                explicit_velocity_scale_to_mps=scale,
                velocity_integral_to_position_ratio=float((speed[valid]*dt[valid]).sum()/ds[valid].sum()),
                object_messages=count, object_nonempty_messages=nonempty,
                maximum_object_count=maximum,
                topics={k:dict(count=v.message_count, type=v.msg_type) for k,v in metadata.items()}))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(bags=records,
        ratio_scope='Moving segments >1m/s; exclude >=250ms gaps and >=5m jumps; not an independent simulator realtime-factor measurement.'), indent=2)+'\n')
    print(json.dumps({r['path']:dict(ego_hz=r['ego_hz'], gaps=r['gaps_over100ms'],
        ratio=r['velocity_integral_to_position_ratio']) for r in records}), flush=True)


def verify(args):
    """Check every extracted payload before reusing a prior visual-only package."""
    manifest = json.loads((args.inputs/'manifest.json').read_text())
    from app.sensor4_calibration import load_sensor4_calibration
    calibration=load_sensor4_calibration(manifest['sensor_config'],
        ['camera_front','camera_front_left','camera_front_right','camera_back'])
    if calibration['config_sha256']!=manifest['sensor_sha256']:
        raise ValueError('Historical calibration file changed')
    info = pickle.loads((args.inputs/'infos_val.pkl').read_bytes())
    rows = info['data_list']
    if len(rows) != manifest['selected_frames'] or len(rows) != len(manifest['inputs']):
        raise ValueError('Input frame count mismatch')
    for row, provenance in zip(rows, manifest['inputs']):
        if row['token'] != provenance['token'] or row['timestamp'] != provenance['stamp']:
            raise ValueError('Immutable input order mismatch')
        if row['instances']:
            raise ValueError('Visual-only package must not claim synthetic accuracy GT')
        if digest(args.inputs/row['lidar_points']['lidar_path']) != provenance['points_sha256']:
            raise ValueError('Point payload mismatch')
        if list(row['images'])!=[c['frame_id'] for c in calibration['cameras']]:
            raise ValueError('Camera order differs from calibration')
        for camera in calibration['cameras']:
            name=camera['frame_id'];image=row['images'][name]
            for k in ('cam2img','lidar2cam','lidar2img','cam2lidar'):
                if not np.allclose(image[k],camera[k],atol=1e-10,rtol=0):
                    raise ValueError('Camera projection metadata changed')
            if (image['height'],image['width'])!=(camera['height'],camera['width']):
                raise ValueError('Camera native resolution changed')
            if digest(args.inputs/image['img_path']) != provenance['camera_sha256'][name]:
                raise ValueError('Image payload mismatch')
    if args.pilot_info:
        if args.pilot_info.exists():
            raise FileExistsError(args.pilot_info)
        # Original paths remain relative to the SAME input root.
        args.pilot_info.write_bytes(pickle.dumps(dict(info,
            data_list=[rows[i] for i in np.unique(np.rint(np.linspace(0,len(rows)-1,8)).astype(int))]), protocol=4))
    print(json.dumps(dict(event='all_inputs_verified',frames=len(rows),
        info_sha256=digest(args.inputs/'infos_val.pkl'),
        original_bag=manifest['raw_bag'], accuracy_gt='NONE')), flush=True)


def prepare(args):
    """Extract all synchronized LiDAR frames without inventing accuracy labels."""
    import cv2
    import rosbag
    from app.sensor4_calibration import load_sensor4_calibration
    from prepare_base5_rosbag_quick import nearest, xyzi
    if args.output.exists():raise FileExistsError(args.output)
    calibration=load_sensor4_calibration(args.sensor,
        ['camera_front','camera_front_left','camera_front_right','camera_back'])
    topics=[c['topic'] for c in calibration['cameras']]+['/sensor/lidar1']
    messages={t:[] for t in topics}
    with rosbag.Bag(str(args.bag)) as bag:
        duration=bag.get_end_time()-bag.get_start_time()
        for topic,msg,t in bag.read_messages(topics=topics):
            stamp=msg.header.stamp.to_sec() or t.to_sec()
            messages[topic].append((stamp,msg))
    for topic in messages:
        messages[topic]=sorted(dict(messages[topic]).items())
    args.output.mkdir(parents=True)
    rows=[];fingerprints=[];skipped=[]
    for index,(stamp,lidar) in enumerate(messages['/sensor/lidar1']):
        images=[nearest(messages[c['topic']],stamp,.05) for c in calibration['cameras']]
        if any(im is None for im in images):skipped.append(index);continue
        if lidar.header.frame_id!='lidar1':raise ValueError('Expected historical lidar1 frame')
        token=f'full_route_{index:05d}';folder=args.output/'data'/token;folder.mkdir(parents=True)
        points=xyzi(lidar)
        (folder/'points.bin').write_bytes(points.astype('<f4').tobytes())
        camera_info={};hashes={};offsets={}
        for camera,(image_stamp,msg) in zip(calibration['cameras'],images):
            name=camera['frame_id'];data=bytes(msg.data)
            decoded=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
            if decoded is None or decoded.shape[:2]!=(camera['height'],camera['width']):
                raise ValueError('Actual camera dimensions differ from explicit calibration')
            path=Path('data')/token/(name+'.jpg');(args.output/path).write_bytes(data)
            camera_info[name]=dict(img_path=str(path),height=camera['height'],width=camera['width'],
                **{k:camera[k].tolist() for k in ('cam2img','lidar2cam','lidar2img','cam2lidar')})
            hashes[name]=hashlib.sha256(data).hexdigest();offsets[name]=image_stamp-stamp
        rows.append(dict(token=token,sample_idx=token,scene_id='full_route',scene_group_id='full_route',
            timestamp=stamp,raw_lidar_index=index,images=camera_info,
            lidar_points=dict(lidar_path=str(Path('data')/token/'points.bin'),num_pts_feats=4),instances=[]))
        fingerprints.append(dict(token=token,stamp=stamp,points_sha256=digest(folder/'points.bin'),
            camera_sha256=hashes,camera_offsets_sec=offsets))
    if not rows:raise ValueError('No synchronized route frames')
    info=dict(metainfo=dict(classes=('vehicle','pedestrian','obstacle'),dataset='full_bag_visual_only',box_dim=7),data_list=rows)
    (args.output/'infos_val.pkl').write_bytes(pickle.dumps(info,protocol=4))
    (args.output/'manifest.json').write_text(json.dumps(dict(raw_bag=str(args.bag.resolve()),
        bag_sha256=digest(args.bag),selected_frames=len(rows),input_frames=len(messages['/sensor/lidar1']),
        indexed_duration_sec=duration,sensor_config=str(args.sensor.resolve()),sensor_sha256=calibration['config_sha256'],
        gt_reference='NONE: visual-only sensor package. No AP/recall claim.',
        calibration_note='Explicit historical Sensor_4 contract; no recorded tf_static to independently verify mounting.',
        skipped_camera_sync_indices=skipped,inputs=fingerprints),indent=2)+'\n')
    print(json.dumps(dict(event='prepared',frames=len(rows),skipped_camera_sync=len(skipped))),flush=True)


def filtered(row, thresholds, common=False):
    from analyze_base5_runtime_effects import delivered
    detections = []
    for b, score, label in zip(row['pred_boxes'],row['pred_scores'],row['pred_labels']):
        detections.append(dict(center=[b[0],b[1],b[2]+b[5]/2],size=b[3:6],
            yaw=b[6],score=score,label=label,class_name=('vehicle','pedestrian','obstacle')[label]))
    result = delivered(detections, thresholds)
    # Apply the common observation ROI AFTER the unchanged delivery/NMS.
    if common:
        result = [d for d in result if -30 <= d['center'][0] <= 30
                  and -30 <= d['center'][1] <= 30
                  and -3 <= d['center'][2]-d['size'][2]/2 <= 2]
    return result


def markers(detections, stamp, namespace, color):
    import rospy
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    result = MarkerArray()
    clear = Marker(); clear.header.frame_id='lidar1';clear.header.stamp=stamp
    clear.action=Marker.DELETEALL;result.markers.append(clear)
    for i,d in enumerate(detections):
        box = Marker();box.header=clear.header;box.ns=namespace;box.id=i*2
        box.action=Marker.ADD;box.type=Marker.LINE_LIST;box.pose.orientation.w=1
        box.scale.x=.06;box.color.r,box.color.g,box.color.b=color;box.color.a=1
        box.lifetime=rospy.Duration(.5)
        x,y,z=d['center'];l,w,h=d['size'];c,s=math.cos(d['yaw']),math.sin(d['yaw'])
        corners=[Point(x+c*a-s*b,y+s*a+c*b,z+k) for k in (-h/2,h/2)
                 for a,b in ((-l/2,-w/2),(l/2,-w/2),(l/2,w/2),(-l/2,w/2))]
        for a,b in ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)):
            box.points.extend([corners[a],corners[b]])
        result.markers.append(box)
        label = Marker();label.header=clear.header;label.ns=namespace;label.id=i*2+1
        label.action=Marker.ADD;label.type=Marker.TEXT_VIEW_FACING;label.pose.orientation.w=1
        label.pose.position=Point(x,y,z+h/2+.2);label.scale.z=.35
        label.color=box.color;label.lifetime=box.lifetime
        label.text=f"{namespace} {d['class_name']} {d['score']:.2f}"
        result.markers.append(label)
    return result


def export(args):
    import rosbag
    import rospy
    from compare_base5_common30 import validate_payload
    if args.output.exists():
        raise FileExistsError(args.output)
    infos=pickle.loads((args.inputs/'infos_val.pkl').read_bytes())['data_list']
    reports = {}; identity = None
    with rosbag.Bag(str(args.output),'w') as bag:
        for name,path in zip(('E25','E9'),args.predictions):
            with gzip.open(path,'rt') as f: payload=json.load(f)
            validate_payload(payload)
            if payload['name']!=name or payload['info_sha256']!=digest(args.inputs/'infos_val.pkl'):
                raise ValueError('Wrong model role or prepared sensor infos')
            if len(payload['rows']) != len(infos):raise ValueError('Incomplete route inference')
            if identity is not None and payload['gt_identity'] != identity:raise ValueError('Frame/GT mismatch')
            identity=payload['gt_identity']
            common_counts=[];full_counts=[]
            for info,row in zip(infos,payload['rows']):
                if info['token'] != row['token']:raise ValueError('Different frame ordering')
                stamp=rospy.Time.from_sec(info['timestamp'])
                common=filtered(row,args.thresholds,True);full=filtered(row,args.thresholds)
                color=(.2,1.,.2) if name=='E25' else (0.,.75,1.)
                bag.write(f'/full_route_compare/{name.lower()}/common30',markers(common,stamp,name,color),stamp)
                if name=='E9':
                    # Disable by default: E25 does NOT cover this larger spatial ROI.
                    bag.write('/full_route_compare/e9/full_range',markers(full,stamp,'E9_full',(.95,.6,.05)),stamp)
                common_counts.append([sum(d['label']==j for d in common) for j in range(3)])
                full_counts.append([sum(d['label']==j for d in full) for j in range(3)])
            reports[name]=dict(checkpoint_sha256=payload['checkpoint_sha256'],
                inference_precision=payload['inference_precision'],frames=len(infos),
                common30_instance_frames=np.asarray(common_counts).sum(axis=0).tolist(),
                full_range_instance_frames=np.asarray(full_counts).sum(axis=0).tolist(),
                common30_nonempty_frames=sum(sum(v)>0 for v in common_counts))
    args.output.with_suffix('.json').write_text(json.dumps(dict(models=reports,
        classes=['vehicle','pedestrian','obstacle'],thresholds=args.thresholds,nms_iou=.5,
        scope='Visual-only detection replay, NOT AP/FP/recall or closed-loop safety. Instance-frames are not unique objects.',
        common_scope='LiDAR-frame center box X/Y +/-30m, Z -3..2m, not radial30m.',
        gt='None. Map background and empty Object_topic are not complete negative ground truth.',
        marker_bag_sha256=digest(args.output)),indent=2)+'\n')
    print(json.dumps(reports),flush=True)


def review(args):
    """Original-pixel projections for manual review, never accuracy labels."""
    import cv2
    from render_base5_far_review import draw_box
    if args.output.exists():raise FileExistsError(args.output)
    infos=pickle.loads((args.inputs/'infos_val.pkl').read_bytes())['data_list']
    payloads=[]
    from compare_base5_common30 import validate_payload
    for name,path in zip(('E25','E9'),args.predictions):
        with gzip.open(path,'rt') as f:payloads.append(json.load(f))
        validate_payload(payloads[-1])
        if payloads[-1]['name']!=name or payloads[-1]['info_sha256']!=digest(args.inputs/'infos_val.pkl'):
            raise ValueError('Wrong review model role or prepared sensor infos')
    if any(len(p['rows'])!=len(infos) for p in payloads):raise ValueError('Incomplete route')
    if payloads[0]['gt_identity']!=payloads[1]['gt_identity']:raise ValueError('Different route tokens')
    if any([r['token'] for r in p['rows']]!=[r['token'] for r in infos] for p in payloads):
        raise ValueError('Different ordered route inputs')
    delivered_rows=[[filtered(r,args.thresholds,True) for r in p['rows']] for p in payloads]
    indices=list(np.unique(np.rint(np.linspace(0,len(infos)-1,6)).astype(int)))
    # Add duration-separated high-count disagreement cases, clearly selected.
    ranked=sorted(range(len(infos)),key=lambda i:abs(len(delivered_rows[1][i])-len(delivered_rows[0][i])),reverse=True)
    extra=[]
    for i in ranked:
        if any(abs(infos[i]['timestamp']-infos[j]['timestamp'])<8 for j in indices+extra):continue
        extra.append(i)
        if len(extra)==6:break
    args.output.mkdir(parents=True);entries=[]
    for n,i in enumerate(sorted(indices+extra)):
        info=infos[i];tiles=[]
        for camera,meta in info['images'].items():
            image=cv2.imread(str(args.inputs/meta['img_path']))
            if image is None:raise ValueError('Original JPEG unavailable')
            for model,color in ((0,(40,255,40)),(1,(255,190,0))):
                for d in delivered_rows[model][i]:
                    draw_box(image,[*d['center'],*d['size'],d['yaw']],meta['lidar2img'],color,bottom=False)
            factor=min(768/image.shape[1],432/image.shape[0])
            resized=cv2.resize(image,(round(image.shape[1]*factor),round(image.shape[0]*factor)))
            tile=np.zeros((432,768,3),np.uint8);x=(768-resized.shape[1])//2;y=(432-resized.shape[0])//2
            tile[y:y+resized.shape[0],x:x+resized.shape[1]]=resized
            cv2.putText(tile,camera,(12,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
            tiles.append(tile)
        header=np.zeros((70,1536,3),np.uint8)
        text=f"{info['token']} t={info['timestamp']-infos[0]['timestamp']:.1f}s | GREEN E25 / CYAN E9 | common +/-30m"
        cv2.putText(header,text,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
        counts=[[sum(d['label']==j for d in rows[i]) for j in range(3)] for rows in delivered_rows]
        cv2.putText(header,f"V/P/O instance counts: E25 {counts[0]} / E9 {counts[1]} | NOT an accuracy comparison",
            (12,53),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        path=args.output/f'{n:02d}_{info["token"]}.png'
        if not cv2.imwrite(str(path),np.vstack([header,np.hstack(tiles[:2]),np.hstack(tiles[2:])])):
            raise ValueError('Review output write failed')
        entries.append(dict(image=str(path),frame_index=int(i),token=info['token'],stamp=info['timestamp'],
            selection='even temporal spacing' if i in indices else 'duration-separated largest count disagreement',
            common_counts_vpo=counts))
    (args.output/'index.json').write_text(json.dumps(entries,indent=2)+'\n')
    print(json.dumps(dict(event='reviews_saved',images=len(entries))),flush=True)


def view(args):
    import yaml
    if args.output.exists():raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
    env=os.environ.copy();env.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',
        ROS_HOME=str(args.output.resolve()/'ros_home'),ROS_LOG_DIR=str(args.output.resolve()/'ros_logs'),
        DISABLE_ROS1_EOL_WARNINGS='1')
    env.pop('ROS_HOSTNAME',None)
    cfg=yaml.safe_load((ROOT/'base_ws/src/perception/bevfusion_ros/rviz/bevfusion_zero_shot.rviz').read_text())
    cfg['Preferences']['PromptSaveOnExit']=False
    manager=cfg['Visualization Manager'];manager['Global Options']['Fixed Frame']='lidar1'
    displays=[d for d in manager['Displays'] if d.get('Class') in ('rviz/PointCloud2','rviz/Image')]
    for name,topic,enabled in [('E25 green: common +/-30m','/full_route_compare/e25/common30',True),
        ('E9 cyan: common +/-30m','/full_route_compare/e9/common30',True),
        ('E9 orange: full range ONLY (not E25 comparison)','/full_route_compare/e9/full_range',False)]:
        displays.append({'Class':'rviz/MarkerArray','Name':name,'Enabled':enabled,'Value':enabled,
            'Marker Topic':topic,'Queue Size':2,'Namespaces':{}})
    manager['Displays']=displays
    manager['Views']['Current'].update(Distance=85.,Pitch=1.25,Yaw=math.pi/2,
        **{'Target Frame':'<Fixed Frame>','Focal Point':{'X':0.,'Y':0.,'Z':0.}})
    manager['Views']['Current']['Class']='rviz/Orbit'
    manager['Views']['Saved']=[dict(manager['Views']['Current'],Name='Common +/-30m'),
        dict(manager['Views']['Current'],Name='E9 full range +/-100m',Distance=250.)]
    cfg['Window Geometry'].update(Width=1600,Height=900,X=40,Y=40)
    rviz=args.output/'comparison.rviz';rviz.write_text(yaml.safe_dump(cfg,sort_keys=False))
    (args.output/'session.json').write_text(json.dumps(dict(master_uri=env['ROS_MASTER_URI'],
        raw=str(args.bag.resolve()),markers=str(args.markers.resolve()),rate=args.rate,
        scope='Isolated recorded detection visualization; no live detector/planner/controller.'),indent=2)+'\n')
    processes=[];logs=[]
    def start(name,command):
        log=(args.output/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        start('roscore',['roscore','-p',str(port)])
        for _ in range(100):
            check=subprocess.run(['rosparam','list'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            if check.returncode==0:break
            time.sleep(.1)
        else:raise RuntimeError('Private ROS master unavailable')
        subprocess.run(['rosparam','set','/use_sim_time','true'],env=env,check=True)
        from app.sensor4_calibration import load_sensor4_calibration
        from tf.transformations import quaternion_from_matrix
        calibration=load_sensor4_calibration(ROOT/'configs/sensors/Sensor_4.json')
        transform=calibration['base_from_lidar']
        values=[*transform[:3,3],*quaternion_from_matrix(transform)]
        start('sensor_tf',['rosrun','tf2_ros','static_transform_publisher',*[str(v) for v in values],
            calibration['base_frame'],calibration['lidar_frame']])
        start('rviz',['rviz','-d',str(rviz)])
        time.sleep(3)
        play=start('play',['rosbag','play','--clock','--loop','--rate',str(args.rate),
            str(args.bag),str(args.markers),'--topics','/sensor/lidar1',
            '/sensor/camera/front/compressed','/sensor/camera/back/compressed',
            '/sensor/camera/front_left/compressed','/sensor/camera/front_right/compressed',
            '/full_route_compare/e25/common30','/full_route_compare/e9/common30','/full_route_compare/e9/full_range'])
        print(json.dumps(dict(event='view_started',master_uri=env['ROS_MASTER_URI'],
            controller='NONE',view_pid=processes[-2].pid,play_pid=play.pid)),flush=True)
        while all(p.poll() is None for p in processes):time.sleep(.5)
    except KeyboardInterrupt:pass
    finally:
        for p in reversed(processes):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in reversed(processes):
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
        for log in logs:log.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    a=sub.add_parser('audit');a.add_argument('--bags',type=Path,nargs='+',required=True)
    a.add_argument('--velocity-scales',type=float,nargs='+',required=True);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('verify');a.add_argument('--inputs',type=Path,required=True);a.add_argument('--pilot-info',type=Path)
    a=sub.add_parser('prepare');a.add_argument('--bag',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    a.add_argument('--sensor',type=Path,default=ROOT/'configs/sensors/Sensor_4.json')
    a=sub.add_parser('export');a.add_argument('--inputs',type=Path,required=True)
    a.add_argument('--predictions',type=Path,nargs=2,required=True,help='E25 then E9')
    a.add_argument('--thresholds',type=float,nargs=3,default=[.4,.15,.1]);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('review');a.add_argument('--inputs',type=Path,required=True)
    a.add_argument('--predictions',type=Path,nargs=2,required=True,help='E25 then E9')
    a.add_argument('--thresholds',type=float,nargs=3,default=[.4,.15,.1]);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('view');a.add_argument('--bag',type=Path,required=True);a.add_argument('--markers',type=Path,required=True)
    a.add_argument('--output',type=Path,required=True);a.add_argument('--rate',type=float,default=.75)
    args=p.parse_args()
    if args.action=='audit' and len(args.bags)!=len(args.velocity_scales):p.error('Explicit per-bag velocity scales required')
    if args.action=='view' and args.rate<=0:p.error('Positive playback rate required')
    globals()[args.action](args)
