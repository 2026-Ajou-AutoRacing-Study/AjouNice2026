#!/usr/bin/env python3
"""Record real annotated front frames and read-only telemetry, never drive a car.

The caller owns this process. SIGINT/SIGTERM close ffmpeg's input and finalize
MP4. The fixed wall-time CFR clock repeats the last image explicitly; it never
refreshes its camera stamp. No ROS publisher, native API, sensor or control call
is made. Run with system Python after sourcing the ROS workspace.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import threading
import time


ROOT = Path(__file__).resolve().parents[2]
WIDTH, IMAGE_HEIGHT, HEIGHT = 1280, 720, 960
TOPICS = {
    'image': '/traffic_light_node/debug_image/compressed',
    'runtime': '/traffic_light_node/debug_runtime',
    'ego': '/Ego_topic', 'command': '/ctrl_cmd',
    'odometry': '/localization/kinematic_state',
    'trajectory': '/planning/trajectory',
    'planner_decision': '/perception/traffic_light_recognition/planner_decision_debug',
}
MAX_JPEG_BYTES = 16 * 1024 * 1024
MAX_TRAJECTORY_POINTS = 10000


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='microseconds')


def clean(value):
    """Preserve unknowns as null, not NaN/Infinity or invented zero values."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(item) for item in value]
    return value


def stamp_ns(stamp):
    if stamp is None:
        return None
    sec = getattr(stamp, 'secs', getattr(stamp, 'sec', None))
    nsec = getattr(stamp, 'nsecs', getattr(stamp, 'nanosec', None))
    if type(sec) is int and type(nsec) is int and sec >= 0 and 0 <= nsec < 10**9:
        result = sec * 10**9 + nsec
        return result if result > 0 else None
    return None


def message_stamp(message):
    return stamp_ns(getattr(getattr(message, 'header', None), 'stamp', None))


def validate_output(value, root=ROOT):
    path = Path(value).expanduser().absolute()
    if path.exists() or path.is_symlink():
        raise ValueError('Output must be a fresh directory, never an existing recording')
    path = path.resolve()
    parent = (Path(root) / 'artifacts/evaluation').resolve()
    try:
        relative = path.relative_to(parent)
    except ValueError as error:
        raise ValueError('Output must be below artifacts/evaluation') from error
    if not relative.parts:
        raise ValueError('A new child directory is required')
    return path


class Latest:
    """One immutable latest value per topic; callbacks never decode or write."""
    def __init__(self):
        self.lock = threading.Lock()
        self.values = {}
        self.counts = Counter()
        self.errors = Counter()

    def put(self, key, payload, source_ns, ros_ns, mono):
        with self.lock:
            self.counts[key] += 1
            self.values[key] = dict(sequence=self.counts[key], payload=payload,
                source_stamp_ns=source_ns, receipt_ros_ns=ros_ns,
                receipt_monotonic=mono)

    def error(self, key):
        with self.lock:
            self.errors[key] += 1

    def snapshot(self):
        with self.lock:
            return dict(self.values)

    def counters(self):
        with self.lock:
            return dict(received=dict(self.counts), callback_errors=dict(self.errors))


def timing(record, now_mono, now_ros_ns):
    if record is None:
        return dict(sequence=None, source_stamp_ns=None, receipt_ros_ns=None,
                    receipt_monotonic=None, receipt_age_sec=None, source_age_sec=None)
    source = record['source_stamp_ns']
    return dict(sequence=record['sequence'], source_stamp_ns=source,
        receipt_ros_ns=record['receipt_ros_ns'], receipt_monotonic=record['receipt_monotonic'],
        receipt_age_sec=now_mono-record['receipt_monotonic'],
        source_age_sec=(now_ros_ns-source)/1e9 if source and now_ros_ns > 0 else None)


def freshness(record, now_mono, now_ros_ns, stale_sec=.5):
    ages = timing(record, now_mono, now_ros_ns)
    if record is None:
        return 'WAITING FOR DEBUG IMAGE', ages
    if ages['source_age_sec'] is None:
        return 'UNVERIFIED CAMERA TIME', ages
    if ages['receipt_age_sec'] < 0 or ages['source_age_sec'] < -.05:
        return 'CLOCK REGRESSION / FUTURE IMAGE', ages
    if ages['receipt_age_sec'] > stale_sec or ages['source_age_sec'] > stale_sec:
        return 'STALE - LAST IMAGE IS NOT CURRENT SIGNAL', ages
    return 'LIVE ANNOTATED CAMERA', ages


def planned_speeds(trajectory, odometry, now_mono, max_age=1.):
    """Nearest + arc-length-ahead speeds in an explicitly shared map frame.

    Never use raw simulator Ego positions as if they were map/localization.
    These are planned speeds, not actual speed, an acceleration or a GO verdict.
    """
    if trajectory is None or odometry is None:
        return dict(available=False, reason='trajectory_or_map_odometry_missing')
    if any(not 0 <= now_mono-item['receipt_monotonic'] <= max_age for item in (trajectory, odometry)):
        return dict(available=False, reason='trajectory_or_map_odometry_stale')
    route, pose = trajectory['payload'], odometry['payload']
    if not route['frame_id'] or route['frame_id'].strip('/') != pose['frame_id'].strip('/'):
        return dict(available=False, reason='frame_mismatch_no_transform_attempted')
    points = route['points']
    if not points or not all(isinstance(pose.get(key), (int, float)) for key in ('x', 'y')):
        return dict(available=False, reason='empty_or_nonfinite_geometry')
    valid = [(index, point) for index, point in enumerate(points)
             if all(isinstance(value, (int, float)) and math.isfinite(value) for value in point)]
    if len(valid) != len(points):
        return dict(available=False, reason='nonfinite_trajectory')
    index = min(range(len(points)), key=lambda i: math.hypot(points[i][0]-pose['x'], points[i][1]-pose['y']))
    distance = math.hypot(points[index][0]-pose['x'], points[index][1]-pose['y'])
    if distance > 10.:
        return dict(available=False, reason='map_pose_over_10m_from_trajectory', nearest_distance_m=distance)
    result = dict(available=True, reason='nearest_map_pose_then_forward_arc_length',
                  frame_id=route['frame_id'], nearest_index=index, nearest_distance_m=distance,
                  nearest_mps=points[index][2], ahead_5m_mps=None, ahead_10m_mps=None)
    arc = 0.
    for before, after in zip(points[index:], points[index+1:]):
        arc += math.hypot(after[0]-before[0], after[1]-before[1])
        for threshold in (5, 10):
            key = 'ahead_%dm_mps' % threshold
            if result[key] is None and arc >= threshold:
                result[key] = after[2]
    return result


def frame_evidence(index, fps, start, now_mono, now_ros_ns, snapshot, image_record,
                   repeated, decode_error=None):
    state, ages = freshness(image_record, now_mono, now_ros_ns)
    if decode_error:
        state = 'DECODE ERROR - LAST IMAGE IS NOT CURRENT SIGNAL'
    evidence = dict(frame_index=index, pts_sec=index/fps,
        scheduled_monotonic=start+index/fps, write_monotonic=now_mono,
        schedule_lateness_sec=now_mono-(start+index/fps), write_ros_ns=now_ros_ns,
        utc=utc_now(), image=ages, image_status=state, image_repeated=repeated,
        decode_error=decode_error, telemetry={})
    for key in ('runtime', 'ego', 'command', 'odometry', 'trajectory'):
        record = snapshot.get(key)
        evidence['telemetry'][key] = timing(record, now_mono, now_ros_ns)
        if record is not None:
            # Full runtime JSON retains raw/final/hold/vote provenance. Geometry
            # stays bounded in memory and the state bag; derived speeds go here.
            evidence['telemetry'][key]['value'] = (record['payload'] if key != 'trajectory'
                else {name: value for name, value in record['payload'].items() if name != 'points'})
    runtime = snapshot.get('runtime')
    group = ((runtime or {}).get('payload', {}).get('frame_decision') or {}).get('group_id')
    planner = snapshot.get('planner_decision:%s' % group)
    evidence['telemetry']['planner_decision'] = timing(planner, now_mono, now_ros_ns)
    if planner is not None:
        evidence['telemetry']['planner_decision']['value'] = planner['payload']
    runtime_stamp = runtime['source_stamp_ns'] if runtime else None
    evidence['runtime_exact_image_stamp_match'] = bool(runtime_stamp and image_record
        and runtime_stamp == image_record['source_stamp_ns'])
    evidence['planned_speed'] = planned_speeds(snapshot.get('trajectory'), snapshot.get('odometry'), now_mono)
    return clean(evidence)


def number(value, digits=2):
    return '?' if not isinstance(value, (int, float)) or not math.isfinite(value) else ('%.*f' % (digits, value))


def telemetry_lines(label, evidence):
    telemetry = evidence['telemetry']
    runtime = telemetry.get('runtime', {}).get('value') or {}
    observation = runtime.get('frame_decision') or {}
    group = observation.get('group_id')
    mission = {201351: 'A', 201365: 'B', 201290: 'C', 201326: 'D'}.get(group, str(group or '대기'))
    raw = observation.get('observed_tuples') or []
    colors = {'red': '적색', 'yellow': '황색', 'green': '녹색'}
    shapes = {'circle': '원형', 'left_arrow': '좌회전', 'left': '좌회전'}
    lamps = ' + '.join(colors.get(str(v[0]).lower(), str(v[0])) + '/' +
        shapes.get(str(v[1]).lower(), str(v[1])) for v in raw if len(v) >= 2) or '판독 불가'
    planner = telemetry.get('planner_decision') or {}
    value = planner.get('value') or {}
    age = planner.get('receipt_age_sec')
    valid = value.get('group_id') == group and isinstance(age, (int, float)) and 0 <= age <= .75
    verdict = ('STOP 정지' if value.get('require_stop') else 'GO 진행') if valid else '판단 정보 없음'
    state = ('GO 확정' if value.get('committed') else '접근·재판단') if valid else '미수신/오래된 기록'
    speed = (telemetry.get('ego', {}).get('value') or {}).get('speed_kph')
    gate = observation.get('observation_distance_gate') or {}
    hold = observation.get('observation_timing') or {}
    distances = value if valid else {}
    return [
        '미션 %s  |  실제 속도 %s km/h  |  관측: %s' % (mission, number(speed, 1), lamps),
        '신호 판단: %s  |  %s' % (verdict, state),
        '정차 목표 %s m | 채점선 %s m | 계산 제동거리 %s m | 신호등까지(전방 깊이) %s m' %
        (tuple(number(distances.get(key)) for key in ('target_m', 'official_m', 'brake_m')) +
         (number(gate.get('current_matched_camera_depth_m')),)),
        '판독: %s  |  판단 기록 나이 %s초  |  영상 %.1f초' %
        ('원거리 미사용' if gate.get('suppressed') else
         '이전 관측 완충' if hold.get('output_is_held') else '현재 관측',
         number(age, 2), evidence['pts_sec']),
        ('판단 주기 아님: GO/STOP 표시는 약 0.5초 간격의 과거 요약 로그 기준'
         if label == '과거 판단 0.5초 표본' else ''),
    ]


def render_frame(image, label, evidence):
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    if image is not None:
        # debug_image includes a legacy verbose footer after the 720px camera.
        image = image[:720]
        scale = min(WIDTH/image.shape[1], IMAGE_HEIGHT/image.shape[0])
        target = (max(1, round(image.shape[1]*scale)), max(1, round(image.shape[0]*scale)))
        frame = cv2.resize(image, target) if target != (image.shape[1], image.shape[0]) else image
        if evidence['image_status'] != 'LIVE ANNOTATED CAMERA':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor((gray*.4).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        x, y = (WIDTH-target[0])//2, (IMAGE_HEIGHT-target[1])//2
        canvas[y:y+target[1], x:x+target[0]] = frame
    if not hasattr(render_frame, '_font'):
        render_frame._font = ImageFont.truetype(
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 25)
    panel = Image.new('RGB', (WIDTH, HEIGHT-IMAGE_HEIGHT), (24, 26, 30))
    draw = ImageDraw.Draw(panel)
    for index, line in enumerate(telemetry_lines(label, evidence)):
        draw.text((16, 7+index*46), line, font=render_frame._font,
                  fill=(255, 210, 100) if index == 1 else (235, 235, 235))
    canvas[IMAGE_HEIGHT:] = np.asarray(panel)[:, :, ::-1]
    return canvas


def ffmpeg_arguments(path, fps, encoder):
    command = ['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'warning', '-n',
        '-f', 'rawvideo', '-pixel_format', 'bgr24', '-video_size', '%dx%d' % (WIDTH, HEIGHT),
        '-framerate', str(fps), '-i', 'pipe:0', '-an', '-c:v', encoder, '-threads', '2']
    if encoder == 'libx264':
        command += ['-preset', 'ultrafast', '-crf', '28']
    elif encoder == 'h264_nvenc':
        command += ['-preset', 'fast', '-rc', 'vbr', '-cq', '28', '-b:v', '0']
    else:
        raise ValueError('Unsupported encoder')
    return command + ['-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)]


class VideoWriter:
    def __init__(self, command, log):
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=log, start_new_session=True)
        os.set_blocking(self.process.stdin.fileno(), False)

    def write(self, frame):
        data = memoryview(frame).cast('B')
        deadline = time.monotonic()+3.
        descriptor = self.process.stdin.fileno()
        while data:
            if self.process.poll() is not None:
                raise RuntimeError('ffmpeg exited before all video frames were written')
            if time.monotonic() >= deadline:
                raise RuntimeError('ffmpeg input blocked for 3s; evidence clock cannot keep up')
            if not select.select([], [descriptor], [], .1)[1]:
                continue
            try:
                written = os.write(descriptor, data)
                if not written:
                    raise RuntimeError('ffmpeg accepted zero bytes')
                data = data[written:]
            except BlockingIOError:
                continue

    def close(self):
        try:
            self.process.stdin.close()
        except BrokenPipeError:
            pass
        try:
            result = self.process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
            raise RuntimeError('ffmpeg did not finalize within 20 seconds after EOF')
        if result:
            raise RuntimeError('ffmpeg failed with exit code %s; retain ffmpeg.log' % result)


def subscribe(rospy, latest):
    from sensor_msgs.msg import CompressedImage
    from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
    from nav_msgs.msg import Odometry
    from autoware_planning_msgs.msg import Trajectory
    from std_msgs.msg import String
    from autoware_internal_debug_msgs.msg import StringStamped

    def callback(key, convert):
        def receive(message):
            mono, ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
            try:
                payload, source = convert(message)
                destination = ('planner_decision:%s' % payload['group_id']) if key == 'planner_decision' else key
                latest.put(destination, payload, source, ros_ns, mono)
            except (ValueError, TypeError, AttributeError, OverflowError):
                latest.error(key)
        return receive

    def image(message):
        if not 0 < len(message.data) <= MAX_JPEG_BYTES:
            raise ValueError('JPEG empty or above bounded mailbox size')
        return bytes(message.data), message_stamp(message)

    def runtime(message):
        if len(message.data) > 1024*1024:
            raise ValueError('Runtime diagnostic exceeds 1 MiB limit')
        value = clean(json.loads(message.data))
        if not isinstance(value, dict):
            raise ValueError('Runtime JSON object required')
        source = value.get('source_stamp_ns')
        return value, source if type(source) is int and source > 0 else None

    def trajectory(message):
        points = message.points[:MAX_TRAJECTORY_POINTS]
        return clean(dict(frame_id=message.header.frame_id, total_points=len(message.points),
            retained_points=len(points), truncated=len(points) < len(message.points),
            points=[(point.pose.position.x, point.pose.position.y, point.longitudinal_velocity_mps)
                    for point in points])), message_stamp(message)

    specifications = [
        ('planner_decision', StringStamped, lambda m: (json.loads(m.data), stamp_ns(m.stamp))),
        ('image', CompressedImage, image), ('runtime', String, runtime),
        ('ego', EgoVehicleStatus, lambda m: (clean(dict(speed_kph=math.hypot(m.velocity.x, m.velocity.y)*3.6)), message_stamp(m))),
        ('command', CtrlCmd, lambda m: (clean(dict(accel=m.accel, brake=m.brake, steering=m.steering)), message_stamp(m))),
        ('odometry', Odometry, lambda m: (clean(dict(frame_id=m.header.frame_id,
            x=m.pose.pose.position.x, y=m.pose.pose.position.y)), message_stamp(m))),
        ('trajectory', Trajectory, trajectory),
    ]
    return [rospy.Subscriber(TOPICS[key], cls, callback(key, convert), queue_size=1,
                            buff_size=2**24 if key == 'image' else 2**22)
            for key, cls, convert in specifications]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--label', choices=('current', 'confirm2', 'vote3', 'vote3_group', 'vote7', 'unknown_grace'), required=True)
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--max-seconds', type=float, default=900.)
    parser.add_argument('--encoder', choices=('libx264', 'h264_nvenc'), default='libx264')
    args = parser.parse_args(argv)
    if not 1 <= args.fps <= 30 or not math.isfinite(args.max_seconds) or not 0 < args.max_seconds <= 1800:
        parser.error('Require fps 1..30 and max-seconds finite in (0,1800]')
    output = validate_output(args.output)
    if not shutil.which('ffmpeg'):
        raise RuntimeError('ffmpeg is required')
    import cv2
    import numpy as np
    import rospy
    rospy.init_node('traffic_light_evidence_video', anonymous=True, disable_signals=True)
    stop, latest = threading.Event(), Latest()
    stop_reason = ['max_seconds']

    def request_stop(signum, _frame):
        stop_reason[0] = signal.Signals(signum).name
        stop.set()

    old_handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    output.mkdir(parents=True, exist_ok=False)
    command = ffmpeg_arguments(output/'front_evidence.mp4', args.fps, args.encoder)
    metadata = dict(schema_version=1, label=args.label, encoder=args.encoder,
        development_only=True, read_only_ROS_subscriptions=True, topics=TOPICS,
        ffmpeg_argv=command, canvas_size=[WIDTH, HEIGHT], fps=args.fps,
        max_seconds=args.max_seconds, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        image_contract='original 720px camera with boxes; Korean video-only footer; verbose provenance in frames.jsonl',
        clock_contract='wall-monotonic CFR; repeats keep original camera stamp; PTS=index/fps',
        telemetry_contract='latest received values with separate ages, not exact-image association; actual speed uses Ego; planned speeds use same-frame localization only',
        runtime_contract='runtime_exact_image_stamp_match only for identical positive source stamps; latest runtime may depict a different camera frame',
        runtime_not_planner_GO=True, planner_GO_source=TOPICS['planner_decision'], queue_size_per_topic=1, jpeg_mailbox_max_bytes=MAX_JPEG_BYTES,
        trajectory_max_points=MAX_TRAJECTORY_POINTS, started_utc=utc_now(),
        started_monotonic=time.monotonic(), started_ros_ns=rospy.Time.now().to_nsec(), status='starting')
    metadata_path = output/'metadata.json'

    def save_metadata():
        metadata_path.write_text(json.dumps(clean(metadata), indent=2, allow_nan=False)+'\n')

    save_metadata()
    writer = None
    subscriptions = []
    frame_count = decoded_count = repeated_count = decode_errors = 0
    last_sequence = last_written_sequence = None
    decoded = image_record = None
    decode_error = None
    failure = None
    try:
        subscriptions = subscribe(rospy, latest)
        with (output/'ffmpeg.log').open('xb') as log, (output/'frames.jsonl').open('x', buffering=1) as sidecar:
            writer = VideoWriter(command, log)
            start = time.monotonic()
            metadata.update(status='recording', cfr_started_monotonic=start,
                cfr_started_ros_ns=rospy.Time.now().to_nsec(), cfr_started_utc=utc_now())
            save_metadata()
            while not stop.is_set() and not rospy.is_shutdown():
                target = start+frame_count/args.fps
                if target-start >= args.max_seconds:
                    break
                if stop.wait(max(0., target-time.monotonic())):
                    break
                if time.monotonic()-start >= args.max_seconds:
                    break
                snapshot = latest.snapshot()
                newest = snapshot.get('image')
                if newest is not None and newest['sequence'] != last_sequence:
                    last_sequence = newest['sequence']
                    try:
                        candidate = cv2.imdecode(np.frombuffer(newest['payload'], np.uint8), cv2.IMREAD_COLOR)
                        if candidate is None:
                            raise ValueError('JPEG decoder returned no image')
                        decoded, image_record = candidate, newest
                        decoded_count += 1
                        decode_error = None
                    except (ValueError, cv2.error) as error:
                        decode_error = type(error).__name__+': '+str(error)
                        decode_errors += 1
                now_mono, now_ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
                sequence = image_record['sequence'] if image_record else None
                repeated = bool(image_record and sequence == last_written_sequence)
                evidence = frame_evidence(frame_count, args.fps, start, now_mono, now_ros_ns,
                                          snapshot, image_record, repeated, decode_error)
                writer.write(render_frame(decoded, args.label, evidence))
                evidence['write_completed_monotonic'] = time.monotonic()
                sidecar.write(json.dumps(evidence, allow_nan=False, separators=(',', ':'))+'\n')
                if frame_count == 0:
                    metadata['first_frame'] = {key: evidence[key] for key in
                        ('frame_index', 'pts_sec', 'utc', 'write_monotonic', 'write_ros_ns',
                         'write_completed_monotonic', 'image')}
                frame_count += 1
                repeated_count += int(repeated)
                last_written_sequence = sequence
            if rospy.is_shutdown():
                stop_reason[0] = 'ROS_shutdown'
            metadata.update(capture_ended_monotonic=time.monotonic(),
                capture_ended_utc=utc_now(), capture_ended_ros_ns=rospy.Time.now().to_nsec())
            writer.close()
            writer = None
        if decoded_count == 0:
            raise RuntimeError('No real annotated camera frame was decoded; waiting video is unqualified')
    except Exception as error:
        failure = type(error).__name__+': '+str(error)
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        if writer is not None:
            try:
                writer.close()
            except Exception as error:
                failure = (failure+'; ' if failure else '')+type(error).__name__+': '+str(error)
        metadata.update(status='failed' if failure else 'complete', error=failure,
            stop_reason=stop_reason[0], ended_utc=utc_now(), ended_monotonic=time.monotonic(),
            ended_ros_ns=rospy.Time.now().to_nsec(), frames_written=frame_count,
            decoded_image_count=decoded_count, repeated_frame_count=repeated_count,
            decode_error_count=decode_errors, counters=latest.counters())
        if (output/'front_evidence.mp4').is_file():
            metadata['video_bytes'] = (output/'front_evidence.mp4').stat().st_size
        save_metadata()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
