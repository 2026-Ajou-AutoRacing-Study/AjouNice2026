"""ROS 실행, 실시간 판정, 시나리오별 녹화의 공통 도구."""
import json
import math
import os
import shutil
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

from .config import ROOT


class RuntimeFailure(RuntimeError):
    pass


class ScenarioFailure(RuntimeFailure):
    """입력 시나리오에서 route/trajectory를 만들지 못한 유효한 실패."""
    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


def now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temp, path)


def stop_group(process, *, timeout=12):
    if process.poll() is not None:
        return
    for sig, wait_sec in ((signal.SIGINT, timeout), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=wait_sec)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


class Processes:
    def __init__(self, log_dir, env):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.env = env
        self.running = {}
        self.handles = {}

    def start(self, name, command):
        if name in self.running:
            raise RuntimeFailure("이미 실행 중인 프로세스: {}".format(name))
        logfile = (self.log_dir / (name + ".log")).open("ab", buffering=0)
        try:
            process = subprocess.Popen([str(c) for c in command], stdout=logfile,
                                       stderr=subprocess.STDOUT, env=self.env,
                                       start_new_session=True, cwd=str(ROOT))
        except Exception:
            logfile.close()
            raise
        self.running[name] = process
        self.handles[name] = logfile
        time.sleep(0.4)
        if process.poll() is not None:
            self.stop(name)
            raise RuntimeFailure("{} 시작 실패. 로그: {}".format(name, self.log_dir / (name + ".log")))
        return process

    def stop(self, name):
        process = self.running.pop(name, None)
        if process is not None:
            stop_group(process)
        handle = self.handles.pop(name, None)
        if handle is not None:
            handle.close()

    def stop_many(self, names):
        for name in names:
            self.stop(name)

    def close(self):
        self.stop_many(list(self.running)[::-1])


def master_alive():
    try:
        import rosgraph
        return bool(rosgraph.Master("/planning_scenario_validator").getPid())
    except Exception:
        return False


def check_no_conflicting_publishers():
    import rosgraph
    state = rosgraph.Master("/planning_scenario_validator").getSystemState()[0]
    owners = dict(state)
    for topic in ("/Ego_topic", "/Object_topic", "/sensor/gps", "/ctrl_cmd",
                  "/planning/trajectory", "/localization/kinematic_state",
                  "/perception/object_recognition/detection/objects"):
        if owners.get(topic):
            raise RuntimeFailure("기존 스택이 {}을 발행 중입니다: {}. 새 캠페인은 빈 ROS graph에서 시작하세요".format(topic, owners[topic]))


def prepare_scenario(case, install_dir, ego_network_json):
    """원본은 유지하고 ExternalCtrl 정지 시작 + network sidecar만 설치한다."""
    scene = json.loads(case.source.read_text(encoding="utf-8"))
    ego = scene["egoVehicle"]
    ego["beforeControlMode"] = 2
    ego["currentControlMode"] = 2
    ego["currentVelocity"] = 0.0
    ego["customValue"] = 0.0
    initial = ego["initPosition"]
    saved = initial.get("vehicleSaveData", {})
    saved["velocity"] = 0.0
    for container in (saved, saved.get("vehicle", {})):
        if "lastVelocity" in container:
            container["lastVelocity"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    if isinstance(saved.get("inputData"), list) and len(saved["inputData"]) > 6:
        saved["inputData"][6] = initial.get("gear", 1)
    installed = install_dir / (case.install_stem + ".json")
    atomic_json(installed, scene)
    sidecar = install_dir / "EgoNetwork" / (case.install_stem + "_MN.json")
    atomic_json(sidecar, json.loads(ego_network_json.read_text(encoding="utf-8")))
    pos = initial["pos"]
    return installed, (float(pos["x"]), float(pos["y"]))


class LiveMonitor:
    def __init__(self):
        import rospy
        from autoware_perception_msgs.msg import DetectedObjects
        from autoware_planning_msgs.msg import LaneletRoute, Trajectory
        from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus
        from nav_msgs.msg import Odometry
        self.rospy = rospy
        self.ego = None
        self.odom = None
        self.route = None
        self.trajectory = None
        self.detections = None
        self.stamps = {}
        self.last_ego_gap_at = 0.0
        self.previous_ego_stamp = 0.0
        self.collision_ids = set()
        self.seen_collision_ids = set()
        self.collision_count = 0
        self.armed = False
        self.events_path = None
        self.goal_min = float("inf")
        self.max_speed = 0.0
        self.arrival_since = None
        self.last_progress_time = time.monotonic()
        self.best_goal_distance = float("inf")
        self.evidence_lock = threading.Lock()
        self.drive_started_at = None
        self.start_odom_xy = None
        self.max_displacement_m = 0.0
        self.last_motion_at = None
        self.recent_odom = deque(maxlen=300)
        self.trajectory_message_count = 0
        self.trajectory_max_speed_mps = 0.0
        self.latest_trajectory_max_speed_mps = None
        self.trajectory_near_max_speed_mps = 0.0
        self.latest_trajectory_near_speed_mps = None
        self.control_command_count = 0
        self.max_accel_command = 0.0
        self.max_brake_command = 0.0
        self.latest_accel_command = None
        self.latest_brake_command = None
        self.detection_message_count = 0
        self.detection_nonempty_message_count = 0
        self.detection_max_objects = 0
        self.latest_detection_objects = None
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self._ego, queue_size=10)
        rospy.Subscriber("/CollisionData", CollisionData, self._collision, queue_size=10)
        rospy.Subscriber("/localization/kinematic_state", Odometry, self._odom, queue_size=10)
        rospy.Subscriber("/planning/mission_planning/route", LaneletRoute, self._route, queue_size=10)
        rospy.Subscriber("/planning/trajectory", Trajectory, self._trajectory, queue_size=10)
        rospy.Subscriber("/perception/object_recognition/detection/objects",
                         DetectedObjects, self._detections, queue_size=10)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self._control, queue_size=10)

    def _ego(self, message):
        self.ego = message
        received_at = time.monotonic()
        if self.previous_ego_stamp and received_at - self.previous_ego_stamp >= 0.7:
            self.last_ego_gap_at = received_at
        self.previous_ego_stamp = received_at
        self.stamps["ego"] = received_at

    def _odom(self, message):
        self.odom = message
        received_at = time.monotonic()
        self.stamps["odom"] = received_at
        with self.evidence_lock:
            if self.drive_started_at is not None:
                position = message.pose.pose.position
                xy = (float(position.x), float(position.y))
                if self.start_odom_xy is None:
                    self.start_odom_xy = xy
                self.max_displacement_m = max(self.max_displacement_m,
                    math.hypot(xy[0] - self.start_odom_xy[0], xy[1] - self.start_odom_xy[1]))
                speed = abs(float(message.twist.twist.linear.x))
                self.recent_odom.append((received_at, xy[0], xy[1], speed))
                if speed >= 0.3:
                    self.last_motion_at = received_at

    def _route(self, message):
        self.route = message
        self.stamps["route"] = time.monotonic()

    def _trajectory(self, message):
        self.trajectory = message
        self.stamps["trajectory"] = time.monotonic()
        with self.evidence_lock:
            if self.drive_started_at is not None:
                self.trajectory_message_count += 1
                speed = max((abs(float(point.longitudinal_velocity_mps))
                             for point in message.points), default=0.0)
                self.latest_trajectory_max_speed_mps = speed
                self.trajectory_max_speed_mps = max(self.trajectory_max_speed_mps, speed)
                near_speed = max((abs(float(point.longitudinal_velocity_mps))
                                  for point in message.points[:5]), default=0.0)
                self.latest_trajectory_near_speed_mps = near_speed
                self.trajectory_near_max_speed_mps = max(self.trajectory_near_max_speed_mps,
                                                          near_speed)

    def _detections(self, message):
        self.detections = message
        self.stamps["detections"] = time.monotonic()
        with self.evidence_lock:
            if self.drive_started_at is not None:
                self.detection_message_count += 1
                count = len(message.objects)
                self.latest_detection_objects = count
                self.detection_max_objects = max(self.detection_max_objects, count)
                if count:
                    self.detection_nonempty_message_count += 1

    def _control(self, message):
        with self.evidence_lock:
            if self.drive_started_at is None:
                return
            self.control_command_count += 1
            self.latest_accel_command = float(message.accel)
            self.latest_brake_command = float(message.brake)
            self.max_accel_command = max(self.max_accel_command, self.latest_accel_command)
            self.max_brake_command = max(self.max_brake_command, self.latest_brake_command)

    def _collision(self, message):
        current = {int(obj.unique_id) for obj in message.collision_object}
        if self.armed:
            for object_id in current - self.seen_collision_ids:
                self.collision_count += 1
                self.event("COLLISION", object_id=object_id)
            self.seen_collision_ids.update(current)
        self.collision_ids = current
        self.stamps["collision"] = time.monotonic()

    def event(self, name, **fields):
        if self.events_path is None:
            return
        with self.events_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"time": now(), "event": name, **fields}, ensure_ascii=False) + "\n")

    def wait(self, predicate, seconds, label):
        deadline = time.monotonic() + seconds
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.1)
        raise RuntimeFailure("{} 준비 제한시간({:.0f}초) 초과".format(label, seconds))

    def scenario_confirmed(self, xy, *, after, baseline_xy, tolerance):
        if self.ego is None or self.stamps.get("ego", 0) <= after:
            return False
        if math.hypot(float(self.ego.position.x) - xy[0],
                      float(self.ego.position.y) - xy[1]) > tolerance:
            return False
        # With the same starting position, a fresh UDP Ego packet can still
        # belong to the *old* scene. MORAI pauses its Ego stream during reload.
        # Require that observable reset gap before accepting the new scene.
        if baseline_xy is not None and math.hypot(
                baseline_xy[0] - xy[0], baseline_xy[1] - xy[1]) <= tolerance:
            return self.last_ego_gap_at > after
        return True

    def wait_scenario(self, xy, *, after, timeout, tolerance, baseline_xy=None):
        self.wait(lambda: self.scenario_confirmed(
            xy, after=after, baseline_xy=baseline_xy, tolerance=tolerance),
            timeout, "ScenarioLoad Ego 위치/재시작")

    def wait_odom(self, *, after, timeout):
        self.wait(lambda: self.stamps.get("odom", 0) > after, timeout, "localization")

    def wait_detection(self, *, after, timeout, process=None, log_path=None):
        deadline = time.monotonic() + timeout
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            if self.stamps.get("detections", 0) > after:
                return
            if process is not None and process.poll() is not None:
                raise RuntimeFailure("BEVFusion 실행기가 종료됐습니다. 로그: {}".format(log_path))
            time.sleep(0.1)
        raise RuntimeFailure("BEVFusion detection 준비 제한시간({:.0f}초) 초과. LiDAR/카메라/서버 연결을 확인하세요".format(timeout))

    def wait_route_and_trajectory(self, route_uuid, *, after, timeout):
        self.wait(lambda: self.route is not None and self.stamps.get("route", 0) > after
                  and list(self.route.uuid.uuid) == list(route_uuid), timeout, "요청한 route")
        route_time = self.stamps["route"]
        self.wait(lambda: self.trajectory is not None
                  and self.stamps.get("trajectory", 0) > route_time
                  and bool(self.trajectory.points), timeout, "새 planning trajectory")

    def arm(self, events_path):
        self.begin_case(events_path)
        self.collision_count = 0
        # 이전 scenario의 contact ID가 새 시나리오 첫 접촉을 숨기지 않게 한다.
        self.collision_ids.clear()
        self.seen_collision_ids.clear()
        self.armed = True
        self.goal_min = float("inf")
        self.max_speed = 0.0
        self.arrival_since = None
        self.best_goal_distance = float("inf")
        self.last_progress_time = time.monotonic()
        with self.evidence_lock:
            self.drive_started_at = None
        self.event("RUN_ARMED", existing_collision_ids=sorted(self.collision_ids))

    def start_driving(self, started_at):
        """Reset evidence after the controller is ready, excluding startup brake hold."""
        with self.evidence_lock:
            self.drive_started_at = started_at
            self.start_odom_xy = None if self.odom is None else (
                float(self.odom.pose.pose.position.x), float(self.odom.pose.pose.position.y))
            self.max_displacement_m = 0.0
            self.last_motion_at = started_at
            self.recent_odom.clear()
            self.trajectory_message_count = 0
            self.trajectory_max_speed_mps = 0.0
            self.latest_trajectory_max_speed_mps = None
            self.trajectory_near_max_speed_mps = 0.0
            self.latest_trajectory_near_speed_mps = None
            self.control_command_count = 0
            self.max_accel_command = 0.0
            self.max_brake_command = 0.0
            self.latest_accel_command = None
            self.latest_brake_command = None
            self.detection_message_count = 0
            self.detection_nonempty_message_count = 0
            self.detection_max_objects = 0
            self.latest_detection_objects = None

    def drive_evidence(self):
        """Summarize live subscribers without replaying the saved rosbag."""
        with self.evidence_lock:
            if self.drive_started_at is None:
                return {"drive_observation_available": False}
            tick = time.monotonic()
            odom_age = tick - self.stamps.get("odom", self.drive_started_at)
            trajectory_age = tick - self.stamps.get("trajectory", self.drive_started_at)
            final_speed = (abs(float(self.odom.twist.twist.linear.x))
                           if self.odom is not None else None)
            recent = [sample for sample in self.recent_odom if sample[0] >= tick - 5.0]
            recent_displacement = (math.hypot(recent[-1][1] - recent[0][1],
                                              recent[-1][2] - recent[0][2])
                                   if len(recent) >= 2 else None)
            return {
                "drive_observation_available": True,
                "max_displacement_m": round(self.max_displacement_m, 3),
                "final_speed_mps": None if final_speed is None else round(final_speed, 3),
                "stationary_duration_sec": round(tick - self.last_motion_at, 2)
                    if self.last_motion_at is not None else None,
                "last_5s_displacement_m": None if recent_displacement is None
                    else round(recent_displacement, 3),
                "odom_age_sec": round(odom_age, 2),
                "trajectory_age_sec": round(trajectory_age, 2),
                "trajectory_message_count": self.trajectory_message_count,
                "trajectory_max_speed_mps": round(self.trajectory_max_speed_mps, 3),
                "latest_trajectory_max_speed_mps": self.latest_trajectory_max_speed_mps,
                "trajectory_near_max_speed_mps": round(self.trajectory_near_max_speed_mps, 3),
                "latest_trajectory_near_speed_mps": self.latest_trajectory_near_speed_mps,
                "control_command_count": self.control_command_count,
                "max_accel_command": round(self.max_accel_command, 3),
                "max_brake_command": round(self.max_brake_command, 3),
                "latest_accel_command": self.latest_accel_command,
                "latest_brake_command": self.latest_brake_command,
                "detection_message_count": self.detection_message_count,
                "detection_nonempty_message_count": self.detection_nonempty_message_count,
                "detection_max_objects": self.detection_max_objects,
                "latest_detection_objects": self.latest_detection_objects,
            }

    def begin_case(self, events_path):
        self.events_path = Path(events_path)
        self.events_path.touch(exist_ok=True)
        self.collision_count = 0
        self.goal_min = float("inf")
        self.max_speed = 0.0

    def disarm(self):
        self.armed = False
        with self.evidence_lock:
            self.drive_started_at = None
        self.events_path = None

    def run_until_done(self, case, validation, started, state_callback=None):
        radius = float(validation.get("goal_tolerance_m", 2.0))
        hold = float(validation.get("goal_hold_sec", 1.0))
        max_stop_speed = float(validation.get("goal_speed_max_mps", 0.5))
        stale = float(validation.get("topic_loss_timeout_sec", 4.0))
        stuck_sec = float(validation.get("stuck_timeout_sec", 0.0))
        while not self.rospy.is_shutdown():
            tick = time.monotonic()
            elapsed = tick - started
            if self.collision_count:
                return "COLLISION", elapsed
            if elapsed >= case.timeout_sec:
                return "TIMEOUT", elapsed
            if tick - self.stamps.get("odom", started) > stale:
                raise RuntimeFailure("localization topic이 {}초 이상 끊겼습니다".format(stale))
            if tick - self.stamps.get("trajectory", started) > stale:
                raise RuntimeFailure("planning trajectory가 {}초 이상 끊겼습니다".format(stale))
            if self.odom is not None:
                p = self.odom.pose.pose.position
                distance = math.hypot(p.x - case.goal["x"], p.y - case.goal["y"])
                speed = abs(self.odom.twist.twist.linear.x)
                self.goal_min = min(self.goal_min, distance)
                self.max_speed = max(self.max_speed, speed)
                if distance < self.best_goal_distance - 1.0:
                    self.best_goal_distance = distance
                    self.last_progress_time = tick
                if distance <= radius and (case.arrival_mode == "pass_through" or speed <= max_stop_speed):
                    self.arrival_since = self.arrival_since or tick
                    if tick - self.arrival_since >= hold:
                        self.event("GOAL_REACHED", distance_m=round(distance, 3))
                        return "SUCCESS", elapsed
                else:
                    self.arrival_since = None
                if stuck_sec > 0 and elapsed > stuck_sec and tick - self.last_progress_time > stuck_sec:
                    return "STUCK", elapsed
                if state_callback:
                    state_callback({"elapsed_sec": round(elapsed, 2), "goal_distance_m": round(distance, 2),
                                    "speed_mps": round(speed, 2), "collision_count": self.collision_count})
            time.sleep(0.2)
        return "MANUAL_ABORT", time.monotonic() - started


def _pose_message(value):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = value["x"], value["y"], value["z"]
    angle = math.radians(value["yaw_deg"]) / 2
    pose.orientation.z, pose.orientation.w = math.sin(angle), math.cos(angle)
    return pose


def request_route(case, monitor, *, timeout):
    import rospy
    from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
    rospy.wait_for_service("/planning/set_waypoint_route", timeout=timeout)
    request = SetWaypointRouteRequest()
    request.header.frame_id = "map"
    request.header.stamp = rospy.Time.now()
    request.goal_pose = _pose_message(case.goal)
    request.waypoints = [_pose_message(p) for p in case.waypoints]
    route_id = uuid.uuid4().bytes
    request.uuid.uuid = list(route_id)
    request.allow_modification = False
    after = time.monotonic()
    response = rospy.ServiceProxy("/planning/set_waypoint_route", SetWaypointRoute)(request)
    if not response.status.success:
        raise ScenarioFailure("ROUTE_FAILURE", "route 요청 거부: {}".format(response.status))
    try:
        monitor.wait_route_and_trajectory(route_id, after=after, timeout=timeout)
    except RuntimeFailure as error:
        raise ScenarioFailure("NO_TRAJECTORY", str(error)) from error
    monitor.event("ROUTE_READY", uuid=str(uuid.UUID(bytes=route_id)))
    return str(uuid.UUID(bytes=route_id))


def publish_brake(duration=1.0):
    import rospy
    from morai_msgs.msg import CtrlCmd
    publisher = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
    command = CtrlCmd()
    command.longlCmdType = 1
    command.accel = 0.0
    command.brake = 1.0
    command.steering = 0.0
    until = time.monotonic() + duration
    while time.monotonic() < until and not rospy.is_shutdown():
        publisher.publish(command)
        time.sleep(0.05)
    publisher.unregister()


class BrakeHold:
    """Keep MORAI stationary while asynchronous model/planning nodes start."""

    def __init__(self):
        import rospy
        from morai_msgs.msg import CtrlCmd
        self.rospy = rospy
        self.publisher = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.command = CtrlCmd()
        self.command.longlCmdType = 1
        self.command.brake = 1.0
        self.stopped = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, name="validator_brake_hold", daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stopped.is_set() and not self.rospy.is_shutdown():
            self.publisher.publish(self.command)
            self.stopped.wait(0.05)

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        self.publisher.unregister()


def video_size(display):
    output = subprocess.check_output(["xdpyinfo", "-display", str(display)], text=True, timeout=5)
    import re
    match = re.search(r"dimensions:\s+(\d+x\d+) pixels", output)
    if not match:
        raise RuntimeFailure("X11 화면 크기를 확인하지 못했습니다")
    return match.group(1)


def focus_planviz(timeout=45, display=None):
    """Maximize/raise the RViz window before full-screen X11 capture."""
    command = ["/usr/bin/python3", str(ROOT / "base_ws/src/planning_addons/"
                                        "planning_scenario_validator/scripts/focus_planviz.py"),
               str(timeout)]
    # libwnck can occasionally crash while RViz replaces its X11 window on a
    # scenario transition. A fresh helper process succeeds once that window
    # settles; keep the recorder non-fatal if every attempt fails.
    focus_env = os.environ.copy()
    if display:
        focus_env["DISPLAY"] = str(display)
    for attempt in range(3):
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=timeout + 3, env=focus_env)
        except subprocess.TimeoutExpired as error:
            raise RuntimeFailure("Planviz 창 최대화 대기 시간이 초과됐습니다") from error
        if result.returncode == 0:
            return
        if result.returncode not in (-11, 139) or attempt == 2:
            detail = result.stderr.strip() or result.stdout.strip() or "진단 출력 없음"
            raise RuntimeFailure("Planviz 자동 최대화 실패(exit {}): {}".format(
                result.returncode, detail))
        time.sleep(1.0)


class Recording:
    def __init__(self, config, output, env):
        self.config, self.output, self.env = config, Path(output), env
        self.bag = None
        self.video = None
        self.handles = []

    def start(self, basename):
        self.output.mkdir(parents=True, exist_ok=True)
        self.bag_prefix = self.output / (basename + "_recording")
        bag_log = (self.output / "rosbag.log").open("wb")
        self.handles.append(bag_log)
        self.bag = subprocess.Popen(["rosbag", "record", "-O", str(self.bag_prefix),
                                     *self.config["topics"]], stdout=bag_log,
                                    stderr=subprocess.STDOUT, start_new_session=True, env=self.env)
        time.sleep(0.7)
        if self.bag.poll() is not None:
            raise RuntimeFailure("rosbag 시작 실패: {}".format(self.output / "rosbag.log"))
        video = self.config.get("video", {})
        if video.get("enabled", False):
            if not shutil.which("ffmpeg"):
                raise RuntimeFailure("영상 녹화에 ffmpeg가 필요합니다")
            self.video_path = self.output / (basename + "_recording.mp4")
            video_log = (self.output / "video.log").open("wb")
            self.handles.append(video_log)
            self.video = subprocess.Popen([
                "ffmpeg", "-nostdin", "-loglevel", "warning", "-y", "-f", "x11grab",
                "-framerate", str(video.get("fps", 15)), "-video_size", video_size(video["display"]),
                "-i", str(video["display"]), "-c:v", "libx264", "-preset", "ultrafast",
                "-crf", "28", "-pix_fmt", "yuv420p", str(self.video_path)
            ], stdout=video_log, stderr=subprocess.STDOUT, start_new_session=True, env=self.env)
            time.sleep(0.5)
            if self.video.poll() is not None:
                raise RuntimeFailure("화면 녹화 시작 실패: {}".format(self.output / "video.log"))

    def stop(self, final_basename):
        if self.video is not None:
            stop_group(self.video, timeout=15)
        if self.bag is not None:
            stop_group(self.bag, timeout=30)
        for handle in self.handles:
            handle.close()
        bag_source = Path(str(self.bag_prefix) + ".bag") if self.bag is not None else None
        if bag_source:
            deadline = time.monotonic() + 15
            while not bag_source.is_file() and time.monotonic() < deadline:
                time.sleep(0.2)
            if not bag_source.is_file():
                raise RuntimeFailure("rosbag이 정상 마감되지 않았습니다: {} (active 파일은 보존됨)".format(self.bag_prefix))
        bag_final = self.output / (final_basename + ".bag") if bag_source else None
        if bag_source:
            bag_source.rename(bag_final)
        video_final = self.output / (final_basename + ".mp4") if self.video is not None else None
        if video_final:
            if not self.video_path.is_file():
                raise RuntimeFailure("영상 파일이 생성되지 않았습니다")
            self.video_path.rename(video_final)
        return bag_final, video_final
