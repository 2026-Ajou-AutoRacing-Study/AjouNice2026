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

# 실제 Ego 접촉은 차체 중심 간 약 5m 이내. 이보다 먼 /CollisionData 보고는 Ego 충돌이 아니다.
COLLISION_EGO_RADIUS_M = 10.0


class RuntimeFailure(RuntimeError):
    pass


class SafetyFailure(RuntimeFailure):
    """A failed live safety gate must stop a campaign, not auto-advance a case."""


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
        self.split_layout_initialized = False
        self.commands = []

    def start(self, name, command):
        if name in self.running:
            raise RuntimeFailure("이미 실행 중인 프로세스: {}".format(name))
        self.commands.append({'name': name, 'command': [str(c) for c in command], 'started_at': now()})
        atomic_json(self.log_dir / 'commands.json', self.commands)
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
    from .repetitions import sampled_scene
    scene, _ = sampled_scene(scene, source_sha256=case.source_sha256,
                             repeat_index=getattr(case, "repeat_index", 1),
                             settings=getattr(case, "randomization", {}))
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
        from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus, GPSMessage
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Int8
        self.rospy = rospy
        self.ego = None
        self.odom = None
        self.route = None
        self.trajectory = None
        self.detections = None
        self.competition = None
        self.gps = None
        self.imu = None
        self.control_odom = None
        self.gear = None
        self.control_mode = None
        self.control_source = None
        self.preparation_collisions = {}
        self.stamps = {}
        self.last_ego_gap_at = 0.0
        self.previous_ego_stamp = 0.0
        self.collision_ids = set()
        self.seen_collision_ids = set()
        self.collision_count = 0
        self.far_collision = None
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
        # Receipt time, not GNSS validity, is the watchdog's stream contract.
        # A GPS shadow may publish zero coordinates while the receiver is healthy.
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self._competition, queue_size=10)
        rospy.Subscriber("/sensor/gps", GPSMessage, self._gps, queue_size=10)
        rospy.Subscriber("/sensor/imu", Imu, self._imu, queue_size=10)
        rospy.Subscriber("/localization/control_kinematic_state", Odometry,
                         self._control_odom, queue_size=10)
        rospy.Subscriber("/vehicle/status/gear", Int8, self._gear, queue_size=10)
        rospy.Subscriber("/vehicle/status/control_mode", Int8, self._control_mode, queue_size=10)

    def _competition(self, message):
        self.competition = message
        self.stamps["competition"] = time.monotonic()

    def _gps(self, message):
        self.gps = message
        self.stamps["gps"] = time.monotonic()

    def _imu(self, message):
        self.imu = message
        self.stamps["imu"] = time.monotonic()

    def _control_odom(self, message):
        self.control_odom = message
        self.stamps["control_state"] = time.monotonic()

    def _gear(self, message):
        self.gear = int(message.data)
        self.stamps["gear"] = time.monotonic()

    def _control_mode(self, message):
        self.control_mode = int(message.data)
        self.stamps["control_mode"] = time.monotonic()

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
        self.stamps["control"] = time.monotonic()
        self.control_source = (getattr(message, "_connection_header", None) or {}).get("callerid")
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
        received_at = time.monotonic()
        if not self.armed and getattr(self, "events_path", None) is not None:
            # Keep preparation contacts distinct from armed drive collision counts.
            # An opted-in gate can reject them using the scene-ready receipt time.
            for obj in message.collision_object:
                object_id = int(obj.unique_id)
                distance = self._ego_distance(obj.position)
                detail = {"object_id": object_id, "receipt_monotonic": received_at,
                          "distance_m": None if distance is None else round(distance, 3)}
                if object_id not in self.preparation_collisions:
                    self.event("PREPARATION_COLLISION", **detail)
                self.preparation_collisions[object_id] = detail
        if self.armed:
            for obj in message.collision_object:
                object_id = int(obj.unique_id)
                if object_id in self.seen_collision_ids:
                    continue
                distance = self._ego_distance(obj.position)
                if distance is not None and distance > COLLISION_EGO_RADIUS_M:
                    # MORAI는 이전 실행에서 Ego와 부딪힌 unique_id를 재사용된 NPC에도
                    # 충돌로 보고한다. Ego 충돌이 아니므로 FAIL로 세지 않고 오염으로 처리한다.
                    self.far_collision = (object_id, round(distance, 1))
                    self.event("COLLISION_FAR_FROM_EGO", object_id=object_id,
                               distance_m=round(distance, 1),
                               position=[round(float(obj.position.x), 1), round(float(obj.position.y), 1)])
                    continue
                self.collision_count += 1
                self.event("COLLISION", object_id=object_id,
                           distance_m=None if distance is None else round(distance, 1))
            self.seen_collision_ids.update(current)
        self.collision_ids = current
        self.stamps["collision"] = received_at

    def require_fresh_inputs(self, required, *, after=0.0, freshness_sec=1.0):
        """Opt-in guard: absent, stale, or pre-gate packets cannot prove readiness."""
        ages = {}
        for key in required:
            stamp = self.stamps.get(key)
            if stamp is None or not math.isfinite(stamp) or stamp <= after:
                raise SafetyFailure("안전 입력 topic {}의 새 메시지가 없습니다".format(key))
            # Sample now after the selected stamp: callbacks can update stamps
            # concurrently, so a once-per-loop 'now' can appear slightly older
            # than a perfectly valid received packet and cause false aborts.
            age = time.monotonic() - stamp
            if not 0.0 <= age <= freshness_sec:
                raise SafetyFailure("안전 입력 topic {}이 오래됐습니다: {:.3f}s".format(key, age))
            ages[key] = round(age, 4)
        return {"required": list(required), "receipt_age_sec": ages}

    def raw_ego_speed_mps(self):
        """The native UDP receiver already converts km/h to ROS SI units once."""
        if self.ego is None:
            raise SafetyFailure("실제 Ego 속도 수신이 없습니다")
        try:
            components = [float(getattr(self.ego.velocity, axis)) for axis in ("x", "y", "z")]
        except (AttributeError, TypeError, ValueError) as error:
            raise SafetyFailure("실제 Ego 속도 메시지가 올바르지 않습니다") from error
        if not all(math.isfinite(value) for value in components):
            raise SafetyFailure("실제 Ego 속도가 유한하지 않습니다")
        return math.sqrt(sum(value * value for value in components))

    def gps_is_valid(self):
        message = getattr(self, "gps", None)
        if message is None:
            return False
        # This validity contract is only used at pre-shadow startup. During an
        # outage the zero coordinates are expected and must not trip a watchdog.
        try:
            latitude, longitude = float(message.latitude), float(message.longitude)
            quality = int(message.status)
        except (AttributeError, TypeError, ValueError):
            return False
        return (math.isfinite(latitude) and math.isfinite(longitude)
                and latitude != 0.0 and longitude != 0.0
                and quality > 0)

    def preparation_contacts(self, *, after):
        # Callbacks may add another contact while the runner evaluates a gate.
        rows = list(getattr(self, "preparation_collisions", {}).values())
        return [dict(row) for row in rows
                if row["receipt_monotonic"] > after]

    def wait_stationary(self, *, after, timeout=15.0, hold_sec=1.0,
                        speed_max_mps=0.05,
                        required=("ego", "gear", "control_mode", "collision"),
                        freshness_sec=1.0, expected_gears=(1,),
                        expected_control_mode=2, require_valid_gps=False,
                        check_preparation_collision=True, process=None):
        """Require fresh raw Ego/gear feedback and a continuous stationary hold.

        This method is not called by generic campaigns unless the runner opts in.
        It returns measured evidence; it never infers Park from a sent command.
        """
        deadline = time.monotonic() + timeout
        stationary_since = None
        first_stationary_ego_stamp = None
        issue = "새 입력 대기"
        receipt = None
        names = tuple(dict.fromkeys(tuple(required) + (("gps",) if require_valid_gps else ())))
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            if process is not None and process.poll() is not None:
                raise SafetyFailure("정지 확인 중 소유한 프로세스가 종료됐습니다")
            if check_preparation_collision:
                contacts = self.preparation_contacts(after=after)
                if contacts:
                    self.event("SAFETY_PREPARATION_CONTACT", contacts=contacts)
                    raise SafetyFailure("준비 중 CollisionData 접촉 보고가 있습니다: {}".format(contacts))
            try:
                receipt = self.require_fresh_inputs(names, after=after, freshness_sec=freshness_sec)
                speed = self.raw_ego_speed_mps()
                if expected_gears is not None and self.gear not in expected_gears:
                    raise SafetyFailure("실제 gear {}: 요구 {}".format(self.gear, expected_gears))
                if expected_control_mode is not None and self.control_mode != expected_control_mode:
                    raise SafetyFailure("실제 control mode {}: 요구 {}".format(
                        self.control_mode, expected_control_mode))
                if require_valid_gps and not self.gps_is_valid():
                    raise SafetyFailure("음영 진입 전 GNSS가 유효하지 않습니다")
                if speed > speed_max_mps:
                    raise SafetyFailure("실제 Ego 속도 {:.3f}m/s: 정지 기준 {:.3f}m/s".format(
                        speed, speed_max_mps))
            except SafetyFailure as error:
                issue = str(error)
                stationary_since = None
                first_stationary_ego_stamp = None
                time.sleep(0.05)
                continue
            tick = time.monotonic()
            stationary_since = tick if stationary_since is None else stationary_since
            ego_stamp = self.stamps.get("ego", 0.0)
            if first_stationary_ego_stamp is None:
                first_stationary_ego_stamp = ego_stamp
            # Polling the same fresh message for half a second is not evidence
            # of a half-second measured stop. Require new Ego feedback spanning
            # the complete hold interval as well as the wall-time interval.
            if (tick - stationary_since >= hold_sec
                    and ego_stamp - first_stationary_ego_stamp >= hold_sec):
                receipt.update(speed_mps=round(speed, 6), gear=self.gear,
                    control_mode=self.control_mode, park_verified=self.gear == 1,
                    hold_sec=round(tick - stationary_since, 3),
                    gate_after_monotonic=after, verified_at_monotonic=tick,
                    gps_valid=self.gps_is_valid() if require_valid_gps else None)
                self.event("SAFETY_STATIONARY_CONFIRMED", **receipt)
                return receipt
            time.sleep(0.05)
        raise SafetyFailure("실제 Ego 정지/기어 확인 실패 ({:.1f}s): {}".format(timeout, issue))

    def verify_stopped(self, *, after=None, timeout=10.0, hold_sec=0.5,
                       speed_max_mps=0.05, freshness_sec=1.0, require_park=False):
        """Call after braking starts, with receivers/sender alive, before bag close."""
        receipt = self.wait_stationary(after=time.monotonic() if after is None else after,
            timeout=timeout, hold_sec=hold_sec, speed_max_mps=speed_max_mps,
            required=("ego", "gear", "control_mode"), freshness_sec=freshness_sec,
            expected_gears=(1,) if require_park else None, expected_control_mode=None,
            check_preparation_collision=False)
        self.event("SAFETY_STOP_CONFIRMED", **receipt)
        return receipt

    def localization_watchdog(self, *, pose_source="localization", freshness_sec=1.0,
                              require_collision=True, require_control=True):
        """Opted-in closed-loop stream guard; invalid GPS remains an expected outage."""
        required = ["ego", "odom", "gear", "control_mode"]
        if require_collision:
            required.append("collision")
        if require_control:
            required.append("control")
        if pose_source == "localization":
            required.extend(("competition", "gps", "imu", "control_state"))
        elif pose_source != "maptf_morai":
            raise SafetyFailure("지원하지 않는 안전 측위 source: {}".format(pose_source))
        receipt = self.require_fresh_inputs(required, freshness_sec=freshness_sec)
        receipt.update(raw_ego_speed_mps=self.raw_ego_speed_mps(),
                       gps_valid=self.gps_is_valid() if pose_source == "localization" else None)
        return receipt

    def _ego_distance(self, position):
        """Ego(/Ego_topic)와 충돌 객체 사이 거리. 둘 다 MORAI 좌표. Ego를 모르면 None."""
        if self.ego is None:
            return None
        return math.hypot(float(position.x) - float(self.ego.position.x),
                          float(position.y) - float(self.ego.position.y))

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

    def wait_route_and_trajectory(self, route_uuid, *, after, timeout, process=None):
        def alive():
            if process is not None and process.poll() is not None:
                raise RuntimeFailure('planning 스택이 route/trajectory 준비 중 종료됐습니다. stack_logs/planning.log 확인')
            return True
        self.wait(lambda: alive() and self.route is not None and self.stamps.get("route", 0) > after
                  and list(self.route.uuid.uuid) == list(route_uuid), timeout, "요청한 route")
        route_time = self.stamps["route"]
        self.wait(lambda: alive() and self.trajectory is not None
                  and self.stamps.get("trajectory", 0) > route_time
                  and bool(self.trajectory.points), timeout, "새 planning trajectory")

    def arm(self, events_path):
        self.begin_case(events_path)
        self.collision_count = 0
        self.far_collision = None
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
        self.preparation_collisions = {}

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
            if self.far_collision:
                raise RuntimeFailure(
                    "MORAI가 Ego와 {1}m 떨어진 객체 {0}을 충돌로 보고했습니다. 이전 충돌 상태가 남은 것으로 "
                    "보입니다. MORAI를 재시작한 뒤 재개하세요".format(*self.far_collision))
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


def request_route(case, monitor, *, timeout, process=None):
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
        monitor.wait_route_and_trajectory(route_id, after=after, timeout=timeout, process=process)
    except RuntimeFailure as error:
        if process is not None and process.poll() is not None:
            raise RuntimeFailure(str(error)) from error
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


def arrange_split_windows(*, bottom_bar_px="auto", display=None, startup_grace_sec=0.0):
    """Best-effort MORAI/RViz tiling; a failed layout must not abort recording."""
    command = ["/usr/bin/python3", str(ROOT / "base_ws/src/planning_addons/"
                                        "planning_scenario_validator/scripts/arrange_split_windows.py"),
               str(bottom_bar_px), str(startup_grace_sec)]
    layout_env = os.environ.copy()
    if display:
        layout_env["DISPLAY"] = str(display)
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=18, env=layout_env)
    except subprocess.TimeoutExpired as error:
        raise RuntimeFailure("MORAI/RViz 반반 배치 시간 초과") from error
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip() or "진단 출력 없음"
        raise RuntimeFailure("MORAI/RViz 자동 반반 배치 실패: {}".format(detail))
    return result.stdout.strip()


class Recording:
    def __init__(self, config, output, env):
        self.config, self.output, self.env = config, Path(output), env
        self.bag = None
        self.video = None
        self.handles = []

    def start(self, basename):
        self.output.mkdir(parents=True, exist_ok=True)
        self.bag_prefix = self.output / (basename + "_recording")
        if self.config.get('bag_enabled', True):
            bag_log = (self.output / "rosbag.log").open("wb")
            self.handles.append(bag_log)
            topics = self.config['topics']
            if not topics:
                raise RuntimeFailure('bag ON인데 선택된 녹화 토픽이 없습니다')
            patterns = self.config.get('topic_regex', [])
            if self.config.get('analysis_mode', False) or patterns:
                import re
                # Debug/info topics differ across planner versions. Capture actual publishers too.
                topics = ['--regex'] + ['^' + re.escape(t) + '$' for t in topics] + list(patterns)
                if self.config.get('analysis_mode', False):
                    topics += ['^/planning/.*(avoidance|static_obstacle|debug).*']
            self.bag = subprocess.Popen(["rosbag", "record", "-O", str(self.bag_prefix),
                                         *topics], stdout=bag_log,
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
