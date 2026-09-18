#!/usr/bin/env python3
"""ROS node implementing the development-time competition judge."""

import csv
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rospy
import tf2_ros
import yaml
from morai_msgs.msg import CollisionData, EgoVehicleStatus
from std_msgs.msg import Int8, String
from visualization_msgs.msg import Marker, MarkerArray

from competition_rule_monitor.rules import OrderedCheckpoints, PeriodicPenalty, is_speeding


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _rotate(q, point):
    # Quaternion-vector rotation without a geometry helper dependency.
    x, y, z = point
    tx = 2.0 * (q.y * z - q.z * y)
    ty = 2.0 * (q.z * x - q.x * z)
    tz = 2.0 * (q.x * y - q.y * x)
    return (
        x + q.w * tx + q.y * tz - q.z * ty,
        y + q.w * ty + q.z * tx - q.x * tz,
        z + q.w * tz + q.x * ty - q.y * tx,
    )


class CompetitionRuleMonitor:
    PARK = 1
    DRIVE = 4

    def __init__(self):
        self.state = "READY"
        self.error = ""
        self.allow_late_drive_start = rospy.get_param("~allow_late_drive_start", True)
        self.gear = None
        self.previous_gear = None
        self.seen_park = False
        self.pose = None
        self.pose_stamp = None
        self.ego_position = None
        self.ego_heading_deg = None
        self.speed_kph = None
        self.ego_stamp = None
        self.gear_stamp = None
        self.points_ready = False
        self.morai_to_map = None
        self.finish_point = None
        self.finish_zone_departed = False
        self.checkpoint_points = []
        self.current_lanelets = []
        self.in_blackout = False
        self.in_speed_exception = False
        self.active_collision_ids = set()

        self.rule_revision = rospy.get_param("~rule_revision")
        self.map_version = int(rospy.get_param("~map_version"))
        self.expected_map_sha = str(rospy.get_param("~map_sha256"))
        self.radius = float(rospy.get_param("~checkpoint_radius_m", 3.0))
        self.speed_limit = float(rospy.get_param("~speed_limit_kph", 60.0))
        self.speed_rule = PeriodicPenalty(
            rospy.get_param("~speed_penalty_interval_sec", 3.0),
            rospy.get_param("~speed_penalty_sec", 15),
            immediate=True,
        )
        self.lane_rule = PeriodicPenalty(
            rospy.get_param("~lane_penalty_interval_sec", 3.0),
            rospy.get_param("~lane_penalty_sec", 5),
        )
        self.collision_penalty = int(rospy.get_param("~collision_penalty_sec", 15))
        self.checkpoint_cfg = rospy.get_param("~checkpoints")
        self.finish_cfg = rospy.get_param("~start_finish")
        self.checkpoints = OrderedCheckpoints(len(self.checkpoint_cfg))
        self.blackout_ids = {int(v) for v in rospy.get_param("~gps_blackout_lanelet_ids")}
        self.exception_speed = float(
            rospy.get_param("~high_speed_exception/lanelet_speed_limit_above_kph", 60.0)
        )
        self.expected_exception_lanelets = int(
            rospy.get_param("~high_speed_exception/expected_lanelet_count", 51)
        )
        vehicle = rospy.get_param("~vehicle")
        self.wheelbase = float(vehicle["wheelbase_m"])
        self.front_track = float(vehicle["front_track_m"])
        self.rear_track = float(vehicle["rear_track_m"])
        self.tire_width = float(vehicle["tire_width_m"])

        self.map_file = Path(rospy.get_param("~map_file")).expanduser().resolve()
        self.projector_file = Path(rospy.get_param("~projector_file")).expanduser().resolve()
        self.repo_root = Path(rospy.get_param("~repo_root")).expanduser().resolve()
        self.output_root = Path(rospy.get_param("~output_root")).expanduser().resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)

        self.lanelet2 = None
        self.lanelet_map = None
        self.forbidden_lines = []
        self.line_lanelets = {}
        self.exception_lanelets = set()
        self._load_map()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.status_pub = rospy.Publisher(
            "/competition/rule_monitor/status", String, queue_size=10, latch=True
        )
        self.event_pub = rospy.Publisher(
            "/competition/rule_monitor/event", String, queue_size=100
        )
        self.marker_pub = rospy.Publisher(
            "/competition/rule_monitor/checkpoints", MarkerArray, queue_size=1, latch=True
        )
        self.wheel_pub = rospy.Publisher(
            "/competition/rule_monitor/wheels", MarkerArray, queue_size=1
        )

        rospy.Subscriber(
            rospy.get_param("~ego_topic"), EgoVehicleStatus, self._ego_cb, queue_size=20
        )
        rospy.Subscriber(rospy.get_param("~gear_topic"), Int8, self._gear_cb, queue_size=20)
        rospy.Subscriber(
            rospy.get_param("~collision_topic"), CollisionData, self._collision_cb, queue_size=20
        )

        self.run_dir = None
        self.events_file = None
        self.samples_file = None
        self.samples_writer = None
        self.run_started_ros = None
        self.run_started_wall = None
        self.last_sample_time = 0.0
        self.penalties = {"lane": 0, "speed": 0, "collision": 0}
        self.penalty_counts = {"lane": 0, "speed": 0, "collision": 0}
        self.lane_contact_wheels = set()
        self.lane_contact_lanelets = set()
        self.lane_contact_lines = set()

        rospy.Timer(rospy.Duration(0.05), self._timer_cb)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "competition_rule_monitor ready: map=v%d checkpoints=%d blackout=%s speed_exception_lanelets=%d",
            self.map_version,
            len(self.checkpoint_cfg),
            sorted(self.blackout_ids),
            len(self.exception_lanelets),
        )

    def _load_map(self):
        if not self.map_file.is_file() or not self.projector_file.is_file():
            raise RuntimeError("map/projector file missing")
        actual_sha = hashlib.sha256(self.map_file.read_bytes()).hexdigest()
        if actual_sha != self.expected_map_sha:
            raise RuntimeError(
                "map SHA mismatch: expected {}, got {}".format(
                    self.expected_map_sha, actual_sha
                )
            )
        ctypes.CDLL("libautoware_lanelet2_extension_lib.so", mode=ctypes.RTLD_GLOBAL)
        import lanelet2

        projector = yaml.safe_load(self.projector_file.read_text(encoding="utf-8"))
        origin_cfg = projector["map_origin"]
        origin = lanelet2.io.Origin(
            float(origin_cfg["latitude"]), float(origin_cfg["longitude"])
        )
        self.lanelet2 = lanelet2
        self.lanelet_map = lanelet2.io.load(
            str(self.map_file), lanelet2.projection.UtmProjector(origin)
        )

        forbidden = {
            (str(item["type"]), str(item["subtype"]))
            for item in rospy.get_param("~forbidden_line_types")
        }
        for line in self.lanelet_map.lineStringLayer:
            attrs = dict(line.attributes)
            if (attrs.get("type", ""), attrs.get("subtype", "")) in forbidden:
                self.forbidden_lines.append((int(line.id), lanelet2.geometry.to2D(line)))

        for lanelet in self.lanelet_map.laneletLayer:
            lanelet_id = int(lanelet.id)
            for bound in (lanelet.leftBound, lanelet.rightBound):
                self.line_lanelets.setdefault(int(bound.id), set()).add(lanelet_id)
            attrs = dict(lanelet.attributes)
            try:
                speed = float(attrs.get("speed_limit", "nan"))
            except ValueError:
                speed = float("nan")
            if attrs.get("subtype") == "road" and speed > self.exception_speed:
                self.exception_lanelets.add(lanelet_id)

        if len(self.exception_lanelets) != self.expected_exception_lanelets:
            raise RuntimeError(
                "high-speed exception lanelet count mismatch: expected {}, got {}".format(
                    self.expected_exception_lanelets, len(self.exception_lanelets)
                )
            )

        missing = self.blackout_ids.difference(
            int(lanelet.id) for lanelet in self.lanelet_map.laneletLayer
        )
        if missing:
            raise RuntimeError("GPS blackout lanelets missing: {}".format(sorted(missing)))

    def _transform_rule_points(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "morai_map", rospy.Time(0), rospy.Duration(0.02)
            ).transform
        except Exception:
            return False

        def convert(xyz):
            rotated = _rotate(transform.rotation, xyz)
            return (
                rotated[0] + transform.translation.x,
                rotated[1] + transform.translation.y,
                rotated[2] + transform.translation.z,
            )

        self.finish_point = convert(self.finish_cfg["xyz"])
        self.checkpoint_points = [
            (int(item["number"]), convert(item["xyz"])) for item in self.checkpoint_cfg
        ]
        self.morai_to_map = transform
        self.points_ready = True
        self._update_pose_from_ego()
        self._publish_markers()
        rospy.loginfo("competition checkpoints transformed morai_map -> map")
        return True

    def _ego_cb(self, msg):
        self.speed_kph = float(msg.velocity.x) * 3.6
        now = rospy.Time.now().to_sec()
        self.ego_stamp = now
        self.ego_position = (
            float(msg.position.x), float(msg.position.y), float(msg.position.z)
        )
        self.ego_heading_deg = float(msg.heading)
        self._update_pose_from_ego(now)

    def _update_pose_from_ego(self, now=None):
        if self.morai_to_map is None or self.ego_position is None:
            return
        transform = self.morai_to_map
        rotated = _rotate(transform.rotation, self.ego_position)
        transform_yaw = _yaw_from_quaternion(transform.rotation)
        self.pose = (
            rotated[0] + transform.translation.x,
            rotated[1] + transform.translation.y,
            rotated[2] + transform.translation.z,
            transform_yaw + math.radians(self.ego_heading_deg),
        )
        self.pose_stamp = rospy.Time.now().to_sec() if now is None else now

    def _gear_cb(self, msg):
        now = rospy.Time.now().to_sec()
        gear = int(msg.data)
        previous = self.gear
        self.previous_gear = previous
        self.gear = gear
        self.gear_stamp = now
        if gear == self.PARK:
            self.seen_park = True
            if self.state in ("FINISHED", "ERROR"):
                self._reset_ready()
        if (
            self.state == "READY"
            and self.seen_park
            and previous == self.PARK
            and gear == self.DRIVE
        ):
            if self._inputs_ready(now):
                self._start_run(now)
            else:
                self.error = "P->D observed before pose/speed/checkpoint inputs were ready"
                rospy.logerr(self.error)

    def _collision_cb(self, msg):
        current = {int(obj.unique_id) for obj in msg.collision_object}
        if self.state == "RUNNING":
            for object_id in sorted(current.difference(self.active_collision_ids)):
                self.penalties["collision"] += self.collision_penalty
                self.penalty_counts["collision"] += 1
                self._record_event(
                    "COLLISION_PENALTY",
                    {
                        "object_id": object_id,
                        "penalty_sec": self.collision_penalty,
                        "display": "COLLISION +{}s | object={}".format(
                            self.collision_penalty, object_id
                        ),
                    },
                    gui=True,
                )
        self.active_collision_ids = current

    def _inputs_ready(self, now):
        return (
            self.points_ready
            and self.pose is not None
            and self.speed_kph is not None
            and self.pose_stamp is not None
            and self.ego_stamp is not None
            and now - self.pose_stamp < 1.0
            and now - self.ego_stamp < 1.0
        )

    def _start_run(self, now):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        candidate = self.output_root / stamp
        suffix = 1
        while candidate.exists():
            candidate = self.output_root / "{}_{}".format(stamp, suffix)
            suffix += 1
        candidate.mkdir(parents=True)
        self.run_dir = candidate
        self.events_file = (candidate / "events.jsonl").open("w", encoding="utf-8")
        self.samples_file = (candidate / "samples.csv").open("w", encoding="utf-8", newline="")
        self.samples_writer = csv.DictWriter(
            self.samples_file,
            fieldnames=[
                "elapsed_sec", "x", "y", "yaw_rad", "speed_kph", "gear",
                "lanelet_ids", "gps_blackout", "speed_exception", "state",
            ],
        )
        self.samples_writer.writeheader()
        self.run_started_ros = now
        self.run_started_wall = time.time()
        self.state = "RUNNING"
        self.error = ""
        self.checkpoints.reset()
        self.finish_zone_departed = False
        self.penalties = {"lane": 0, "speed": 0, "collision": 0}
        self.penalty_counts = {"lane": 0, "speed": 0, "collision": 0}
        self.active_collision_ids.clear()
        self._write_manifest()
        self._record_event("RUN_START", {"gear": self.gear, "display": "RUN START | gear=D"})
        self._publish_markers()
        rospy.loginfo("competition run started: %s", self.run_dir)

    def _reset_ready(self):
        self._close_files()
        self.state = "READY"
        self.error = ""
        self.checkpoints.reset()
        self.finish_zone_departed = False
        self.run_started_ros = None
        self.run_dir = None
        self.speed_rule.end(rospy.Time.now().to_sec())
        self.lane_rule.end(rospy.Time.now().to_sec())
        self.active_collision_ids.clear()
        self._publish_markers()

    def _write_manifest(self):
        try:
            git_sha = subprocess.check_output(
                ["git", "-C", str(self.repo_root), "rev-parse", "HEAD"],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
        except Exception:
            git_sha = "unknown"
        manifest = {
            "rule_revision": self.rule_revision,
            "map_version": self.map_version,
            "map_file": str(self.map_file),
            "map_sha256": hashlib.sha256(self.map_file.read_bytes()).hexdigest(),
            "git_sha": git_sha,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "speed_limit_kph": self.speed_limit,
            "gps_blackout_lanelets": sorted(self.blackout_ids),
            "high_speed_exception_lanelets": sorted(self.exception_lanelets),
        }
        (self.run_dir / "manifest.yaml").write_text(
            yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False), encoding="utf-8"
        )

    def _elapsed(self, now=None):
        if self.run_started_ros is None:
            return 0.0
        return max(0.0, (rospy.Time.now().to_sec() if now is None else now) - self.run_started_ros)

    def _record_event(self, kind, data=None, gui=False):
        event = {
            "type": kind,
            "ros_time": rospy.Time.now().to_sec(),
            "elapsed_sec": self._elapsed(),
            "state": self.state,
        }
        event.update(data or {})
        if self.pose is not None:
            event["pose"] = {"x": self.pose[0], "y": self.pose[1], "yaw": self.pose[3]}
        if self.events_file is not None:
            self.events_file.write(json.dumps(event, ensure_ascii=False) + "\n")
            self.events_file.flush()
        if gui:
            self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def _lanelet_ids_at(self, x, y):
        point = self.lanelet2.core.BasicPoint2d(float(x), float(y))
        result = []
        for lanelet in self.lanelet_map.laneletLayer.nearest(point, 12):
            if self.lanelet2.geometry.inside(lanelet, point):
                result.append(int(lanelet.id))
        return sorted(set(result))

    def _wheel_points(self):
        x, y, _, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        result = {}
        for name, longitudinal, lateral in (
            ("FL", self.wheelbase, self.front_track / 2.0),
            ("FR", self.wheelbase, -self.front_track / 2.0),
            ("RL", 0.0, self.rear_track / 2.0),
            ("RR", 0.0, -self.rear_track / 2.0),
        ):
            result[name] = (
                x + c * longitudinal - s * lateral,
                y + s * longitudinal + c * lateral,
            )
        return result

    def _checkpoint_distance(self, point):
        # The official checkpoint is a circular area.  Judge entry using the
        # vehicle wheels, not only base_link (rear-axle center), so a vehicle
        # that physically enters the area is not rejected by its reference
        # point remaining just outside the radius.
        wheel_center_distance = min(
            math.hypot(x - point[0], y - point[1])
            for x, y in self._wheel_points().values()
        )
        return max(0.0, wheel_center_distance - self.tire_width / 2.0)

    def _lane_contacts(self, wheels):
        contacts = []
        threshold = self.tire_width / 2.0
        for wheel, xy in wheels.items():
            point = self.lanelet2.core.BasicPoint2d(*xy)
            for line_id, line in self.forbidden_lines:
                distance = self.lanelet2.geometry.distance(line, point)
                if distance <= threshold:
                    contacts.append((wheel, line_id, distance))
        return contacts

    def _finish_lane_contact(self, now):
        total = self.lane_rule.end(now)
        if total is None:
            return
        payload = {
            **total,
            "wheels": sorted(self.lane_contact_wheels),
            "lanelet_ids": sorted(self.lane_contact_lanelets),
            "line_ids": sorted(self.lane_contact_lines),
        }
        payload["display"] = (
            "LANE CONTACT END | duration={:.2f}s | penalty={} | lanelets={} | wheels={} | lines={}"
        ).format(
            total["duration"],
            "+{}s".format(total["penalty_seconds"]) if total["penalties"] else "none",
            ",".join(map(str, payload["lanelet_ids"])) or "unknown",
            ",".join(payload["wheels"]) or "unknown",
            ",".join(map(str, payload["line_ids"])) or "unknown",
        )
        # A sub-threshold contact is still essential debugging evidence.  It
        # must reach the GUI/event topic even when it earns no penalty.
        self._record_event("LANE_CONTACT_END", payload, gui=True)
        if total["penalties"] > 0:
            payload["display"] = (
                "LANE TOTAL | lanelets={} | duration={:.2f}s | penalties={} | total=+{}s"
            ).format(
                ",".join(map(str, payload["lanelet_ids"])) or "unknown",
                total["duration"], total["penalties"], total["penalty_seconds"],
            )
            self._record_event("LANE_TOTAL", payload, gui=True)
        self.lane_contact_wheels.clear()
        self.lane_contact_lanelets.clear()
        self.lane_contact_lines.clear()

    def _finish_speed_contact(self, now):
        total = self.speed_rule.end(now)
        if total is None:
            return
        self._record_event("SPEEDING_END", total)
        if total["penalties"] > 0:
            total["display"] = "SPEED TOTAL | duration={:.2f}s | penalties={} | total=+{}s".format(
                total["duration"], total["penalties"], total["penalty_seconds"]
            )
            self._record_event("SPEED_TOTAL", total, gui=True)

    def _evaluate_rules(self, now):
        x, y = self.pose[0], self.pose[1]
        self.current_lanelets = self._lanelet_ids_at(x, y)
        self.in_blackout = bool(self.blackout_ids.intersection(self.current_lanelets))
        self.in_speed_exception = bool(self.exception_lanelets.intersection(self.current_lanelets))

        finish_distance = math.hypot(x - self.finish_point[0], y - self.finish_point[1])
        if finish_distance > self.radius:
            self.finish_zone_departed = True

        next_number = self.checkpoints.next_number
        if next_number is not None:
            point = self.checkpoint_points[next_number - 1][1]
            checkpoint_distance = self._checkpoint_distance(point)
            if checkpoint_distance <= self.radius:
                if self.checkpoints.observe(next_number, True):
                    self._record_event(
                        "CHECKPOINT",
                        {
                            "checkpoint": next_number,
                            "distance_m": checkpoint_distance,
                            "display": "CP{} PASSED".format(next_number),
                        },
                    )
                    self._publish_markers()
        if self.finish_zone_departed and finish_distance <= self.radius:
            if self.checkpoints.next_number is None:
                self._finish_run(now)
            else:
                missing = list(range(self.checkpoints.next_number, self.checkpoints.count + 1))
                self._fail_run("finish crossed with missing checkpoints: {}".format(missing))
            return

        speeding = is_speeding(self.speed_kph, self.speed_limit, self.in_speed_exception)
        if speeding and not self.speed_rule.active:
            for threshold, penalty in self.speed_rule.begin(now):
                self._issue_speed_penalty(threshold, penalty)
            self._record_event("SPEEDING_BEGIN", {"speed_kph": self.speed_kph})
        elif speeding:
            for threshold, penalty in self.speed_rule.update(now):
                self._issue_speed_penalty(threshold, penalty)
        elif self.speed_rule.active:
            self._finish_speed_contact(now)

        wheels = self._wheel_points()
        contacts = [] if self.in_blackout else self._lane_contacts(wheels)
        if contacts and not self.lane_rule.active:
            self.lane_rule.begin(now)
            self._record_event(
                "LANE_CONTACT_BEGIN",
                {
                    "wheels": sorted({item[0] for item in contacts}),
                    "lanelet_ids": self.current_lanelets,
                    "line_ids": sorted({item[1] for item in contacts}),
                    "display": "LANE CONTACT BEGIN | lanelets={} | wheels={} | lines={}".format(
                        ",".join(map(str, self.current_lanelets)) or "unknown",
                        ",".join(sorted({item[0] for item in contacts})) or "unknown",
                        ",".join(map(str, sorted({item[1] for item in contacts}))) or "unknown",
                    ),
                },
                gui=True,
            )
        if contacts:
            self.lane_contact_wheels.update(item[0] for item in contacts)
            self.lane_contact_lines.update(item[1] for item in contacts)
            self.lane_contact_lanelets.update(self.current_lanelets)
            for line_id in self.lane_contact_lines:
                self.lane_contact_lanelets.update(self.line_lanelets.get(line_id, ()))
            for threshold, penalty in self.lane_rule.update(now):
                self.penalties["lane"] += penalty
                self.penalty_counts["lane"] += 1
                started = self._elapsed(self.lane_rule.started_at)
                self._record_event(
                    "LANE_PENALTY",
                    {
                        "contact_threshold_sec": threshold,
                        "contact_started_elapsed_sec": started,
                        "penalty_sec": penalty,
                        "wheels": sorted(self.lane_contact_wheels),
                        "lanelet_ids": sorted(self.lane_contact_lanelets),
                        "line_ids": sorted(self.lane_contact_lines),
                        "display": (
                            "LANE +{}s | lanelets={} | wheels={} | contact={:.2f}s | started={:.2f}s"
                        ).format(
                            penalty,
                            ",".join(map(str, sorted(self.lane_contact_lanelets))) or "unknown",
                            ",".join(sorted(self.lane_contact_wheels)), threshold, started,
                        ),
                    },
                    gui=True,
                )
        elif self.lane_rule.active:
            self._finish_lane_contact(now)

        self._publish_wheels(wheels, contacts)
        if now - self.last_sample_time >= 0.1:
            self._write_sample(now)
            self.last_sample_time = now

    def _issue_speed_penalty(self, threshold, penalty):
        self.penalties["speed"] += penalty
        self.penalty_counts["speed"] += 1
        self._record_event(
            "SPEED_PENALTY",
            {
                "speed_kph": self.speed_kph,
                "threshold_sec": threshold,
                "penalty_sec": penalty,
                "lanelet_ids": self.current_lanelets,
                "display": "SPEED +{}s | {:.3f}km/h | continuous={:.2f}s".format(
                    penalty, self.speed_kph, threshold
                ),
            },
            gui=True,
        )

    def _write_sample(self, now):
        if self.samples_writer is None:
            return
        self.samples_writer.writerow({
            "elapsed_sec": "{:.3f}".format(self._elapsed(now)),
            "x": "{:.6f}".format(self.pose[0]),
            "y": "{:.6f}".format(self.pose[1]),
            "yaw_rad": "{:.8f}".format(self.pose[3]),
            "speed_kph": "{:.6f}".format(self.speed_kph),
            "gear": self.gear,
            "lanelet_ids": ";".join(map(str, self.current_lanelets)),
            "gps_blackout": int(self.in_blackout),
            "speed_exception": int(self.in_speed_exception),
            "state": self.state,
        })
        self.samples_file.flush()

    def _finish_run(self, now):
        self._finish_lane_contact(now)
        self._finish_speed_contact(now)
        self.state = "FINISHED"
        elapsed = self._elapsed(now)
        total_penalty = sum(self.penalties.values())
        summary = {
            "state": self.state,
            "elapsed_sec": elapsed,
            "checkpoint_count": self.checkpoints.passed,
            "penalty_counts": dict(self.penalty_counts),
            "penalty_seconds": dict(self.penalties),
            "total_penalty_sec": total_penalty,
            "total_time_sec": elapsed + total_penalty,
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        (self.run_dir / "summary.yaml").write_text(
            yaml.safe_dump(summary, allow_unicode=True, sort_keys=False), encoding="utf-8"
        )
        category_totals = []
        for name in ("lane", "speed", "collision"):
            if self.penalty_counts[name] > 0:
                category_totals.append(
                    "{}={} (+{}s)".format(
                        name.upper(), self.penalty_counts[name], self.penalties[name]
                    )
                )
        if not category_totals:
            category_totals.append("NO PENALTIES")
        display = "RUN TOTAL | {} | TOTAL PENALTY=+{}s | TIME={:.2f}s".format(
            " | ".join(category_totals), total_penalty, elapsed + total_penalty
        )
        self._record_event("RUN_TOTAL", {**summary, "display": display}, gui=True)
        self._publish_markers()
        self._close_files()
        rospy.loginfo("competition run finished: %s", display)

    def _fail_run(self, reason):
        self.error = reason
        self.state = "ERROR"
        self._record_event("ERROR", {"reason": reason, "display": "ERROR | " + reason}, gui=True)
        if self.run_dir is not None:
            summary = {
                "state": "ERROR", "reason": reason, "elapsed_sec": self._elapsed(),
                "penalty_counts": dict(self.penalty_counts),
                "penalty_seconds": dict(self.penalties),
            }
            (self.run_dir / "summary.yaml").write_text(
                yaml.safe_dump(summary, allow_unicode=True, sort_keys=False), encoding="utf-8"
            )
        self._close_files()

    def _timer_cb(self, _event):
        now = rospy.Time.now().to_sec()
        if not self.points_ready:
            self._transform_rule_points()
        # Do not make launch order a hidden prerequisite.  The normal path is
        # still P -> D, but if the monitor is started after the controller has
        # already selected D, begin as soon as all judging inputs are ready.
        if (
            self.state == "READY"
            and self.gear == self.DRIVE
            and self._inputs_ready(now)
            and (self.seen_park or self.allow_late_drive_start)
        ):
            self._start_run(now)
        if self.state == "RUNNING":
            if not self._inputs_ready(now) or self.gear_stamp is None or now - self.gear_stamp >= 2.0:
                self._fail_run("required pose/speed/gear input timeout")
            else:
                self._evaluate_rules(now)
        self._publish_status(now)

    def _publish_status(self, now):
        payload = {
            "state": self.state,
            "error": self.error,
            "elapsed_sec": self._elapsed(now),
            "speed_kph": self.speed_kph,
            "speed_limit_kph": self.speed_limit,
            "gear": self.gear,
            "checkpoints_passed": self.checkpoints.passed,
            "checkpoints_total": self.checkpoints.count,
            "next_checkpoint": self.checkpoints.next_number,
            "gps_blackout": self.in_blackout,
            "speed_exception": self.in_speed_exception,
            "current_lanelets": self.current_lanelets,
            "penalty_counts": self.penalty_counts,
            "penalty_seconds": self.penalties,
            "total_penalty_sec": sum(self.penalties.values()),
            "run_directory": str(self.run_dir) if self.run_dir else "",
            "inputs_ready": self._inputs_ready(now),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_markers(self):
        if not self.points_ready:
            return
        array = MarkerArray()
        points = [("START / FINISH", self.finish_point)] + [
            ("CP{}".format(number), point) for number, point in self.checkpoint_points
        ]
        for index, (label, point) in enumerate(points):
            passed = self.state == "FINISHED" if index == 0 else index <= self.checkpoints.passed
            color = (0.78, 0.78, 0.80) if passed else (0.62, 0.20, 0.86)
            cylinder = Marker()
            cylinder.header.frame_id = "map"
            cylinder.header.stamp = rospy.Time.now()
            cylinder.ns = "competition_checkpoints"
            cylinder.id = index * 2
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            cylinder.pose.position.x, cylinder.pose.position.y = point[0], point[1]
            cylinder.pose.position.z = point[2] + 0.05
            cylinder.pose.orientation.w = 1.0
            cylinder.scale.x = self.radius * 2.0
            cylinder.scale.y = self.radius * 2.0
            cylinder.scale.z = 0.10
            cylinder.color.r, cylinder.color.g, cylinder.color.b = color
            cylinder.color.a = 0.28
            array.markers.append(cylinder)
            text = Marker()
            text.header = cylinder.header
            text.ns = "competition_checkpoint_labels"
            text.id = index * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = point[0], point[1]
            text.pose.position.z = point[2] + 2.0
            text.pose.orientation.w = 1.0
            text.scale.z = 1.2
            text.color.r, text.color.g, text.color.b, text.color.a = (*color, 0.95)
            text.text = label
            array.markers.append(text)
        self.marker_pub.publish(array)

    def _publish_wheels(self, wheels, contacts):
        contact_wheels = {item[0] for item in contacts}
        array = MarkerArray()
        for index, (name, point) in enumerate(sorted(wheels.items())):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "competition_wheels"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = point
            marker.pose.position.z = self.pose[2] + 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = self.tire_width
            marker.scale.z = 0.08
            if name in contact_wheels:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.05, 0.05
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.9, 0.2
            marker.color.a = 0.9
            array.markers.append(marker)
        self.wheel_pub.publish(array)

    def _close_files(self):
        for handle_name in ("events_file", "samples_file"):
            handle = getattr(self, handle_name)
            if handle is not None and not handle.closed:
                handle.flush()
                handle.close()
            setattr(self, handle_name, None)
        self.samples_writer = None

    def _shutdown(self):
        if self.state == "RUNNING":
            self._record_event("RUN_ABORTED", {"reason": "node shutdown"})
        self._close_files()


def main():
    rospy.init_node("competition_rule_monitor")
    try:
        CompetitionRuleMonitor()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("competition_rule_monitor failed: %s", exc)
        raise
