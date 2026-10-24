#!/usr/bin/env python3
"""Development-only interventions around the unchanged real validator pipeline.

No alternative driving controller is used. The stock validator owns sensors,
localization, od_gt, tracking/prediction, ROS1 planning, PP/PI and UDP transport.
This wrapper only inserts an observed Park hold or a stopped ScenarioLoad.
"""
import argparse
import dataclasses
import hashlib
import json
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "base_ws/src/planning_addons/planning_scenario_validator/src"))
from planning_scenario_validator import runner as r
from planning_scenario_validator import runtime as rt


def near_link(point, yaw_deg):
    data = json.loads((ROOT / "kcity_map/mgeo_map/link_set.json").read_text())
    links = data if isinstance(data, list) else data.get("links", [])
    best = None
    for link in links:
        pts = link.get("points", [])
        lengths = [math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(pts, pts[1:])]
        total, covered = sum(lengths), 0.0
        if not total:
            continue
        for a, b, length in zip(pts, pts[1:], lengths):
            if not length:
                continue
            heading = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
            difference = abs((heading-yaw_deg+180.0) % 360.0 - 180.0)
            if difference > 60.0:
                covered += length
                continue
            t = max(0.0, min(1.0, ((point[0]-a[0])*(b[0]-a[0]) +
                                  (point[1]-a[1])*(b[1]-a[1]))/(length*length)))
            distance = math.hypot(point[0]-(a[0]+t*(b[0]-a[0])),
                                  point[1]-(a[1]+t*(b[1]-a[1])))
            value = (distance, link.get("idx"), (covered+t*length)/total)
            if best is None or value[0] < best[0]:
                best = value
            covered += length
    if best is None or best[0] > 3.0:
        raise ValueError("Checkpoint has no confirmed nearby direction-compatible MGeo link")
    return best


def checkpoint_case(case, cfg, folder, number, known_prior=False):
    import yaml
    rules = yaml.safe_load((ROOT / "base_ws/src/competition/competition_rule_monitor/config/kcity_v87_rules.yaml").read_text())
    point = next(row for row in rules["checkpoints"] if row["number"] == number)
    scene = json.loads(case.source.read_text())
    initial = scene["egoVehicle"]["initPosition"]
    xyz, yaw = point["xyz"], point["yaw_deg"]
    for index, key in enumerate(("x", "y", "z")):
        initial["pos"][key] = xyz[index]
        initial["pos"]["_"+key] = "{:.3f}".format(xyz[index])
    initial["rot"]["yaw"] = "{:.9f}".format(yaw)
    distance, link, ratio = near_link(xyz, yaw)
    initial["initLink"], initial["initLinkRatio"] = link, ratio
    path = folder / "checkpoint_target.json"
    rt.atomic_json(path, scene)
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    target = dataclasses.replace(case, source=path, install_stem="pv_cp{}_{}".format(number, sha[:8]),
                                 source_sha256=sha)
    rt.atomic_json(folder / "checkpoint_target_metadata.json", {
        "kind": "ScenarioLoad emulation, not an official reset event",
        "checkpoint": number, "xyz_morai": xyz, "scenario_yaw_deg": yaw,
        "nearest_link": link, "link_ratio": ratio, "link_distance_m": distance,
        "position_runtime_source": ("explicit lab checkpoint position prior; Ego is safety/oracle only"
                                    if known_prior else "valid GNSS only; Ego is safety/oracle only")})
    return target


def position_errors(monitor):
    if monitor.ego is None or monitor.odom is None or monitor.control_odom is None:
        return None
    import rospy
    import tf2_ros
    if not hasattr(monitor, "trial_tf_buffer"):
        monitor.trial_tf_buffer = tf2_ros.Buffer()
        monitor.trial_tf_listener = tf2_ros.TransformListener(monitor.trial_tf_buffer)
    transform = monitor.trial_tf_buffer.lookup_transform("map", "morai_map", rospy.Time(0), rospy.Duration(.1))
    q = transform.transform.rotation
    angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    x = math.cos(angle)*monitor.ego.position.x-math.sin(angle)*monitor.ego.position.y+transform.transform.translation.x
    y = math.sin(angle)*monitor.ego.position.x+math.cos(angle)*monitor.ego.position.y+transform.transform.translation.y
    def error(odom):
        p = odom.pose.pose.position
        return math.hypot(p.x-x, p.y-y)
    oracle_yaw = math.radians(monitor.ego.heading - 1.347) + angle
    def yaw_error(odom):
        q = odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return abs(math.degrees(math.atan2(math.sin(yaw-oracle_yaw), math.cos(yaw-oracle_yaw))))
    return {"global_xy_error_m": error(monitor.odom), "smooth_xy_error_m": error(monitor.control_odom),
            "global_yaw_error_deg": yaw_error(monitor.odom),
            "smooth_yaw_error_deg": yaw_error(monitor.control_odom),
            "ego_xy_morai": [monitor.ego.position.x, monitor.ego.position.y]}


def validate_corrected_heading(orientation, now_sec):
    age = now_sec - orientation.header.stamp.to_sec()
    q = orientation.orientation
    components = (q.x, q.y, q.z, q.w)
    if not (math.isfinite(age) and 0 <= age <= .1
            and orientation.header.frame_id.lstrip('/') == 'base_link'
            and all(math.isfinite(x) for x in components)
            and abs(math.hypot(*components) - 1.) < 1e-3
            and all(math.isfinite(x) for x in orientation.orientation_covariance)
            and orientation.orientation_covariance[0] >= 0):
        raise rt.SafetyFailure('Known reset prior requires fresh corrected Competition heading')
    return q


def publish_known_checkpoint_prior(monitor, folder):
    """Emulate an authoritative return pose, not GPS and not runtime Ego GT.

    The driver of this experiment knows the pose because it just requested
    ScenarioLoad. Real deployment requires an approved checkpoint event and
    known placement accuracy; the localization stack cannot infer this event.
    """
    import copy
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import Imu
    meta = json.loads((folder / "checkpoint_target_metadata.json").read_text())
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    transform = buffer.lookup_transform('map', 'morai_map', rospy.Time(0), rospy.Duration(3))
    rquat = transform.transform.rotation
    angle = math.atan2(2*(rquat.w*rquat.z+rquat.x*rquat.y), 1-2*(rquat.y*rquat.y+rquat.z*rquat.z))
    x, y, z = meta['xyz_morai']
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x = math.cos(angle)*x-math.sin(angle)*y+transform.transform.translation.x
    pose.pose.pose.position.y = math.sin(angle)*x+math.cos(angle)*y+transform.transform.translation.y
    pose.pose.pose.position.z = z+transform.transform.translation.z
    pose.pose.covariance = [0.] * 36
    pose.pose.covariance[0] = pose.pose.covariance[7] = .01
    for index in (14, 21, 28):
        pose.pose.covariance[index] = 1e6
    pose.pose.covariance[35] = math.radians(1.)**2
    publisher = rospy.Publisher('/localization/initialpose', PoseWithCovarianceStamped,
                                queue_size=1, latch=False)
    monitor.wait(lambda: publisher.get_num_connections() > 0, 3, 'Global reset subscriber')
    try:
        # TF/subscriber setup may take time. Re-check Park and get a new heading
        # immediately before the sole publication instead of trusting old caches.
        monitor.require_fresh_inputs(('ego', 'gear', 'control_mode', 'competition', 'imu'),
                                     freshness_sec=.2)
        if monitor.raw_ego_speed_mps() > .05 or monitor.gear != 1 or monitor.control_mode != 2:
            raise rt.SafetyFailure('Known checkpoint prior requires observed stationary Park')
        orientation = rospy.wait_for_message('/localization/gt/orientation', Imu, timeout=3)
        q = validate_corrected_heading(orientation, rospy.Time.now().to_sec())
        # Use a current event epoch; never rewrite any UDP/sensor message timestamp.
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.orientation = copy.deepcopy(q)  # already map heading; no second rotation
        publisher.publish(pose)
        rt.atomic_json(folder / 'known_checkpoint_prior_receipt.json', {
            'source': 'explicit development ScenarioLoad target, NOT inferred from Ego/GPS',
            'checkpoint': meta['checkpoint'], 'xy_map': [pose.pose.pose.position.x, pose.pose.pose.position.y],
            'heading_source': 'fresh canonical corrected Competition orientation',
            'published_once': True, 'event_stamp': pose.header.stamp.to_sec(),
            'deployment_gate': 'official reset event and known return placement uncertainty required'})
        monitor.event('LOCALIZATION_TRIAL_KNOWN_CHECKPOINT_PRIOR', checkpoint=meta['checkpoint'])
        rospy.sleep(.2)
    finally:
        publisher.unregister()


class Interventions:
    def __init__(self, args):
        self.args, self.context, self.used = args, None, False
        self.original_case = r._run_case
        self.original_loop = rt.LiveMonitor.run_until_done

    def case(self, cfg, processes, monitor, case, state):
        self.context = (cfg, processes, case)
        self.used = False
        return self.original_case(cfg, processes, monitor, case, state)

    def resume_controller(self, cfg, processes, monitor):
        processes.stop("parking_prepare")
        processes.start("udp_control", r._launch("sensor_interface", "morai_udp_interface.launch",
            network_config=cfg["network"]["ego_network_json"], enable_sensors=False,
            enable_lidar=False, enable_ego_network=False, enable_control=True))
        monitor.wait(lambda: r._control_sender_ready(processes), 10, "owned D sender")
        rt.publish_brake(duration=.2)
        restart = time.monotonic()
        processes.start("controller", r._launch("trajectory_follower", "pp_pi_autoware.launch",
            kinematic_state_topic="/localization/control_kinematic_state"))
        monitor.wait(lambda: monitor.control_source == "/longi_pi_autoware" and
                     monitor.stamps.get("control", 0.0) > restart, 15, "resumed fresh PP/PI owner")
        r._verify_control_owner(monitor)
        monitor.event("LOCALIZATION_TRIAL_CONTROLLER_RESUMED")

    def intervene(self, monitor):
        cfg, processes, case = self.context
        folder = monitor.events_path.parent
        monitor.event("LOCALIZATION_TRIAL_STOP_BEGIN", mode=self.args.mode)
        processes.stop("controller")
        stop = r._stop_and_verify(cfg, processes, monitor)
        if stop["status"] != "VERIFIED" or not stop["stop"].get("park_verified"):
            raise rt.SafetyFailure("Trial stop/P not verified")
        hold = rt.BrakeHold()
        hold.start()
        try:
            processes.start("parking_prepare", ["roslaunch", str(ROOT /
                "base_ws/src/planning_addons/planning_scenario_validator/launch/parking_hold.launch"),
                "network_config:="+str(cfg["network"]["ego_network_json"])])
            monitor.wait(lambda: "/morai_ctrl_cmd_udp_sender" in dict(
                __import__("rosgraph").Master("/trial_hold").getSystemState()[1]).get("/ctrl_cmd", []),
                10, "owned P sender")
            after = time.monotonic()
            monitor.verify_stopped(after=after, timeout=10, hold_sec=1.0, require_park=True)
            if self.args.mode == "checkpoint":
                target = checkpoint_case(case, cfg, folder, self.args.checkpoint,
                                         known_prior=self.args.known_checkpoint_prior)
                installed, ego_xy = rt.prepare_scenario(target, cfg["install"], cfg["network"]["ego_network_json"])
                before = None if monitor.ego is None else [monitor.ego.position.x, monitor.ego.position.y]
                monitor.event("LOCALIZATION_TRIAL_TELEPORT_REQUEST", checkpoint=self.args.checkpoint, before=before)
                r._load_scene(cfg, installed, monitor, ego_xy)
                monitor.event("LOCALIZATION_TRIAL_TELEPORT_CONFIRMED", target=list(ego_xy))
                after = time.monotonic()
                monitor.wait_stationary(after=after, timeout=15, hold_sec=1.0,
                    require_valid_gps=not self.args.known_checkpoint_prior, check_preparation_collision=True)
                if self.args.known_checkpoint_prior:
                    publish_known_checkpoint_prior(monitor, folder)
                records, deadline, stable_since = [], time.monotonic()+self.args.recovery_timeout, None
                while time.monotonic() < deadline:
                    monitor.require_fresh_inputs(("ego", "odom", "control_state", "competition", "gps", "imu"),
                                                 after=after, freshness_sec=1.0)
                    if monitor.raw_ego_speed_mps() > .05 or monitor.gear != 1:
                        raise rt.SafetyFailure("Vehicle moved during checkpoint verification")
                    try:
                        errors = position_errors(monitor)
                    except Exception:
                        errors = None
                    if errors is not None:
                        errors["stamp"] = time.time()
                        records.append(errors)
                        aligned = (max(errors["global_xy_error_m"], errors["smooth_xy_error_m"]) < .25
                                   and max(errors["global_yaw_error_deg"], errors["smooth_yaw_error_deg"]) < 1.0)
                        stable_since = time.monotonic() if aligned and stable_since is None else stable_since if aligned else None
                        if stable_since is not None and time.monotonic()-stable_since >= .5:
                            break
                    time.sleep(.05)
                rt.atomic_json(folder/"checkpoint_recovery_observations.json", records)
                if stable_since is None or time.monotonic()-stable_since < .5:
                    raise rt.SafetyFailure("Global/Smooth failed independent GT recovery gate; no driving restart")
                monitor.event("LOCALIZATION_TRIAL_RECOVERED", errors=records[-1])
                route = rt.request_route(case, monitor, timeout=45, process=processes.running["planning"])
                monitor.event("LOCALIZATION_TRIAL_ROUTE_READY", route=route)
            else:
                monitor.event("LOCALIZATION_TRIAL_SHADOW_HOLD_BEGIN", seconds=self.args.hold_sec)
                until = time.monotonic()+self.args.hold_sec
                while time.monotonic() < until:
                    monitor.require_fresh_inputs(("ego", "odom", "control_state", "competition", "gps", "imu"),
                                                 freshness_sec=1.0)
                    if monitor.raw_ego_speed_mps() > .05 or monitor.gear != 1:
                        raise rt.SafetyFailure("Vehicle moved during shadow Park hold")
                    if monitor.gps_is_valid():
                        raise rt.SafetyFailure("GPS became valid: not an actual shadow hold")
                    if monitor.collision_count:
                        raise rt.SafetyFailure("Collision during shadow Park hold")
                    time.sleep(.1)
                monitor.event("LOCALIZATION_TRIAL_SHADOW_HOLD_END")
            monitor.wait_stationary(after=after, timeout=10, hold_sec=1.0,
                                    require_valid_gps=False, check_preparation_collision=True)
            hold.stop()
            self.resume_controller(cfg, processes, monitor)
        finally:
            hold.stop()

    def loop(self, monitor, case, validation, started, callback=None):
        rt.atomic_json(monitor.events_path.parent / "intervention_receipt.json", {
            "harness_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "arguments": vars(self.args),
            "kind": "development intervention around stock real closed loop",
        })
        def progress(metrics):
            if callback:
                callback(metrics)
            trigger = metrics["elapsed_sec"] >= self.args.trigger_sec
            if self.args.mode == "shadow_hold":
                trigger = trigger and not monitor.gps_is_valid()
            if not self.used and trigger:
                self.used = True
                self.intervene(monitor)
        result = self.original_loop(monitor, case, validation, started, progress)
        if result[0] == "SUCCESS" and not self.used:
            raise rt.RuntimeFailure("Goal reached without required intervention")
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config")
    parser.add_argument("--mode", choices=("checkpoint", "shadow_hold"), required=True)
    parser.add_argument("--checkpoint", type=int, default=13)
    parser.add_argument("--trigger-sec", type=float, default=2.0)
    parser.add_argument("--hold-sec", type=float, default=30.0)
    parser.add_argument("--recovery-timeout", type=float, default=20.0)
    parser.add_argument("--known-checkpoint-prior", action="store_true",
                        help="Development-only explicit known return position; not an automatic checkpoint detector")
    parser.add_argument("--execute", action="store_true", help="Explicit simulator actuation authorization")
    args = parser.parse_args()
    if not (1 <= args.checkpoint <= 15) or any(not math.isfinite(x) or x <= 0 for x in
            (args.trigger_sec, args.hold_sec, args.recovery_timeout)):
        parser.error("finite positive trial times/checkpoint required")
    if args.known_checkpoint_prior and args.mode != 'checkpoint':
        parser.error('known prior is checkpoint-only')
    cfg = r.load_campaign(args.config)
    if not cfg["localization_safety"].get("enabled") or cfg["stack"]["pose_source"] != "localization" or (
            cfg["stack"]["detector"] != "od_gt" or cfg["stack"].get("traffic_light")):
        parser.error("trial requires localization + od_gt + lights OFF + safety ON")
    if len(cfg["cases"]) != 1:
        parser.error("one scene per intervention trial; never auto-advance after uncertain recovery")
    if not args.execute:
        return r.main([args.config, "--check"])
    hooks = Interventions(args)
    r._run_case = hooks.case
    rt.LiveMonitor.run_until_done = lambda monitor, *a, **k: hooks.loop(monitor, *a, **k)
    return r.main([args.config])


if __name__ == "__main__":
    raise SystemExit(main())
