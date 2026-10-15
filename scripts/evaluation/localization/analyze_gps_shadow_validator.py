#!/usr/bin/env python3
"""Independent, phase-aware localization/closed-loop analysis of validator bags.

No ROS node is started. Raw Ego is an offline oracle only. Localization error
and actual Ego distance to the recorded planning trajectory remain separate.
Run with the ROS system Python after sourcing /opt/ros/noetic/setup.bash.
"""

import argparse
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np


STATE_TOPICS = {
    "global_state": "/localization/kinematic_state",
    "smooth_state": "/localization/control_kinematic_state",
    "global_ekf": "/localization/odom",
    "local_ekf": "/localization/odometry/local",
}
RAW_TOPICS = ["/Ego_topic", "/sensor/gps", "/sensor/imu", "/Competition_topic",
              "/competition/raw_twist", "/localization/imu",
              "/localization/gt/orientation", "/localization/gt/twist",
              "/localization/gnss/odom", "/localization/initialpose",
              "/tf", "/tf_static", "/localization/acceleration",
              "/planning/trajectory", "/ctrl_cmd", "/control/command/steering",
              "/control/autoware/controller_valid",
              "/purepursuit_autoware/debug/lateral_error", "/Object_topic",
              "/perception/object_recognition/detection/objects",
              "/perception/object_recognition/tracking/objects",
              "/perception/object_recognition/objects"]


def wrapped(angle):
    with np.errstate(invalid="ignore"):
        return (np.asarray(angle) + math.pi) % (2 * math.pi) - math.pi


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def statistics(values):
    """Keep signed and absolute distributions explicit, including zero samples."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {"count": 0}
    absolute = np.abs(values)
    def distribution(a):
        return {"mean": float(np.mean(a)), "p50": float(np.percentile(a, 50)),
                "p90": float(np.percentile(a, 90)), "p95": float(np.percentile(a, 95)),
                "min": float(np.min(a)), "max": float(np.max(a))}
    return {"count": int(values.size), "signed": distribution(values),
            "absolute": distribution(absolute), "rmse": float(np.sqrt(np.mean(values**2)))}


def header_time(message, receipt):
    header = getattr(message, "header", None)
    if header is not None:
        value = header.stamp.to_sec()
        if math.isfinite(value) and value > 0:
            return value
    return receipt


def inspect_bag(path, time_axis):
    import rosbag  # Lazy import permits pure numeric unit tests without ROS.
    rows = {topic: [] for topic in set(RAW_TOPICS + list(STATE_TOPICS.values()))}
    timing = {}
    frames = {}
    header_fallback={}
    static_tf = []
    trajectories = []
    tf_rows = []
    object_content={}
    with rosbag.Bag(str(path), "r") as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        topics = bag.get_type_and_topic_info().topics
        counts = {name: value.message_count for name, value in topics.items()}
        for topic, msg, bt in bag.read_messages(topics=list(rows)):
            receipt = bt.to_sec()
            ht = header_time(msg, receipt)
            header=getattr(msg,"header",None)
            if header is None or header.stamp.to_sec()<=0:
                header_fallback[topic]=header_fallback.get(topic,0)+1
            tm = ht if time_axis == "header" else receipt
            timing.setdefault(topic, []).append((ht, receipt))
            if hasattr(msg, "header"):
                frames.setdefault(topic, set()).add(msg.header.frame_id)
            if topic == "/tf_static":
                for tf in msg.transforms:
                    if tf.header.frame_id.lstrip("/") == "map" and tf.child_frame_id.lstrip("/") == "morai_map":
                        t = tf.transform.translation
                        static_tf.append((t.x, t.y, yaw(tf.transform.rotation)))
            elif topic == "/tf":
                for tf in msg.transforms:
                    if tf.header.frame_id.lstrip("/") == "map" and tf.child_frame_id.lstrip("/") == "odom":
                        t = tf.transform.translation
                        ts = tf.header.stamp.to_sec() if time_axis == "header" else receipt
                        tf_rows.append((ts, t.x, t.y, yaw(tf.transform.rotation), receipt))
            elif topic == "/Ego_topic":
                rows[topic].append((tm, msg.position.x, msg.position.y, math.radians(msg.heading),
                                    msg.velocity.x, msg.velocity.y, msg.acceleration.y, receipt))
            elif topic in STATE_TOPICS.values():
                p, tw = msg.pose.pose.position, msg.twist.twist
                rows[topic].append((tm, p.x, p.y, yaw(msg.pose.pose.orientation), tw.linear.x,
                                    tw.linear.y, tw.angular.z, msg.twist.covariance[7], receipt))
            elif topic == "/sensor/gps":
                finite = math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
                valid = (finite and abs(msg.latitude) <= 90 and abs(msg.longitude) <= 180
                         and not (msg.latitude == 0 and msg.longitude == 0))
                rows[topic].append((tm, float(valid), msg.latitude, msg.longitude,
                                    float(msg.status), ht, receipt))
            elif topic == "/localization/acceleration":
                a = msg.accel.accel.linear
                rows[topic].append((tm, a.x, a.y, receipt))
            elif topic == "/planning/trajectory":
                trajectories.append({"time": tm, "header": ht, "receipt": receipt,
                    "frame": msg.header.frame_id.lstrip("/"),
                    "xy": np.asarray([(p.pose.position.x, p.pose.position.y) for p in msg.points], dtype=float)})
            elif topic == "/ctrl_cmd":
                rows[topic].append((tm, msg.steering, msg.accel, msg.brake, receipt))
            elif topic in ("/control/command/steering", "/purepursuit_autoware/debug/lateral_error", "/control/autoware/controller_valid"):
                rows[topic].append((tm, float(msg.data), receipt))
            elif topic == "/Object_topic" or topic.startswith("/perception/object_recognition/"):
                report=object_content.setdefault(topic,{"messages":0,"nonempty_messages":0,
                    "object_instances":0,"dimensions":set(),"nonfinite_dimensions":0})
                objects=list(msg.npc_list)+list(msg.pedestrian_list)+list(msg.obstacle_list) if topic=="/Object_topic" else msg.objects
                report["messages"]+=1
                report["nonempty_messages"]+=bool(objects)
                report["object_instances"]+=len(objects)
                for obj in objects:
                    dims=obj.size if topic=="/Object_topic" else obj.shape.dimensions
                    dimension=tuple(float(v) for v in (dims.x,dims.y,dims.z))
                    if not all(math.isfinite(v) for v in dimension): report["nonfinite_dimensions"]+=1
                    else: report["dimensions"].add(tuple(round(v,6) for v in dimension))
                rows[topic].append((tm,receipt))
            else:
                rows[topic].append((tm, receipt))
    if not static_tf:
        raise ValueError("Recorded map -> morai_map static TF is required for independent GT.")
    if any(not np.allclose(value, static_tf[0], atol=1e-8) for value in static_tf[1:]):
        raise ValueError("map -> morai_map static TF changes within bag; use separate runs.")
    if not rows["/Ego_topic"]:
        raise ValueError("Raw /Ego_topic oracle is required.")
    if not rows["/sensor/gps"]:
        raise ValueError("Raw /sensor/gps is required for phase classification.")
    return {"rows": {k: np.asarray(v, dtype=float) for k, v in rows.items()},
            "timing": timing, "frames": {k: sorted(v) for k, v in frames.items()},
            "counts": counts, "transform": static_tf[0], "trajectories": trajectories,
            "tf": np.asarray(tf_rows, dtype=float), "start": start, "end": end,
            "header_fallback":header_fallback,
            "object_content":{k:dict(v,dimensions=sorted(v['dimensions'])) for k,v in object_content.items()}}


def timing_summary(rows):
    arr = np.asarray(rows, dtype=float)
    if arr.size == 0:
        return {"count": 0}
    hd, rd = np.diff(arr[:, 0]), np.diff(arr[:, 1])
    duration = arr[-1, 1] - arr[0, 1]
    return {"count": len(arr), "receipt_hz": (len(arr)-1)/duration if duration > 0 else None,
            "header_backward_count": int(np.count_nonzero(hd < 0)),
            "header_duplicate_count": int(np.count_nonzero(hd == 0)),
            "header_gap_sec": statistics(hd[hd > 0]), "receipt_gap_sec": statistics(rd[rd > 0]),
            "receipt_minus_header_sec": statistics(arr[:, 1]-arr[:, 0])}


def split_oracle(ego, max_gap=.25, settle=.5):
    """Never sort samples across clock resets or interpolate through load teleports."""
    if not len(ego):
        return [], []
    segments, events = [], []
    first = 0
    while first < len(ego) and not np.all(np.isfinite(ego[first,:7])):
        events.append({"time":float(ego[first,0]) if np.isfinite(ego[first,0]) else None,
                       "previous_time":None,"reason":"nonfinite_initial_oracle",
                       "distance_m":None,"gap_sec":None})
        first += 1
    if first == len(ego):
        return [],events
    for i in range(first+1, len(ego)):
        before, after = ego[i-1], ego[i]
        dt = after[0] - before[0]
        ds = np.linalg.norm(after[1:3]-before[1:3])
        speed = max(abs(before[4]), abs(after[4]))
        reason = None
        if not np.all(np.isfinite(after[:7])):
            reason = "nonfinite_oracle"
        elif dt <= 0:
            reason = "duplicate_or_backward_header"
        elif dt > max_gap:
            reason = "ego_gap"
        elif ds > max(2.0, 3.0*speed*dt):
            reason = "teleport_or_physics_discontinuity"
        elif abs(float(wrapped(after[3]-before[3]))) > math.radians(45):
            reason = "heading_discontinuity"
        if reason:
            if i-first >= 2:
                segments.append(ego[first:i].copy())
            events.append({"time": float(after[0]) if np.isfinite(after[0]) else None,
                           "previous_time": float(before[0]) if np.isfinite(before[0]) else None,
                           "reason": reason, "distance_m": float(ds) if np.isfinite(ds) else None,
                           "gap_sec": float(dt) if np.isfinite(dt) else None})
            first = i+1 if reason=="nonfinite_oracle" else i
    if len(ego)-first >= 2:
        segments.append(ego[first:].copy())
    # Exclude settling after all starts, including a SceneLoad boundary. Regular
    # message gaps need only a bounded interpolation, but this conservative
    # margin avoids stale state during restart/physics preparation.
    trimmed = [seg[seg[:, 0] >= seg[0, 0]+settle] for seg in segments]
    return [seg for seg in trimmed if len(seg) >= 2], events


def within_window(times, start, end):
    values=np.asarray(times)
    finite=np.isfinite(values)
    clean=np.where(finite,values,0.)
    return finite & (clean>=start) & (clean<=end)


def interpolate_segments(segments, queries):
    queries = np.asarray(queries, dtype=float)
    width = segments[0].shape[1]-1 if segments else 7
    output = np.full((len(queries), width), np.nan)
    ids = np.full(len(queries), -1, dtype=int)
    seen = np.zeros(len(queries), dtype=bool)
    ambiguous = np.zeros(len(queries), dtype=bool)
    for n, seg in enumerate(segments):
        mask = (queries >= seg[0, 0]) & (queries <= seg[-1, 0])
        ambiguous |= mask & seen
        seen |= mask
        if not np.any(mask):
            continue
        for j in range(1, seg.shape[1]):
            values = np.unwrap(seg[:, j]) if j == 3 else seg[:, j]
            output[mask, j-1] = np.interp(queries[mask], seg[:, 0], values)
        ids[mask] = n
    output[ambiguous] = np.nan
    ids[ambiguous] = -2
    return output, ids


def gps_outages(gps, start, end, freshness=.5):
    """Zero/nonfinite raw position and receipt/header-stale periods, separately."""
    intervals = []
    open_at = None
    for row in gps:
        if not bool(row[1]) and open_at is None:
            open_at = row[0]
        elif bool(row[1]) and open_at is not None:
            if row[0] > open_at:
                intervals.append({"start": float(open_at), "end": float(row[0]), "reason": "raw_invalid"})
            open_at = None
    if open_at is not None:
        intervals.append({"start": float(open_at), "end": float(end), "reason": "raw_invalid_unrecovered"})
    for a, b in zip(gps[:-1], gps[1:]):
        if b[-1]-a[-1] > freshness:
            # Bag receipt gaps detect transport loss independently of reused headers.
            intervals.append({"start": float(a[-1]+freshness), "end": float(b[-1]), "reason": "receipt_stale"})
    stale_start = None
    for row in gps:
        stale = row[-1]-row[-2] > freshness
        if stale and stale_start is None:
            stale_start = row[-1]
        elif not stale and stale_start is not None:
            intervals.append({"start": float(stale_start), "end": float(row[-1]), "reason": "header_stale"})
            stale_start = None
    if stale_start is not None:
        intervals.append({"start": float(stale_start), "end": float(end), "reason": "header_stale_unrecovered"})
    intervals.sort(key=lambda item: item["start"])
    merged = []
    for item in intervals:
        if item["end"] <= item["start"]:
            continue
        if merged and item["start"] <= merged[-1]["end"]:
            merged[-1]["end"] = max(merged[-1]["end"], item["end"])
            merged[-1]["reasons"].append(item["reason"])
        else:
            merged.append({"start": item["start"], "end": item["end"], "reasons": [item["reason"]]})
    return merged


def phase_masks(times, outages, start, end):
    times = np.asarray(times)
    finite=np.isfinite(times)
    times=np.where(finite,times,np.inf)
    if not outages:
        return {"normal_no_observed_shadow": finite}
    masks = {"pre": times < outages[0]["start"], "outage": np.zeros(len(times), dtype=bool),
             "reacquisition_0_1s": np.zeros(len(times), dtype=bool),
             "post": np.zeros(len(times), dtype=bool)}
    for n, interval in enumerate(outages):
        a, b = interval["start"], interval["end"]
        masks["outage"] |= (times >= a) & (times < b)
        if b < end:
            next_start = outages[n+1]["start"] if n+1 < len(outages) else end
            masks["reacquisition_0_1s"] |= (times >= b) & (times < min(b+1, next_start))
            masks["post"] |= (times >= b+1) & (times < next_start)
    return masks


def pose_errors(state, oracle):
    dx, dy = state[:, 1]-oracle[:, 0], state[:, 2]-oracle[:, 1]
    heading = oracle[:, 2]
    return {"xy_m": np.hypot(dx, dy), "lateral_m": -dx*np.sin(heading)+dy*np.cos(heading),
            "longitudinal_m": dx*np.cos(heading)+dy*np.sin(heading),
            "yaw_deg": np.degrees(wrapped(state[:, 3]-heading))}


def summarize_phases(times, fields, masks, speed=None):
    answer = {}
    for phase, selected in masks.items():
        answer[phase] = {"all": {key: statistics(value[selected]) for key, value in fields.items()}}
        if speed is not None:
            clean_speed = np.nan_to_num(np.abs(speed), nan=-1.)
            moving = selected & (clean_speed > .5)
            stopped = selected & (clean_speed >= 0) & (clean_speed <= .05)
            answer[phase]["moving_gt_speed_gt_0p5_mps"] = {key: statistics(value[moving]) for key, value in fields.items()}
            answer[phase]["stopped_gt_speed_le_0p05_mps"] = {key: statistics(value[stopped]) for key, value in fields.items()}
    return answer


def nearest_bounded_path(xy, path, max_segment=2., max_overshoot=2.):
    """Signed normal error to a finite segment; never extend a trajectory line."""
    if len(path) < 2:
        return None
    delta = path[1:]-path[:-1]
    length = np.linalg.norm(delta, axis=1)
    valid = np.all(np.isfinite(path[:-1]), axis=1) & np.all(np.isfinite(path[1:]), axis=1) & (length > 1e-6) & (length <= max_segment)
    if not np.any(valid):
        return None
    a, d, length = path[:-1][valid], delta[valid], length[valid]
    raw = np.sum((xy-a)*d, axis=1)/(length**2)
    bounded = np.clip(raw, 0, 1)
    residual = xy-(a+bounded[:, None]*d)
    distance = np.linalg.norm(residual, axis=1)
    pick = int(np.argmin(distance))
    overshoot = abs(raw[pick]-bounded[pick])*length[pick]
    if overshoot > max_overshoot:
        return None
    signed = (d[pick, 0]*residual[pick, 1]-d[pick, 1]*residual[pick, 0])/length[pick]
    return signed, float(distance[pick]), float(overshoot)


def trajectory_error(ego, segment_ids, trajectories, max_age=.5):
    receipts = np.asarray([t["receipt"] for t in trajectories])
    selected = np.searchsorted(receipts, ego[:, -1], side="right")-1
    result = np.full((len(ego), 5), np.nan)
    rejected = {"no_past_trajectory": 0, "trajectory_stale": 0, "wrong_frame": 0, "unbounded_geometry": 0}
    for i, j in enumerate(selected):
        if segment_ids[i] < 0:
            continue
        if j < 0:
            rejected["no_past_trajectory"] += 1
            continue
        trajectory = trajectories[j]
        receipt_age = ego[i, -1]-trajectory["receipt"]
        age = ego[i, 0]-trajectory["time"]
        if not (0 <= receipt_age <= max_age) or abs(age) > max_age:
            rejected["trajectory_stale"] += 1
            continue
        if trajectory["frame"] != "map":
            rejected["wrong_frame"] += 1
            continue
        projection = nearest_bounded_path(ego[i, 1:3], trajectory["xy"])
        if projection is None:
            rejected["unbounded_geometry"] += 1
            continue
        result[i] = (*projection, receipt_age, age)
    return result, rejected


def recovery_summary(times, error, outages, threshold=.2, hold=1., max_gap=.1, metric_name="xy_m"):
    result = []
    for interval in outages:
        after = np.flatnonzero(times >= interval["end"])
        recovered = None
        for i in after:
            target = np.flatnonzero((times >= times[i]) & (times <= times[i]+hold))
            if len(target) < 2 or times[target[-1]]-times[i] < hold-.05:
                continue
            if np.max(np.diff(times[target])) > max_gap:
                continue
            if np.all(np.isfinite(error[target])) and np.all(error[target] <= threshold):
                recovered = float(times[i]-interval["end"])
                break
        result.append({"outage_end": interval["end"], "error_metric":metric_name,"threshold_m":threshold,
                       "continuous_hold_sec": hold, "recovery_after_reacquisition_sec": recovered})
    return result


def outage_evolution(times, fields, outages, usable):
    """First/last state and linear trend, with no interpolation over missing data."""
    answer = []
    for interval in outages:
        selected = (times >= interval["start"]) & (times < interval["end"]) & usable
        record = {"outage_start": interval["start"], "outage_end": interval["end"],
                  "samples": int(np.count_nonzero(selected)), "fields": {}}
        for name, values in fields.items():
            finite = selected & np.isfinite(values)
            t, v = times[finite], values[finite]
            if len(t) < 2:
                continue
            duration = t[-1]-t[0]
            if duration <= 0:
                continue
            centered = t-np.mean(t)
            denom = np.sum(centered**2)
            slope = np.sum(centered*(v-np.mean(v)))/denom if denom else 0.
            record["fields"][name] = {"first": float(v[0]), "last": float(v[-1]),
                "change": float(v[-1]-v[0]), "duration_sec": float(duration),
                "linear_slope_per_sec": float(slope)}
        answer.append(record)
    return answer


def execution_evidence(path, events_override=None):
    """Read recorded validator evidence, without inventing a success judgement."""
    evidence={}
    sidecar=path.with_suffix(".json")
    if sidecar.is_file():
        doc=json.loads(sidecar.read_text(encoding="utf-8"))
        evidence={"sidecar":str(sidecar),"status":doc.get("status"),
                  "drive_reason":doc.get("drive_reason"),"metrics":doc.get("metrics"),
                  "safety":doc.get("safety")}
    events=events_override or path.parent/"events.jsonl"
    if events_override is not None and not events.is_file():
        raise ValueError("Explicit validator events file is missing: "+str(events))
    if events.is_file():
        messages=[json.loads(line) for line in events.read_text(encoding="utf-8").splitlines() if line.strip()]
        evidence["events_source"]=str(events)
        for message in messages:
            if message.get("event")=="CONTROLLER_START":
                evidence["controller_start_stamp"]=datetime.fromisoformat(message["time"]).timestamp()
                break
        for message in messages:
            event=message.get("event")
            # Gracefully aborted drives have CASE_RESULT but no RUN_FINISHED.
            # Exclude brake/parking shutdown without turning an abort into PASS.
            if event not in ("RUN_FINISHED", "CASE_RESULT"):
                continue
            stamp=datetime.fromisoformat(message["time"]).timestamp()
            controller_start=evidence.get("controller_start_stamp")
            if event=="CASE_RESULT" and controller_start is None:
                continue
            if controller_start is not None and stamp<controller_start:
                continue
            evidence["run_finished_stamp"]=stamp
            evidence["evaluation_stop_event"]=event
            break
    return evidence


def analyze(path, args):
    bag = inspect_bag(path, args.time_axis)
    rows, start, end = bag["rows"], bag["start"], bag["end"]
    execution=execution_evidence(path,getattr(args,"events",None))
    start_override=getattr(args,"evaluation_start_sec",None)
    if start_override is not None:
        evaluation_start=start+start_override
        start_policy="explicit_bag_offset"
    elif execution.get("controller_start_stamp") is not None:
        evaluation_start=execution["controller_start_stamp"]
        start_policy="recorded_validator_CONTROLLER_START"
    else:
        evaluation_start=start
        start_policy="bag_start_no_validator_gate_available"
    evaluation_end=execution.get("run_finished_stamp",end)
    raw_times=rows["/Ego_topic"][:,0]
    if start_policy=="recorded_validator_CONTROLLER_START" and not (
        raw_times[0]-.5 <= evaluation_start <= raw_times[-1]+.5 and evaluation_start<evaluation_end):
        if getattr(args,"events",None) is not None:
            raise ValueError("Explicit events are outside raw Ego time extent; cannot reuse a different run's event clock.")
        execution["evaluation_window_error"]="Controller event clock does not overlap raw Ego clock; auto window disabled."
        evaluation_start,evaluation_end=start,end
        start_policy="bag_start_event_clock_unverified"
    ego = rows["/Ego_topic"].copy()
    if any(frame.lstrip("/") != "morai_map" for frame in bag["frames"].get("/Ego_topic", [])):
        raise ValueError("Raw Ego frame is not morai_map; do not silently apply map-from-morai TF.")
    tx, ty, theta = bag["transform"]
    x, y = ego[:, 1].copy(), ego[:, 2].copy()
    ego[:, 1] = tx+math.cos(theta)*x-math.sin(theta)*y
    ego[:, 2] = ty+math.sin(theta)*x+math.cos(theta)*y
    ego[:, 3] += math.radians(args.heading_offset_deg)+theta
    segments, exclusions = split_oracle(ego, args.oracle_max_gap_sec, args.settle_sec)
    input_intervals = gps_outages(rows["/sensor/gps"], start, end, args.gps_stale_sec)
    # The initial SceneLoad produces simultaneous raw GPS/Ego transport gaps.
    # Preserve them as input diagnostics, but do not count an interval wholly
    # outside usable post-load oracle coverage as an operational GPS shadow.
    outages = [o for o in input_intervals if any(
        o["end"] > seg[0, 0] and o["start"] < seg[-1, 0] for seg in segments)]
    excluded_gps_intervals = [o for o in input_intervals if o not in outages]
    states, values_for_recovery = {}, {}
    for name, topic in STATE_TOPICS.items():
        state = rows[topic]
        if not state.size:
            states[name] = {"available": False, "topic": topic}
            continue
        reference, ids = interpolate_segments(segments, state[:, 0])
        fields = pose_errors(state, reference) if name != "local_ekf" else {}
        fields.update({"vx_mps": state[:, 4], "vy_mps": state[:, 5], "yaw_rate_radps": state[:, 6], "vy_covariance": state[:, 7]})
        if name == "local_ekf":
            derivative = np.full(len(state), np.nan)
            dt = np.diff(state[:, 0])
            pairs = (dt > 1e-6) & (dt < .1) & (ids[:-1] >= 0) & (ids[:-1] == ids[1:])
            derivative[1:][pairs] = np.diff(state[:, 5])[pairs]/dt[pairs]
            fields["vy_time_derivative_mps2_not_body_ay"] = derivative
        usable = (ids >= 0) & within_window(state[:,0],evaluation_start,evaluation_end)
        masks = {k: v & usable for k, v in phase_masks(state[:, 0], outages, start, end).items()}
        states[name] = {"available": True, "topic": topic, "frame": bag["frames"].get(topic),
                       "oracle_matched_samples": int(np.count_nonzero(usable)),
                       "oracle_excluded_samples": int(np.count_nonzero(~usable)),
                       "outside_evaluation_window_samples":int(np.count_nonzero(~within_window(state[:,0],evaluation_start,evaluation_end))),
                       "oracle_unmatched_samples":int(np.count_nonzero(ids<0)),
                       "phases": summarize_phases(state[:, 0], fields, masks, reference[:, 3])}
        states[name]["overall_evaluated"]=summarize_phases(state[:,0],fields,{"run":usable},reference[:,3])["run"]
        evaluated=state[usable]
        states[name]["evaluated_timing"]=timing_summary(np.column_stack((evaluated[:,0],evaluated[:,-1]))) if len(evaluated) else {"count":0}
        states[name]["outage_evolution"] = outage_evolution(state[:, 0], fields, outages, usable)
        if name != "local_ekf":
            error = fields["xy_m"].copy()
            error[~usable] = np.nan
            states[name]["recovery"] = recovery_summary(state[:, 0], error, outages)
            lateral=np.abs(fields["lateral_m"].copy());lateral[~usable]=np.nan
            states[name]["lateral_recovery"] = recovery_summary(state[:,0],lateral,outages,.1,metric_name="lateral_m")
            values_for_recovery[name] = (state[:, 0], fields)
        else:
            states[name]["note"] = "Local EKF is odom-frame motion; absolute map pose error is intentionally not computed."
    _, ego_ids = interpolate_segments(segments, ego[:, 0])
    ego_ids[~within_window(ego[:,0],evaluation_start,evaluation_end)]=-3
    trajectory, rejects = trajectory_error(ego, ego_ids, bag["trajectories"], args.trajectory_max_age_sec)
    ego_masks = {k: v & (ego_ids >= 0) for k,v in phase_masks(ego[:, 0], outages, start, end).items()}
    actual_fields = {"lateral_to_recorded_trajectory_m": trajectory[:, 0],
                     "distance_to_bounded_trajectory_m": trajectory[:, 1],
                     "projection_overshoot_m": trajectory[:, 2],
                     "trajectory_receipt_age_sec": trajectory[:, 3],
                     "trajectory_header_age_sec": trajectory[:, 4],
                     "ego_vx_mps": ego[:, 4], "ego_vy_mps": ego[:, 5],
                     "ego_ay_mps2": ego[:, 6]}
    # Ego physical increments are separately reported; normal obstacle avoidance
    # belongs to the planned trajectory and is never an estimator drift metric.
    actual = {"note": "Independent Ego versus latest causally received, non-stale recorded trajectory; not lane centerline and not localization drift.",
              "trajectory_messages": len(bag["trajectories"]), "rejections": rejects,
              "phases": summarize_phases(ego[:, 0], actual_fields, ego_masks, ego[:, 4]),
              "overall_evaluated":summarize_phases(ego[:,0],actual_fields,{"run":ego_ids>=0},ego[:,4])["run"],
              "lateral_recovery":recovery_summary(ego[:,0],np.abs(trajectory[:,0]),outages,.2,metric_name="actual_trajectory_lateral_m")}
    inputs = {}
    for topic, field_map in (("/localization/acceleration", {"ax_mps2": 1,"ay_mps2": 2}),
                             ("/control/command/steering", {"steering_rad": 1}),
                             ("/ctrl_cmd", {"steering_rad": 1, "accel": 2,"brake": 3}),
                             ("/purepursuit_autoware/debug/lateral_error", {"controller_lateral_debug_m": 1})):
        arr = rows[topic]
        if not arr.size:
            inputs[topic] = {"available": False}
            continue
        ref, ids = interpolate_segments(segments, arr[:, 0])
        fields = {key: arr[:, col] for key,col in field_map.items()}
        if "steering_rad" in field_map:
            dt = np.diff(arr[:, 0]); rate = np.full(len(arr), np.nan)
            adjacent = (dt>0) & (dt<.1) & (ids[:-1]>=0) & (ids[1:]==ids[:-1])
            rate[1:][adjacent] = np.diff(arr[:, field_map["steering_rad"]])[adjacent]/dt[adjacent]
            fields["steering_rate_radps"] = rate
        usable=(ids>=0)&within_window(arr[:,0],evaluation_start,evaluation_end)
        masks = {k: v & usable for k,v in phase_masks(arr[:, 0], outages, start, end).items()}
        inputs[topic] = {"available": True, "frame": bag["frames"].get(topic),
                         "phases": summarize_phases(arr[:, 0], fields, masks, ref[:, 3]),
                         "overall_evaluated":summarize_phases(arr[:,0],fields,{"run":usable},ref[:,3])["run"],
                         "outage_evolution": outage_evolution(arr[:, 0], fields, outages, usable)}
        if "steering_rad" in field_map:
            sign_values = arr[:, field_map["steering_rad"]]
            for phase, mask in masks.items():
                active = mask & (np.abs(sign_values) > .01) & (np.abs(np.nan_to_num(ref[:,3])) > .5)
                index = np.flatnonzero(active)
                if len(index) < 2:
                    inputs[topic]["phases"][phase]["steering_sign_flip_count"] = 0
                    continue
                signs = np.sign(sign_values[index])
                adjacent = (arr[index[1:],0]-arr[index[:-1],0] < .5) & (ids[index[1:]]==ids[index[:-1]])
                inputs[topic]["phases"][phase]["steering_sign_flip_count"] = int(np.count_nonzero((signs[1:]*signs[:-1] < 0) & adjacent))
    # Oracle motion/header residual is not mislabeled as estimator frame jump.
    residuals = []
    for segment in segments:
        dt = np.diff(segment[:, 0]); unwrapped = np.unwrap(segment[:,3]); h = (unwrapped[:-1]+unwrapped[1:])/2
        speed = (segment[:-1,4]+segment[1:,4])/2
        delta = np.diff(segment[:,1:3],axis=0)
        ex = delta[:,0]-dt*speed*np.cos(h); ey=delta[:,1]-dt*speed*np.sin(h)
        moving = np.abs(speed)>.5
        residuals.extend(np.column_stack((-ex*np.sin(h)+ey*np.cos(h), ex*np.cos(h)+ey*np.sin(h)))[moving])
    residuals = np.asarray(residuals)
    dt = np.diff(ego[:,0])
    pair_valid = (ego_ids[:-1]>=0) & (ego_ids[:-1]==ego_ids[1:]) & (dt>0) & (dt<.1)
    mid_time = (ego[:-1,0]+ego[1:,0])/2
    physical_distance = np.linalg.norm(np.diff(ego[:,1:3],axis=0),axis=1)
    body_speed = np.hypot(ego[:,4],ego[:,5])
    velocity_distance = (body_speed[:-1]+body_speed[1:])/2*dt
    phase_motion = {}
    for phase, mask in phase_masks(mid_time,outages,start,end).items():
        valid = mask & pair_valid
        travelled, integrated = float(np.sum(physical_distance[valid])), float(np.sum(velocity_distance[valid]))
        phase_motion[phase] = {"consecutive_pair_count":int(np.count_nonzero(valid)),
            "covered_time_sec":float(np.sum(dt[valid])), "physical_xy_distance_m":travelled,
            "raw_ego_speed_time_integral_m":integrated,
            "physical_distance_over_speed_time_integral":travelled/integrated if integrated>1e-3 else None,
            "note":"An empirical physics/time-axis consistency ratio, not a velocity scale to apply to production."}
    tf_closure = {"available": False}
    if bag["tf"].size and rows[STATE_TOPICS["local_ekf"]].size and rows[STATE_TOPICS["global_state"]].size:
        tf = bag["tf"]
        local_segments, _ = split_oracle(rows[STATE_TOPICS["local_ekf"]], args.oracle_max_gap_sec, 0.)
        global_segments, _ = split_oracle(rows[STATE_TOPICS["global_state"]], args.oracle_max_gap_sec, 0.)
        local, li = interpolate_segments(local_segments, tf[:,0])
        global_pose, gi = interpolate_segments(global_segments, tf[:,0])
        _, oi = interpolate_segments(segments, tf[:,0])
        c, s = np.cos(tf[:,3]), np.sin(tf[:,3])
        compose_x=tf[:,1]+c*local[:,0]-s*local[:,1]
        compose_y=tf[:,2]+s*local[:,0]+c*local[:,1]
        fields={"xy_closure_m":np.hypot(compose_x-global_pose[:,0],compose_y-global_pose[:,1]),
                "yaw_closure_deg":np.degrees(wrapped(tf[:,3]+local[:,2]-global_pose[:,2]))}
        usable = (li>=0)&(gi>=0)&(oi>=0)&within_window(tf[:,0],evaluation_start,evaluation_end)
        masks={k:v&usable for k,v in phase_masks(tf[:,0],outages,start,end).items()}
        tf_closure={"available":True,"matched_samples":int(np.count_nonzero(usable)),
                    "phases":summarize_phases(tf[:,0],fields,masks),
                    "note":"Recorded map->odom composed with co-time local odometry versus recorded global state. Not independent GT."}
    result = {"bag": str(path.resolve()), "duration_sec": end-start,
        "configuration": {"time_axis": args.time_axis, "ego_heading_offset_deg": args.heading_offset_deg,
                          "map_from_morai": {"x":tx,"y":ty,"yaw_rad":theta},
                          "oracle_max_gap_sec":args.oracle_max_gap_sec,"settle_exclusion_sec":args.settle_sec,
                          "evaluation_start_stamp":evaluation_start,"evaluation_start_bag_offset_sec":evaluation_start-start,
                          "evaluation_end_stamp":evaluation_end,"evaluation_end_bag_offset_sec":evaluation_end-start,
                          "evaluation_start_policy":start_policy,
                          "evaluation_end_policy":execution.get("evaluation_stop_event","bag_end_no_terminal_event"),
                          "gps_stale_sec":args.gps_stale_sec,"trajectory_max_age_sec":args.trajectory_max_age_sec},
        "topic_message_counts":bag["counts"], "topic_frames":bag["frames"],"execution_evidence":execution,
        "recorded_object_footprints":bag["object_content"],
        "timing":{k:timing_summary(v) for k,v in bag["timing"].items()},
        "missing_or_zero_header_fallback_counts":bag["header_fallback"],
        "raw_gps_quality":{
            "message_count":int(len(rows['/sensor/gps'])),
            "invalid_coordinate_count":int(np.count_nonzero(rows['/sensor/gps'][:,1]==0)),
            "zero_zero_count":int(np.count_nonzero((rows['/sensor/gps'][:,2]==0)&(rows['/sensor/gps'][:,3]==0))),
            "nonfinite_coordinate_count":int(np.count_nonzero(~np.all(np.isfinite(rows['/sensor/gps'][:,2:4]),axis=1))),
            "status_histogram":{str(int(status)):int(np.count_nonzero(rows['/sensor/gps'][:,4]==status)) for status in np.unique(rows['/sensor/gps'][:,4])}},
        "oracle_exclusions":[dict(item,time_bag_offset_sec=item["time"]-start if item["time"] is not None else None) for item in exclusions],
        "oracle_segment_count":len(segments),
        "gps_outages":[dict(o,duration_sec=o["end"]-o["start"],start_bag_offset_sec=o["start"]-start,end_bag_offset_sec=o["end"]-start) for o in outages],
        "gps_intervals_excluded_outside_oracle_coverage":[dict(o,start_bag_offset_sec=o["start"]-start,end_bag_offset_sec=o["end"]-start) for o in excluded_gps_intervals],
        "gps_status_does_not_override_position_validity":True,
        "states":states,"actual_ego":actual,"motion_and_control":inputs,"tf_closure":tf_closure,
        "oracle_motion_by_phase":phase_motion,
        "oracle_kinematic_residual":{ "lateral_m":statistics(residuals[:,0]) if residuals.size else {"count":0},
                                      "longitudinal_m":statistics(residuals[:,1]) if residuals.size else {"count":0}},
        "limitations":["A single drive is not a repeated controlled trial.",
            "Independent raw Ego positions are not a competition input/fallback.",
            "Header interpolation cannot remove simulator pose-update cadence artifacts.",
            "Trajectory error uses recorded active geometry, not intended lane center or future avoidance correctness.",
            "Global acceleration topic does not expose Local EKF's hidden ay state; Local ay requires debug state or differentiated vy with model caveats.",
            "Phase onset is bounded by GPS sample cadence; receipt-stale windows use receipt times and are explicitly labeled."]}
    return result


def write_markdown(result, output):
    rows = ["# GPS shadow validator bag analysis", "", f"Bag: `{result['bag']}`", "",
            f"Independent Ego yaw offset: {result['configuration']['ego_heading_offset_deg']} deg; time axis: {result['configuration']['time_axis']}.", "",
            f"Evaluation window: {result['configuration']['evaluation_start_bag_offset_sec']:.3f}–{result['configuration']['evaluation_end_bag_offset_sec']:.3f}s ({result['configuration']['evaluation_start_policy']}). Input timing/count diagnostics retain the full bag.", "",
            "| State / phase (all matched samples) | XY mean / p50 / p95 (m) | Lateral mean absolute / p95 (m) | Longitudinal mean absolute / p95 (m) | Yaw absolute p95 (deg) |",
            "|---|---:|---:|---:|---:|"]
    def value(metric,key):
        return f"{metric['absolute'][key]:.4f}" if metric.get("count") else "NA"
    for name, state in result["states"].items():
        if not state.get("available") or name=="local_ekf": continue
        for phase, metric in state["phases"].items():
            m=metric["all"]; xy=m["xy_m"]; lat=m["lateral_m"]; lon=m["longitudinal_m"]
            rows.append(f"| {name} / {phase} | {value(xy,'mean')} / {value(xy,'p50')} / {value(xy,'p95')} | {value(lat,'mean')} / {value(lat,'p95')} | {value(lon,'mean')} / {value(lon,'p95')} | {value(m['yaw_deg'],'p95')} |")
    rows += ["", "| Actual Ego / phase | Recorded trajectory lateral mean absolute / p50 / p95 / max (m) | Samples |",
             "|---|---:|---:|"]
    for phase, data in result["actual_ego"]["phases"].items():
        m=data["all"]["lateral_to_recorded_trajectory_m"]
        rows.append(f"| {phase} | {' / '.join(value(m,k) for k in ('mean','p50','p95','max'))} | {m['count']} |")
    rows += ["", "GPS intervals:", ""]
    for o in result["gps_outages"]:
        rows.append(f"- {o['start_bag_offset_sec']:.3f}–{o['end_bag_offset_sec']:.3f}s ({o['duration_sec']:.3f}s): {', '.join(o['reasons'])}")
    rows += ["", "Oracle segmentation/exclusions:", ""]
    for o in result["oracle_exclusions"]:
        offset=f"{o['time_bag_offset_sec']:.3f}s" if o['time_bag_offset_sec'] is not None else "unknown stamp"
        if o['distance_m'] is None:
            rows.append(f"- {offset}: {o['reason']}.")
        else:
            rows.append(f"- {offset}: {o['reason']}, displacement {o['distance_m']:.3f}m, gap {o['gap_sec']:.3f}s.")
    rows += ["", "Limits:", ""]+[f"- {v}" for v in result["limitations"]]
    output.write_text("\n".join(rows)+"\n",encoding="utf-8")


def plot_bag(path, args, output):
    """Small causal diagnostics figure; signed errors preserve drift direction."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    bag=inspect_bag(path,args.time_axis)
    ego=bag["rows"]["/Ego_topic"].copy()
    tx,ty,theta=bag["transform"]
    x,y=ego[:,1].copy(),ego[:,2].copy()
    ego[:,1]=tx+math.cos(theta)*x-math.sin(theta)*y
    ego[:,2]=ty+math.sin(theta)*x+math.cos(theta)*y
    ego[:,3]+=math.radians(args.heading_offset_deg)+theta
    segments,_=split_oracle(ego,args.oracle_max_gap_sec,args.settle_sec)
    receipt=execution_evidence(path,getattr(args,"events",None))
    evaluation_start=bag['start']+args.evaluation_start_sec if getattr(args,"evaluation_start_sec",None) is not None else receipt.get("controller_start_stamp",bag["start"])
    evaluation_end=receipt.get("run_finished_stamp",bag["end"])
    outages=[o for o in gps_outages(bag["rows"]["/sensor/gps"],bag["start"],bag["end"],args.gps_stale_sec)
             if any(o["end"]>seg[0,0] and o["start"]<seg[-1,0] for seg in segments)]
    start=bag["start"]
    fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True)
    for name in ("global_state","smooth_state"):
        arr=bag["rows"][STATE_TOPICS[name]]
        if not arr.size: continue
        ref,ids=interpolate_segments(segments,arr[:,0])
        err=pose_errors(arr,ref)["lateral_m"];err[(ids<0)|~within_window(arr[:,0],evaluation_start,evaluation_end)]=np.nan
        axes[0].plot(arr[:,0]-start,err,label=name,linewidth=1.3)
    _,ids=interpolate_segments(segments,ego[:,0])
    ids[~within_window(ego[:,0],evaluation_start,evaluation_end)]=-3
    path_error,_=trajectory_error(ego,ids,bag["trajectories"],args.trajectory_max_age_sec)
    axes[1].plot(ego[:,0]-start,path_error[:,0],color="tab:green",label="actual Ego to active trajectory")
    for name in ("global_ekf","local_ekf"):
        arr=bag["rows"][STATE_TOPICS[name]]
        if arr.size:
            values=arr[:,5].copy();values[~within_window(arr[:,0],evaluation_start,evaluation_end)]=np.nan
            axes[2].plot(arr[:,0]-start,values,label=name+" vy",linewidth=1.2)
    acc=bag["rows"]["/localization/acceleration"]
    if acc.size:
        values=acc[:,2].copy();values[~within_window(acc[:,0],evaluation_start,evaluation_end)]=np.nan
        axes[3].plot(acc[:,0]-start,values,label="published Global ay",color="tab:red")
    for n,ax in enumerate(axes):
        ax.axhline(0,color="black",linewidth=.5,alpha=.4)
        for o in outages:
            ax.axvspan(o["start"]-start,o["end"]-start,color="gray",alpha=.18,label="raw GPS invalid/stale" if n==0 else None)
            if o["end"]<bag["end"]:
                ax.axvspan(o["end"]-start,o["end"]-start+1,color="gold",alpha=.16,label="reacquisition 0-1s" if n==0 else None)
        ax.grid(alpha=.2)
        handles,_=ax.get_legend_handles_labels()
        if handles: ax.legend(loc="best",fontsize=8)
    axes[0].set_ylabel("Signed localization\nlateral error (m)")
    axes[1].set_ylabel("Signed actual path\nlateral error (m)")
    axes[2].set_ylabel("Estimated body vy\n(m/s)")
    axes[3].set_ylabel("Estimated body ay\n(m/s²)")
    axes[3].set_xlabel("Bag offset (s)")
    if outages:
        axes[3].set_xlim(max(0,outages[0]["start"]-start-6),min(bag["end"]-start,outages[-1]["end"]-start+7))
    fig.suptitle(path.name+f"\nRaw Ego oracle: heading {args.heading_offset_deg}° once; gray = GPS invalid/stale",fontsize=10)
    fig.tight_layout()
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(str(output),dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--events",type=Path,
                        help="Original case events.jsonl for offline replay; ISO times must match the recorded header epoch.")
    parser.add_argument("--heading-offset-deg", type=float, default=-1.347)
    parser.add_argument("--time-axis", choices=("header","receipt"),default="header")
    parser.add_argument("--oracle-max-gap-sec", type=float, default=.25)
    parser.add_argument("--settle-sec",type=float,default=.5)
    parser.add_argument("--gps-stale-sec",type=float,default=.5)
    parser.add_argument("--trajectory-max-age-sec",type=float,default=.5)
    parser.add_argument("--evaluation-start-sec",type=float,
                        help="Explicit bag-offset evaluation start. Default: recorded CONTROLLER_START if available; otherwise bag start.")
    args=parser.parse_args()
    result=analyze(args.bag,args)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    if args.markdown:
        args.markdown.parent.mkdir(parents=True,exist_ok=True)
        write_markdown(result,args.markdown)
    if args.plot:
        plot_bag(args.bag,args,args.plot)
    print(json.dumps({"output":str(args.output),"outages":result["gps_outages"],
                      "oracle_segment_count":result["oracle_segment_count"]},indent=2))


if __name__ == "__main__":
    main()
