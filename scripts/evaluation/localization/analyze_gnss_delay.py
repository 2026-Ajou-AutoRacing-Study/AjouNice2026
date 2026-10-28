#!/usr/bin/env python3
"""Estimate an *absolute raw-GNSS* advance from recorded MORAI topics.

Positive delay means GPS(t) resembles the offline Ego oracle at t-delay. This is
an apparent relative observation epoch, NOT measured physical/network latency.
No ROS node, parameter, timestamp, bag, or live process is changed. In particular
already compensated /localization/gnss/odom is never used as the primary input.
Only numpy is needed for numerical tests; rosbag is imported by the bag reader.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

TOPICS = {
    "ego": "/Ego_topic", "competition": "/Competition_topic", "gps": "/sensor/gps",
    "twist": "/localization/gt/twist", "orientation": "/localization/gt/orientation",
}
SCHEMA_VERSION = 1
MAX_LAG_MS = 100


class InsufficientEvidence(ValueError):
    """Input can be read, but it does not support a calibration recommendation."""


def wgs84_to_utm(latitude_deg, longitude_deg, zone=52, northern=True):
    """Same WGS84 series as morai_gps_to_local_odometry (horizontal only).

    Production subtracts only GPS *height*; it does not subtract an XY lever arm.
    We intentionally reproduce that XY convention, not invent a calibration.
    """
    a, inv_f, scale = 6378137.0, 298.257223563, 0.9996
    f = 1.0 / inv_f
    e2 = f * (2.0 - f)
    ep2 = e2 / (1.0 - e2)
    lat, lon = math.radians(latitude_deg), math.radians(longitude_deg)
    central = math.radians((zone - 1) * 6 - 180 + 3)
    s, c, t = math.sin(lat), math.cos(lat), math.tan(lat)
    n = a / math.sqrt(1.0 - e2 * s * s)
    tt, cc, aa = t * t, ep2 * c * c, c * (lon - central)
    m = a * ((1-e2/4-3*e2**2/64-5*e2**3/256)*lat
             - (3*e2/8+3*e2**2/32+45*e2**3/1024)*math.sin(2*lat)
             + (15*e2**2/256+45*e2**3/1024)*math.sin(4*lat)
             - 35*e2**3/3072*math.sin(6*lat))
    east = scale*n*(aa+(1-tt+cc)*aa**3/6
                    +(5-18*tt+tt*tt+72*cc-58*ep2)*aa**5/120)+500000
    north = scale*(m+n*t*(aa*aa/2+(5-tt+9*cc+4*cc*cc)*aa**4/24
                         +(61-58*tt+tt*tt+600*cc-330*ep2)*aa**6/720))
    return east, north if northern else north + 10000000


def yaw_of(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-9:
        raise InsufficientEvidence("Recorded transform/orientation has invalid quaternion")
    x, y, z, w = values / norm
    # The map/morai_map bridge is planar. Silently discarding pitch is unsafe.
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)), roll, pitch


def clean_series(rows, width):
    """Stable time sort, finite rows, first occurrence per header; disclose counts."""
    a = np.asarray(rows, dtype=float).reshape(-1, width)
    counts = {"original": int(len(a)), "nonfinite": 0, "duplicates": 0, "backwards": 0,
              "large_backwards": 0}
    if not len(a):
        return a, counts
    counts["backwards"] = int(np.count_nonzero(np.diff(a[:, 0]) < 0))
    counts["large_backwards"] = int(np.count_nonzero(np.diff(a[:, 0]) < -.5))
    finite = np.isfinite(a).all(axis=1)
    counts["nonfinite"] = int(np.count_nonzero(~finite))
    a = a[finite]
    a = a[np.argsort(a[:, 0], kind="stable")]
    keep = np.r_[True, np.diff(a[:, 0]) > 1e-9] if len(a) else np.array([], dtype=bool)
    counts["duplicates"] = int(np.count_nonzero(~keep))
    return a[keep], counts


def transform_xy(xy, transform):
    tx, ty, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(xy) @ np.array([[c, s], [-s, c]]) + [tx, ty]


def canonical_motion_row(msg, key, relative_time):
    """Keep exact stamp components and production motion validity for diagnostics.

    Columns: relative seconds, vx/map-yaw, ROS stamp secs, nsecs, validity flag.
    Invalid messages must remain in the series: the live callback would replace
    its latest cached message too, rather than retain an older valid message.
    """
    stamp = msg.header.stamp
    seconds, nanoseconds = int(stamp.secs), int(stamp.nsecs)
    value, valid = 0., False
    try:
        frame_ok = msg.header.frame_id.lstrip("/") == "base_link"
        if key == "twist":
            value = float(msg.twist.twist.linear.x)
            valid = frame_ok and math.isfinite(value)
        else:
            q = msg.orientation
            x, y, z, w = (float(v) for v in (q.x, q.y, q.z, q.w))
            norm = math.hypot(x, y, z, w)
            valid = (frame_ok and msg.orientation_covariance[0] >= 0.
                     and all(math.isfinite(v) for v in (x, y, z, w))
                     and math.isfinite(norm) and abs(norm-1.) <= 1e-3)
            if valid:
                x, y, z, w = (v/norm for v in (x, y, z, w))
                value = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        valid = False
    return [relative_time, value if valid else 0., seconds, nanoseconds, float(valid)]


def read_bag(path, heading_offset_deg=-1.347):
    """Return [time,x,y,yaw,speed] oracle and [time,x,y] uncompensated raw GPS."""
    import rosbag  # ROS is deliberately optional for pure synthetic tests.
    rows = {k: [] for k in TOPICS}
    gps_geodetic, transforms, notices = [], [], []
    reverse = {v: k for k, v in TOPICS.items()}
    zero_invalid = 0
    zero_stamps = 0
    configured_ms = []
    with rosbag.Bag(str(path)) as bag:
        epoch = bag.get_start_time()
        for topic, msg, receipt in bag.read_messages(
                topics=list(reverse) + ["/tf_static", "/tf", "/rosout", "/rosout_agg"]):
            if topic in ("/rosout", "/rosout_agg"):
                # Context only: raw GPS reconstruction is independent of this value.
                if "GNSS advance " in msg.msg:
                    try:
                        configured_ms.append(float(msg.msg.split("GNSS advance ", 1)[1].split("ms", 1)[0]))
                    except ValueError:
                        pass
                continue
            if topic in ("/tf_static", "/tf"):
                for tf in msg.transforms:
                    parent, child = tf.header.frame_id.strip("/"), tf.child_frame_id.strip("/")
                    if {parent, child} != {"map", "morai_map"}:
                        continue
                    yaw, roll, pitch = yaw_of(tf.transform.rotation)
                    if max(abs(roll), abs(pitch)) > 1e-4:
                        raise InsufficientEvidence("Nonplanar map/morai_map TF is not supported")
                    xy = np.array([tf.transform.translation.x, tf.transform.translation.y])
                    if parent == "morai_map":
                        xy = -transform_xy(xy, [0, 0, -yaw])
                        yaw = -yaw
                    transforms.append([float(xy[0]), float(xy[1]), yaw])
                continue
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0 or not math.isfinite(stamp):
                zero_stamps += 1
                continue
            t = stamp - epoch
            key = reverse[topic]
            if key in ("ego", "competition"):
                rows[key].append([t, msg.position.x, msg.position.y,
                                  math.radians(msg.heading + heading_offset_deg), msg.velocity.x])
            elif key == "gps":
                lat, lon, status = float(msg.latitude), float(msg.longitude), int(msg.status)
                if (not math.isfinite(lat) or not math.isfinite(lon) or
                        abs(lat) > 90 or abs(lon) > 180 or status <= 0 or
                        (abs(lat) <= 1e-9 and abs(lon) <= 1e-9)):
                    zero_invalid += 1
                    continue
                east, north = wgs84_to_utm(lat, lon)
                # Same finite/nonzero message-offset fallback as production.
                offsets = [float(msg.eastOffset), float(msg.northOffset)]
                offsets = [v if math.isfinite(v) and abs(v) > 1.0 else fallback
                           for v, fallback in zip(offsets, [302595., 4124145.])]
                rows[key].append([t, east-offsets[0], north-offsets[1]])
                gps_geodetic.append([lat, lon])
            elif key == "twist":
                rows[key].append(canonical_motion_row(msg, key, t))
            elif key == "orientation":
                rows[key].append(canonical_motion_row(msg, key, t))
    if not transforms:
        raise InsufficientEvidence("Missing recorded map/morai_map TF; cannot align raw GPS/Ego")
    transform = transforms[0]
    if not np.allclose(transforms, transform, rtol=0, atol=1e-7):
        raise InsufficientEvidence("Recorded map/morai_map TF changed; split the recording")
    cleaned, counts = {}, {}
    for key, width in (("ego", 5), ("competition", 5), ("gps", 3), ("twist", 5), ("orientation", 5)):
        cleaned[key], counts[key] = clean_series(rows[key], width)
        if key in ("ego", "gps") and counts[key]["large_backwards"]:
            raise InsufficientEvidence("{} header jumped backwards by >0.5s; split clock epochs before analysis".format(TOPICS[key]))
    for key in ("ego", "competition", "gps"):
        a = cleaned[key]
        if len(a):
            a[:, 1:3] = transform_xy(a[:, 1:3], transform)
            if key != "gps":
                a[:, 3] += transform[2]
    # Preserve backwards-stamp diagnostics; sorting is not a claim of clock repair.
    if zero_stamps:
        notices.append("Rejected {} zero/nonfinite header stamps".format(zero_stamps))
    provenance = {"bag": str(Path(path).resolve()), "epoch": epoch,
                  "heading_offset_deg_applied_once_to_raw_status": heading_offset_deg,
                  "transform_map_from_morai_map": transform,
                  "raw_xy_conversion": "WGS84 UTM52N minus message offsets (production finite/nonzero fallback); recorded map TF",
                  "xy_lever_arm": "none, same as production converter; GPS height does not change XY",
                  "invalid_zero_or_status_gps_samples": zero_invalid,
                  "series_cleanup": counts, "recorded_configured_advance_ms": sorted(set(configured_ms)),
                  "notices": notices}
    return cleaned, provenance


def oracle_at(ego, query, max_gap=.15):
    """Bounded interpolation, no bridging gaps/teleports; yaw unwrapped once."""
    query = np.asarray(query, dtype=float)
    if len(ego) < 2:
        return np.full((len(query), 4), np.nan), np.zeros(len(query), dtype=bool)
    idx = np.clip(np.searchsorted(ego[:, 0], query), 1, len(ego)-1)
    dt = ego[idx, 0]-ego[idx-1, 0]
    travel = np.linalg.norm(ego[idx, 1:3]-ego[idx-1, 1:3], axis=1)
    good = ((query >= ego[0, 0]) & (query <= ego[-1, 0]) & (dt > 0) & (dt <= max_gap)
            & (travel <= np.maximum(2., dt*np.maximum(np.abs(ego[idx, 4]), np.abs(ego[idx-1, 4]))*2+.5)))
    result = np.column_stack([np.interp(query, ego[:, 0], ego[:, k]) for k in (1, 2, 3, 4)])
    result[:, 2] = np.interp(query, ego[:, 0], np.unwrap(ego[:, 3]))
    return result, good


def robust_fit(speed, along):
    x = np.column_stack([np.ones(len(speed)), -np.asarray(speed)])
    beta = np.linalg.lstsq(x, along, rcond=None)[0]
    for _ in range(25):
        residual = along-x@beta
        scale = max(.002, 1.4826*np.median(np.abs(residual-np.median(residual))))
        weight = np.minimum(1., 1.5*scale/np.maximum(np.abs(residual), 1e-12))
        beta = np.linalg.lstsq(x*np.sqrt(weight[:, None]), along*np.sqrt(weight), rcond=None)[0]
    return beta  # [fixed spatial bias metres, apparent lag seconds]


def metrics(error, direction):
    along = np.sum(error*direction, axis=1)
    cross = np.sum(error*np.column_stack([-direction[:, 1], direction[:, 0]]), axis=1)
    return {"along_mae_m": float(np.mean(np.abs(along))),
            "along_median_m": float(np.median(along)),
            "along_abs_p95_m": float(np.percentile(np.abs(along), 95)),
            "cross_abs_p95_m": float(np.percentile(np.abs(cross), 95)),
            "xy_mean_m": float(np.mean(np.linalg.norm(error, axis=1)))}


def duration_of_samples(times, max_gap=.15):
    delta = np.diff(times)
    return float(np.sum(delta[(delta > 0) & (delta <= max_gap)]))


def input_motion_consistency(ego):
    """Check native speed against position displacement, not delay/receipt age.

    A clock-correct-looking packet can still contain position and velocity whose
    wall-time motion is inconsistent under simulator overload. This cannot be
    repaired by a single fixed GNSS latency. Use only mostly straight 2s windows.
    """
    ratios = []
    if len(ego) < 2:
        return {"window_count": 0, "position_to_native_motion_ratio_median": None}
    for lo in np.arange(ego[0, 0], ego[-1, 0]-2., 2.):
        grid = np.linspace(lo, lo+2., 101)
        values, good = oracle_at(ego, grid)
        if not np.all(good) or np.ptp(np.unwrap(values[:, 2])) > .2 or np.min(values[:, 3]) <= 2.:
            continue
        velocity = values[:, 3, None]*np.column_stack([np.cos(values[:, 2]), np.sin(values[:, 2])])
        integrated = np.array([np.trapz(velocity[:, k], grid) for k in range(2)])
        norm = np.linalg.norm(integrated)
        if norm >= 5.:
            ratios.append(float(np.dot(values[-1, :2]-values[0, :2], integrated)/norm**2))
    return {"window_count": len(ratios),
            "position_to_native_motion_ratio_median": float(np.median(ratios)) if ratios else None,
            "ratio_p10_p90": np.percentile(ratios, [10, 90]).tolist() if ratios else None}


def paired_samples(data):
    """One fixed sample set for every -100..100ms lag and compensation comparison."""
    ego, gps = data["ego"], data["gps"]
    if len(ego) < 2 or len(gps) < 2:
        raise InsufficientEvidence("Need raw /sensor/gps AND offline /Ego_topic; filtered odometry is not a substitute")
    target, valid = oracle_at(ego, gps[:, 0])
    # Reject intervals spanning any discontinuity for every search candidate.
    for lag in range(-MAX_LAG_MS, MAX_LAG_MS+1, 5):
        _, good = oracle_at(ego, gps[:, 0]-lag/1000.)
        valid &= good
    valid &= (target[:, 3] > 2.)
    # Raw GPS teleports and offsets changing are not latency.
    step_t = np.diff(gps[:, 0])
    step_d = np.linalg.norm(np.diff(gps[:, 1:3], axis=0), axis=1)
    bad = (step_t <= .2) & (step_d > np.maximum(2., step_t*np.maximum(target[1:, 3], target[:-1, 3])*2+.5))
    valid[np.flatnonzero(bad)] = False
    valid[np.flatnonzero(bad)+1] = False
    return gps[valid], target[valid], {"gps_samples": len(gps), "rejected_pair_samples": int(np.count_nonzero(~valid))}


def motion_proxy(data, times, oracle):
    """Header-time diagnostic only: NOT ROS callback-order/actual applied rate.

    If canonical messages were recorded, mismatched/invalid latest pairs stay
    unadvanced. Never replace a failed canonical check with an oracle proxy.
    Exact ROS secs+nsecs equality is required, as in the production converter.
    """
    twist, orientation = data.get("twist", np.empty((0, 5))), data.get("orientation", np.empty((0, 5)))
    if len(twist) or len(orientation):
        vel = np.zeros((len(times), 2))
        good = np.zeros(len(times), dtype=bool)
        if len(twist) and len(orientation) and twist.shape[1] >= 5 and orientation.shape[1] >= 5:
            i = np.clip(np.searchsorted(twist[:, 0], times, side="right")-1, 0, len(twist)-1)
            j = np.clip(np.searchsorted(orientation[:, 0], times, side="right")-1, 0, len(orientation)-1)
            good = ((times-twist[i, 0] >= 0) & (times-twist[i, 0] <= .1)
                    & (times-orientation[j, 0] >= 0) & (times-orientation[j, 0] <= .1)
                    & (twist[i, 2] == orientation[j, 2])
                    & (twist[i, 3] == orientation[j, 3])
                    & (twist[i, 4] == 1.) & (orientation[j, 4] == 1.))
            vel[good] = twist[i[good], 1, None]*np.column_stack(
                [np.cos(orientation[j[good], 1]), np.sin(orientation[j[good], 1])])
        return vel, ("canonical header-time diagnostic; exact stamp/frame/covariance/quaternion checks; "
                     "invalid pairs unadvanced; NOT callback replay or actual applied rate"), int(np.count_nonzero(good))
    comp = data.get("competition", np.empty((0, 5)))
    if len(comp) > 1:
        values, good = oracle_at(comp, times)
        if np.all(good):
            return values[:, 3, None]*np.column_stack([np.cos(values[:, 2]), np.sin(values[:, 2])]), "Competition heading/vx interpolated diagnostic proxy (not callback replay)", None
    return oracle[:, 3, None]*np.column_stack([np.cos(oracle[:, 2]), np.sin(oracle[:, 2])]), "offline Ego motion diagnostic proxy; NOT independently measured compensation execution", None


def evaluate(data, min_moving_duration_sec=20., provenance=None):
    result = {"schema_version": SCHEMA_VERSION, "status": "insufficient_evidence", "recommendation_ms": None,
              "recommendation_scope": "absolute_raw_gnss_advance", "reasons": [],
              "physical_network_latency_measured": False, "settings_applied": False,
              "interpretation": "Apparent raw-GNSS vs offline Ego relative epoch; not physical latency, not full EKF/closed-loop certification",
              "provenance": provenance or {}}
    try:
        gps, target, cleanup = paired_samples(data)
    except (KeyError, InsufficientEvidence) as exc:
        result["reasons"].append(str(exc))
        return result
    count = len(gps)
    result["evidence"] = {"moving_samples": count, **cleanup}
    if count < 10:
        result["reasons"].append("Fewer than 10 valid moving paired samples")
        return result
    times, speed, yaw = gps[:, 0], target[:, 3], target[:, 2]
    direction = np.column_stack([np.cos(yaw), np.sin(yaw)])
    error = gps[:, 1:3]-target[:, :2]
    along = np.sum(error*direction, axis=1)
    bias, lag_sec = robust_fit(speed, along)
    lag_ms = float(lag_sec*1000)
    duration = duration_of_samples(times)
    spread = float(np.percentile(speed, 90)-np.percentile(speed, 10))
    evidence = result["evidence"]
    evidence.update(moving_duration_sec=duration, speed_p90_p10_mps=spread,
                    speed_min_max_mps=[float(np.min(speed)), float(np.max(speed))],
                    baseline=metrics(error, direction))
    evidence["input_motion_consistency"] = input_motion_consistency(data["ego"])
    result.update(apparent_relative_lag_ms=lag_ms, spatial_bias_m=float(bias))
    # 2-second block bootstrap, not independent-sample OLS precision on quantized GPS.
    block = np.floor((times-times[0])/2.).astype(int)
    unique = np.unique(block)
    fits = []
    rng = np.random.RandomState(104729)
    if spread >= 2. and len(unique) >= 6:
        indices = [np.flatnonzero(block == b) for b in unique]
        for _ in range(120):
            draw = np.concatenate([indices[k] for k in rng.randint(0, len(indices), len(indices))])
            if np.std(speed[draw]) >= .7:
                fits.append(1000*robust_fit(speed[draw], along[draw])[1])
    ci = np.percentile(fits, [2.5, 97.5]).tolist() if len(fits) >= 60 else None
    evidence["lag_bootstrap_95_interval_ms"] = ci
    evidence["lag_uncertainty_ms"] = float((ci[1]-ci[0])/2) if ci else None
    # A stable global fit must not hide different first/second-half epochs.
    split_lags = []
    for mask in (times <= np.median(times), times > np.median(times)):
        if np.count_nonzero(mask) >= 30 and np.ptp(speed[mask]) >= 2.:
            split_lags.append(float(1000*robust_fit(speed[mask], along[mask])[1]))
        else:
            split_lags.append(None)
    evidence["split_lag_ms"] = split_lags
    curve = []
    for candidate_ms in range(-MAX_LAG_MS, MAX_LAG_MS+1):
        shifted, good = oracle_at(data["ego"], times-candidate_ms/1000.)
        if not np.all(good):
            raise InsufficientEvidence("Common lag coverage unexpectedly changed")
        delta = np.sum((gps[:, 1:3]-shifted[:, :2])*direction, axis=1)
        curve.append({"lag_ms": candidate_ms, "along_mae_m": float(np.mean(np.abs(delta)))})
    evidence["zero_spatial_bias_grid_best_ms"] = min(curve, key=lambda r: r["along_mae_m"])["lag_ms"]
    evidence["lag_curve"] = curve
    reasons = result["reasons"]
    if count < 100:
        reasons.append("Need at least 100 moving raw-GPS samples")
    if duration < min_moving_duration_sec:
        reasons.append("Need at least {:.1f}s moving evidence (got {:.2f}s)".format(min_moving_duration_sec, duration))
    if spread < 2. or np.std(speed) < .7:
        reasons.append("Speed variation too small: fixed spatial bias and time delay are not identifiable; include acceleration and different steady speeds")
    if ci is None or ci[1]-ci[0] > 20.:
        reasons.append("Lag uncertainty exceeds 10ms or insufficient independent time blocks")
    if any(v is None for v in split_lags):
        reasons.append("Both time halves need moving speed variation to check consistency")
    elif abs(split_lags[0]-split_lags[1]) > 15.:
        reasons.append("First/second-half lag differs by more than 15ms; load/clock/input timing is not stable")
    if lag_ms < -5. or lag_ms > MAX_LAG_MS:
        reasons.append("Fitted lag outside selectable nonnegative 0..100ms range")
    motion_consistency = evidence["input_motion_consistency"]
    if (motion_consistency["window_count"] >= 3 and
            abs(motion_consistency["position_to_native_motion_ratio_median"]-1.) > .1):
        reasons.append("Native speed and wall-time Ego displacement disagree by >10%; a fixed GNSS delay is not a sufficient model")
    # Round to a practical 5ms config step, but never silently apply it.
    candidate = int(np.clip(math.floor(lag_ms/5.+.5)*5, 0, MAX_LAG_MS))
    velocity, proxy, valid_header_pair_count = motion_proxy(data, times, target)
    candidate_metrics = metrics(error+velocity*candidate/1000., direction)
    comparison = {"candidate_ms": candidate, "sample_count": count,
                  "baseline": evidence["baseline"], "recommended": candidate_metrics,
                  "motion_source": proxy, "diagnostic_header_pair_valid_samples": valid_header_pair_count,
                  "actual_callback_applied_rate_measured": False,
                  "scope": "same raw input samples; header-time diagnostic only, no callback/EKF/controller re-run"}
    evidence["input_shift_comparison"] = comparison
    # A pure temporal advance cannot be sold as a fix when its net input error worsens.
    if candidate and candidate_metrics["along_mae_m"] > evidence["baseline"]["along_mae_m"] + .01:
        reasons.append("Candidate advance worsens raw-input along MAE; spatial bias or non-temporal error dominates")
    if not reasons:
        result.update(status="ok", recommendation_ms=candidate)
    return result


def aggregate(results):
    if len(results) == 1:
        return results[0]
    usable = [r for r in results if r["status"] == "ok"]
    output = {"schema_version": SCHEMA_VERSION, "status": "insufficient_evidence", "recommendation_ms": None,
              "recommendation_scope": "absolute_raw_gnss_advance", "settings_applied": False,
              "physical_network_latency_measured": False, "reasons": [], "recordings": results}
    if len(usable) != len(results):
        output["reasons"].append("Every supplied recording must independently support the recommendation")
    elif np.ptp([r["apparent_relative_lag_ms"] for r in usable]) > 15.:
        output["reasons"].append("Recordings disagree by more than 15ms; do not collapse different PC/load conditions")
    else:
        median = float(np.median([r["apparent_relative_lag_ms"] for r in usable]))
        output.update(status="ok", apparent_relative_lag_ms=median,
                      recommendation_ms=int(np.clip(math.floor(median/5.+.5)*5, 0, MAX_LAG_MS)))
    return output


def render_report(result):
    lines = ["GNSS apparent relative-epoch audit", "Status: "+result["status"],
             "Recommendation: "+("NONE" if result.get("recommendation_ms") is None else
                                     "gnss_delay_ms:="+str(result["recommendation_ms"])),
             "Meaning: absolute advance from RAW /sensor/gps, NOT additional advance on an already compensated output.",
             "No setting applied. No additional waiting. UDP/header stamps unchanged.",
             "This is relative to offline /Ego_topic, NOT measured physical GNSS/network latency.",
             "Input-shift error comparison is NOT full-EKF or closed-loop validation."]
    if "apparent_relative_lag_ms" in result:
        lines.append("Joint bias/speed fitted apparent lag: {:.2f}ms".format(result["apparent_relative_lag_ms"]))
    for reason in result.get("reasons", []):
        lines.append("HOLD: "+reason)
    if "evidence" in result:
        e = result["evidence"]
        lines.append("Moving samples: {}; supported moving duration: {:.2f}s".format(e["moving_samples"], e.get("moving_duration_sec", 0)))
        if "input_shift_comparison" in e:
            pair = e["input_shift_comparison"]
            lines.append("Same-input along MAE: {:.4f} -> {:.4f}m ({}ms candidate)".format(
                pair["baseline"]["along_mae_m"], pair["recommended"]["along_mae_m"], pair["candidate_ms"]))
            lines.append("Diagnostic motion source: "+pair["motion_source"])
    lines += ["Before tuning planning: verify selected value on a separate run in the SAME PC/network/load configuration.",
              "Do not use this calibration to hide GPS-shadow drift, timestamp jumps, or overloaded simulator motion disagreement."]
    return "\n".join(lines)+"\n"


def write_result(output_dir, result):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir/"result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    report = render_report(result)
    (output_dir/"report.txt").write_text(report, encoding="utf-8")
    print(report, end="")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--heading-offset-deg", type=float, default=-1.347)
    parser.add_argument("--acquisition", choices=["MR", "MRP", "LAN_FIELD", "OFFLINE_REPLAY"], default="MR")
    parser.add_argument("--min-moving-duration-sec", type=float, default=20.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.heading_offset_deg) or not math.isfinite(args.min_moving_duration_sec) or args.min_moving_duration_sec < 0:
        parser.error("heading and minimum duration must be finite; duration must be nonnegative")
    results = []
    try:
        for bag in args.bags:
            try:
                data, provenance = read_bag(bag, args.heading_offset_deg)
                provenance["acquisition"] = args.acquisition
                result = evaluate(data, args.min_moving_duration_sec, provenance)
            except InsufficientEvidence as exc:
                result = {"schema_version": SCHEMA_VERSION, "status": "insufficient_evidence", "recommendation_ms": None,
                          "recommendation_scope": "absolute_raw_gnss_advance", "settings_applied": False,
                          "physical_network_latency_measured": False, "reasons": [str(exc)],
                          "provenance": {"bag": str(bag.resolve()), "acquisition": args.acquisition,
                                         "heading_offset_deg_applied_once_to_raw_status": args.heading_offset_deg}}
            results.append(result)
        result = aggregate(results)
    except Exception as exc:  # actionable CLI errors, never synthesize a successful calibration
        result = {"schema_version": SCHEMA_VERSION, "status": "error", "recommendation_ms": None,
                  "reasons": ["{}: {}".format(type(exc).__name__, exc)], "settings_applied": False}
    write_result(args.output_dir, result)
    return {"ok": 0, "insufficient_evidence": 2, "error": 1}[result["status"]]


if __name__ == "__main__":
    sys.exit(main())
