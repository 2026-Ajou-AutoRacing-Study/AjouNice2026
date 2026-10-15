#!/usr/bin/env python3
"""Offline packet grouping and timing audit; never changes a live driver.

Angle-cut predicate matches ros-drivers/velodyne 1.7.0 driver.cc poll().
https://github.com/ros-drivers/velodyne/blob/1.7.0/velodyne_driver/src/driver/driver.cc
"""
import argparse
import collections
import hashlib
import json
import pathlib
import struct

import numpy as np

POLICIES = ("fixed51", "fixed50", "cut0", "cut180")
TOPIC = "/sensor/lidar1/packets"


def quantiles(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"count": 0}
    return dict(count=len(values), mean=float(values.mean()), min=float(values.min()),
                median=float(np.median(values)), p90=float(np.percentile(values, 90)),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)),
                max=float(values.max()))


def rotations(packet):
    data = bytes(packet.data)
    if len(data) != 1206:
        raise ValueError("Expected 1206-byte VLP16 packet")
    result = []
    for offset in range(0, 1200, 100):
        marker, angle = struct.unpack_from("<HH", data, offset)
        if marker != 0xEEFF or angle >= 36000:
            raise ValueError("Invalid block marker/azimuth")
        result.append(angle)
    return result


def crosses_cut(previous, current, cut):
    return ((previous < cut <= current) or
            (cut <= current < previous) or
            (current < previous < cut))


def footer_nearest_hour(packet_stamp, usec):
    """Interpret the standard microseconds-within-hour field, not ROS now()."""
    if not 0 <= usec < 3600000000:
        raise ValueError("Footer timestamp outside microseconds-within-hour format")
    hour = np.floor(packet_stamp / 3600) * 3600
    candidates = [hour + offset + usec * 1e-6 for offset in (-3600, 0, 3600)]
    return min(candidates, key=lambda timestamp: abs(timestamp - packet_stamp))


def group_indices(azimuths, policy):
    """Half-open packet index ranges; trailing partial scan is not published."""
    if policy not in POLICIES:
        raise ValueError(policy)
    if policy.startswith("fixed"):
        count = int(policy[5:])
        return [(start, start + count) for start in range(0, len(azimuths) - count + 1, count)]
    cut = 0 if policy == "cut0" else 18000
    groups, start, previous = [], 0, None
    for index, angle in enumerate(azimuths):
        if previous is not None and crosses_cut(previous, angle, cut):
            groups.append((start, index + 1))
            start = index + 1
        previous = angle
    return groups


def coverage_deg(block_rotations, start, end):
    """Angular extent including the last block interval, not observed-object FOV."""
    angles = np.asarray(block_rotations[start:end], dtype=np.int64).reshape(-1)
    if len(angles) < 2:
        return 0.0
    steps = np.diff(angles) % 36000
    if np.any(steps >= 18000):
        raise ValueError("Azimuth order reversal or large discontinuity")
    return float((steps.sum() + np.median(steps)) / 100)


def read_source(path):
    import rosbag

    packets, scan_rows, clouds, messages = [], [], {}, collections.defaultdict(list)
    wanted = [TOPIC, "/sensor/lidar1", "/Ego_topic", "/Competition_topic", "/Object_topic"]
    wanted += [f"/sensor/camera/{camera}/compressed" for camera in ("front", "front_left", "front_right", "back")]
    with rosbag.Bag(str(path)) as bag:
        for topic, message, received in bag.read_messages(topics=wanted):
            stamp = message.header.stamp.to_sec()
            if topic == TOPIC:
                start = len(packets)
                packets.extend(message.packets)
                scan_rows.append(dict(start=start, end=len(packets), stamp=stamp,
                                      receipt=received.to_sec(), frame=message.header.frame_id))
            elif topic == "/sensor/lidar1":
                if stamp in clouds:
                    raise ValueError("Duplicate original PointCloud stamp")
                clouds[stamp] = message
                messages[topic].append((stamp, received.to_sec(), message))
            else:
                messages[topic].append((stamp, received.to_sec(), message))
    for rows in messages.values():
        rows.sort(key=lambda row: row[0])
    return dict(packets=packets, scans=scan_rows, clouds=clouds, messages=messages)


def audit(path, source):
    packets = source["packets"]
    if not packets:
        raise ValueError("No raw packets: cannot reconstruct grouping from XYZI alone")
    stamps = np.asarray([packet.stamp.to_sec() for packet in packets])
    blocks = [rotations(packet) for packet in packets]
    azimuths = np.asarray([angles[0] for angles in blocks], dtype=np.int64)
    footers = [struct.unpack_from("<I", bytes(packet.data), 1200)[0] for packet in packets]
    footer_times = np.asarray([footer_nearest_hour(stamp, footer) for stamp, footer in zip(stamps, footers)])
    first_in_batch = np.r_[True, np.diff(np.asarray(footers, dtype=np.int64)) != 0]
    batch_starts = np.flatnonzero(first_in_batch)
    batch_ends = np.r_[batch_starts[1:], len(packets)]
    delta = np.diff(azimuths) % 36000
    original = source["scans"]
    result = {
        "source": str(path), "packets": len(packets), "original_scans": len(original),
        "original_packet_counts": dict(collections.Counter(row["end"] - row["start"] for row in original)),
        "packet_step_deg": quantiles(delta / 100),
        "packet_interarrival_ms": quantiles(np.diff(stamps) * 1000),
        "nonmonotonic_packet_stamps": int(np.count_nonzero(np.diff(stamps) < 0)),
        "azimuth_repeated_adjacent_packets": int(np.count_nonzero(delta == 0)),
        "azimuth_steps_over_7_2deg": int(np.count_nonzero(delta > 720)),
        "packet_footer_modes": dict(collections.Counter(bytes(packet.data)[1204:].hex() for packet in packets)),
        "footer_stamp_changes": int(np.count_nonzero(np.diff(np.asarray(footers, dtype=np.int64)))),
        "footer_batch_packet_counts": dict(collections.Counter(
            int(end-start) for start, end in zip(batch_starts, batch_ends))),
        "footer_batch_period_ms": quantiles(np.diff(footer_times[first_in_batch])*1000),
        "footer_minus_host_stamp_first_in_batch_ms": quantiles((footer_times - stamps)[first_in_batch] * 1000),
        "footer_minus_host_stamp_other_packets_ms": quantiles((footer_times - stamps)[~first_in_batch] * 1000),
        "packet_gaps_over_5ms_arrival_azimuths": dict(collections.Counter(
            str(float(azimuths[index + 1]) / 100) for index in np.flatnonzero(np.diff(stamps) > .005))),
        "original_scan_header_minus_first_packet_ms": quantiles(
            [(row["stamp"] - stamps[row["start"]]) * 1000 for row in original]),
        "original_scan_receipt_minus_first_packet_ms": quantiles(
            [(row["receipt"] - stamps[row["start"]]) * 1000 for row in original]),
        "original_scan_receipt_minus_last_packet_ms": quantiles(
            [(row["receipt"] - stamps[row["end"] - 1]) * 1000 for row in original]),
        "original_cloud_receipt_minus_header_ms": quantiles(
            [(received - stamp) * 1000 for stamp, received, _ in source["messages"]["/sensor/lidar1"]]),
        "topic_receipt_minus_header_ms": {topic: quantiles(
            [(received-stamp)*1000 for stamp, received, _ in rows])
            for topic, rows in source['messages'].items()},
        "policies": {},
        "limitations": ["Packet stamp is the host read midpoint, not exact NIC/simulator acquisition time.",
                        "Original bag includes original grouping/ROS scheduling; candidate publication latency is not measured.",
                        "Angular extent describes packet bins; no-return rays and occlusion require decoded-cloud/GT analysis."],
    }
    for policy in POLICIES:
        groups = group_indices(azimuths, policy)
        extents = [coverage_deg(blocks, start, end) for start, end in groups]
        origins = [int(azimuths[start]) / 100 for start, _ in groups]
        result["policies"][policy] = dict(
            frames=len(groups), trailing_unpublished_packets=len(packets) - groups[-1][1] if groups else len(packets),
            startup_partial_frames=int(policy.startswith("cut") and bool(extents) and extents[0] < 350),
            packet_counts=dict(collections.Counter(end - start for start, end in groups)),
            packet_span_ms=quantiles([(stamps[end - 1] - stamps[start]) * 1000 for start, end in groups]),
            extent_deg=quantiles(extents[1:] if policy.startswith("cut") else extents),
            start_angle_unique_count=len(set(origins[1:] if policy.startswith("cut") else origins)),
            start_angle_step_deg=quantiles([((after - before + 180) % 360) - 180
                                          for before, after in zip(origins, origins[1:])]),
            original_grouping_exact=(groups == [(row["start"], row["end"]) for row in original]))
        result["policies"][policy]["distinct_footer_stamps_per_frame"] = dict(collections.Counter(
            len(set(footers[start:end])) for start, end in groups))
        result["policies"][policy]["footer_first_minus_reference_stamp_ms"] = quantiles(
            [(footer_times[start] - stamps[start]) * 1000 for start, _ in groups])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=pathlib.Path, action="append", required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Fresh output required")
    args.output.mkdir(parents=True)
    rows = []
    for path in args.bag:
        result = audit(path, read_source(path))
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for data in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(data)
        result["bag_sha256"] = digest.hexdigest()
        rows.append(result)
        print(json.dumps({"bag": path.name, "packets": result["packets"],
                          "step_deg": result["packet_step_deg"], "policies": result["policies"]}, indent=2))
    (args.output / "audit.json").write_text(json.dumps(rows, indent=2) + "\n")


if __name__ == "__main__":
    main()
