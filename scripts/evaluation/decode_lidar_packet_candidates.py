#!/usr/bin/env python3
"""Decode recorded/reassembled scans with the installed production VLP16 plugin.

Uses an isolated ROS master and a transform-only nodelet, never a UDP driver.
Saves arrays for offline inference. Baseline field equality is mandatory.
"""
import argparse
import collections
import hashlib
import json
import os
import pathlib
import signal
import socket
import subprocess
import threading
import time

import numpy as np

from lidar_packet_reassembly import POLICIES, group_indices, read_source, rotations, quantiles


def cloud_arrays(message):
    types = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    fields = {field.name: field for field in message.fields}
    values = {}
    for name in ("x", "y", "z", "intensity", "ring", "time"):
        field = fields[name]
        dtype = np.dtype((">" if message.is_bigendian else "<") + types[field.datatype])
        values[name] = np.ndarray((message.height, message.width), dtype=dtype,
                                 buffer=message.data, offset=field.offset,
                                 strides=(message.row_step, message.point_step)).reshape(-1).copy()
    return values


def arrays_exact(left, right):
    # Ubuntu/ROS system NumPy predates array_equal(equal_nan=...).
    return (left.shape == right.shape and left.dtype == right.dtype and
            bool(np.all((left == right) | (np.isnan(left) & np.isnan(right)))))


class Decoder:
    def __init__(self, folder, calibration, port):
        self.folder, self.port = folder, port
        self.processes, self.handles = [], []
        self.condition = threading.Condition()
        self.version, self.latest = 0, None
        self.calibration = calibration

    def launch(self, name, command):
        handle = (self.folder / (name + ".log")).open("w")
        self.handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=os.environ.copy())
        self.processes.append(process)
        return process

    def start(self):
        import rosgraph
        import rospy
        from sensor_msgs.msg import PointCloud2
        from velodyne_msgs.msg import VelodyneScan

        with socket.socket() as check:
            check.bind(("127.0.0.1", self.port))
        os.environ["ROS_MASTER_URI"] = f"http://127.0.0.1:{self.port}"
        os.environ["ROS_IP"] = "127.0.0.1"
        self.launch("master", ["roscore", "-p", str(self.port)])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if rosgraph.is_master_online():
                break
            time.sleep(.1)
        else:
            raise RuntimeError("Private master unavailable")
        rospy.init_node("offline_packet_decoder", anonymous=True, disable_signals=True)
        rospy.set_param("/use_sim_time", False)
        self.launch("manager", ["rosrun", "nodelet", "nodelet", "manager", "__name:=probe_manager"])
        self.launch("decoder", ["rosrun", "nodelet", "nodelet", "load",
                                "velodyne_pointcloud/TransformNodelet", "/probe_manager", "__name:=probe_decoder",
                                "_model:=VLP16", f"_calibration:={self.calibration}",
                                "_min_range:=0.4", "_max_range:=130.0", "_organize_cloud:=false",
                                "_fixed_frame:=", "_target_frame:=",
                                "velodyne_packets:=/probe_packets", "velodyne_points:=/probe_cloud"])
        self.subscriber = rospy.Subscriber("/probe_cloud", PointCloud2, self.receive, queue_size=2)
        self.publisher = rospy.Publisher("/probe_packets", VelodyneScan, queue_size=1)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if self.publisher.get_num_connections() > 0:
                return
            if any(process.poll() is not None for process in self.processes):
                raise RuntimeError("Decoder process exited; inspect logs")
            time.sleep(.1)
        raise RuntimeError("Decoder did not subscribe")

    def receive(self, message):
        with self.condition:
            self.latest = message
            self.version += 1
            self.condition.notify_all()

    def decode(self, packets, frame):
        from velodyne_msgs.msg import VelodyneScan

        scan = VelodyneScan()
        scan.header.stamp = packets[0].stamp
        scan.header.frame_id = frame
        scan.packets = packets
        before = self.version
        start = time.monotonic()
        self.publisher.publish(scan)
        with self.condition:
            deadline = time.monotonic() + 5
            while self.version <= before or self.latest.header.stamp != scan.header.stamp:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Decoder response missing or wrong stamp")
                self.condition.wait(remaining)
            return self.latest, (time.monotonic() - start) * 1000

    def close(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for handle in self.handles:
            handle.close()


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    decoder = Decoder(args.output, args.calibration, args.port)
    manifest = {"cases": [], "calibration": str(args.calibration),
                "scope": "Offline same-packet geometry, isolated installed decoder; not live latency certification."}
    try:
        decoder.start()
        for case_index, path in enumerate(args.bag):
            source = read_source(path)
            packets = source["packets"]
            azimuths = [rotations(packet)[0] for packet in packets]
            footer = [int.from_bytes(bytes(packet.data)[1200:1204], "little") for packet in packets]
            folder = args.output / f"case{case_index}"
            folder.mkdir()
            case = {"source_bag": str(path), "case": case_index, "policies": {}, "frames": []}
            frame_name = source["scans"][0]["frame"]
            for policy in POLICIES:
                groups = group_indices(azimuths, policy)
                timings, equal, saved, mismatches = [], 0, 0, []
                policy_folder = folder / policy
                policy_folder.mkdir()
                for index, (start, end) in enumerate(groups):
                    message, elapsed = decoder.decode(packets[start:end], frame_name)
                    arrays = cloud_arrays(message)
                    timings.append(elapsed)
                    stamp = message.header.stamp.to_sec()
                    if policy == "fixed51":
                        original = cloud_arrays(source["clouds"][stamp])
                        if not all(arrays_exact(arrays[name], original[name]) for name in arrays):
                            differences = {name: float(np.nanmax(abs(arrays[name].astype(float) - original[name].astype(float))))
                                           for name in arrays if arrays[name].shape == original[name].shape}
                            mismatches.append({"frame": index, "difference": differences,
                                               "points": [len(arrays["x"]), len(original["x"])]})
                        else:
                            equal += 1
                    npz = policy_folder / f"{index:05d}.npz"
                    np.savez_compressed(npz, **arrays)
                    saved += 1
                    case["frames"].append({"policy": policy, "index": index,
                                            "path": str(npz.relative_to(args.output)), "stamp": stamp,
                                            "packet_start": start, "packet_end": end,
                                            "packet_span_sec": packets[end - 1].stamp.to_sec() - stamp,
                                            "footer_first_usec": footer[start],
                                            "footer_distinct": len(set(footer[start:end])),
                                            "points": len(arrays["x"]), "startup_partial": policy.startswith("cut") and index == 0})
                    if args.limit and saved >= args.limit:
                        break
                case["policies"][policy] = {"decoded": saved, "original_fields_equal": equal,
                                             "mismatches": mismatches, "decode_roundtrip_ms": quantiles(timings)}
                print(json.dumps({"case": case_index, "policy": policy, **case["policies"][policy]}), flush=True)
                if policy == "fixed51" and mismatches:
                    raise RuntimeError("Baseline decode not exact; candidates must not be used for model comparison")
            manifest["cases"].append(case)
            (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    finally:
        decoder.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=pathlib.Path, action="append", required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--calibration", type=pathlib.Path,
                        default=pathlib.Path("/opt/ros/noetic/share/velodyne_pointcloud/params/VLP16db.yaml"))
    parser.add_argument("--port", type=int, default=11319)
    parser.add_argument("--limit", type=int, default=0, help="Smoke limit per policy, 0=all")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
