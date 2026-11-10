#!/usr/bin/python3
"""Record the compact, timestamp-preserving topic set needed for vehicle ID."""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import time

import rospy


DEFAULT_TOPICS = [
    "/ctrl_cmd",
    "/Ego_topic",
    "/Competition_topic",
    "/competition/raw_twist",
    "/sensor/imu",
    "/sensor/gps",
    "/CollisionData",
    "/tf",
    "/tf_static",
    "/mpc_develop/command_status",
]
REQUIRED_TOPICS = ["/Ego_topic", "/sensor/imu", "/sensor/gps"]
SAFE_LABEL = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Record explicit MORAI vehicle-identification topics to one rosbag.")
    parser.add_argument("--label", required=True,
                        help="Experiment label, e.g. steering_step_v10_run01")
    parser.add_argument("--output-dir", default="",
                        help="Parent directory; default is planner_logs/analysis/mpc_develop")
    parser.add_argument("--preflight-timeout", type=float, default=15.0,
                        help="Seconds to wait for each required source topic")
    parser.add_argument("--skip-preflight", action="store_true",
                        help="Start recording without waiting for Ego, IMU, and GPS messages")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def repo_root():
    configured = os.environ.get("AJOUNICE_ROOT")
    if configured:
        return Path(configured).resolve()
    return Path(__file__).resolve().parents[4]


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def wait_for_required_topics(topics, timeout):
    observed = {}
    for topic in topics:
        try:
            message = rospy.wait_for_message(topic, rospy.AnyMsg, timeout=timeout)
            observed[topic] = {
                "received": True,
                "connection_header": dict(getattr(message, "_connection_header", {}) or {}),
            }
            rospy.loginfo("preflight received %s", topic)
        except rospy.ROSException as error:
            observed[topic] = {"received": False, "error": str(error)}
            raise RuntimeError("required topic {} did not arrive: {}".format(topic, error))
    return observed


def published_topics():
    try:
        return dict(rospy.get_published_topics())
    except rospy.ROSException:
        return {}


def main():
    rospy.init_node("mpc_develop_identification_recorder", disable_signals=True)
    arguments = parse_arguments()
    if not SAFE_LABEL.fullmatch(arguments.label):
        raise RuntimeError("--label must contain only letters, digits, '_' and '-'")
    if arguments.preflight_timeout <= 0.0:
        raise RuntimeError("--preflight-timeout must be positive")
    if shutil.which("rosbag") is None:
        raise RuntimeError("rosbag executable is not available in PATH")

    root = repo_root()
    output_parent = (Path(arguments.output_dir).expanduser().resolve()
                     if arguments.output_dir else root / "planner_logs" / "analysis" / "mpc_develop")
    run_id = "{}_{}".format(dt.datetime.now().strftime("%Y%m%d_%H%M%S"), arguments.label)
    run_directory = output_parent / run_id
    run_directory.mkdir(parents=True, exist_ok=False)
    bag_base = run_directory / "identification"
    manifest_path = run_directory / "manifest.json"

    manifest = {
        "schema_version": 1,
        "run_id": run_id,
        "started_at_utc": utc_now(),
        "cwd": os.getcwd(),
        "output_bag": str(bag_base) + ".bag",
        "topics_requested": DEFAULT_TOPICS,
        "topics_required_before_start": [] if arguments.skip_preflight else REQUIRED_TOPICS,
        "published_topics_before_preflight": published_topics(),
        "command": ["rosbag", "record", "--lz4", "-O", str(bag_base)] + DEFAULT_TOPICS,
    }
    atomic_json(manifest_path, manifest)

    if not arguments.skip_preflight:
        try:
            manifest["preflight"] = wait_for_required_topics(
                REQUIRED_TOPICS, arguments.preflight_timeout)
        except RuntimeError as error:
            manifest["preflight_error"] = str(error)
            manifest["finished_at_utc"] = utc_now()
            atomic_json(manifest_path, manifest)
            raise

    command = manifest["command"]
    rospy.loginfo("recording identification bag: %s", " ".join(command))
    process = subprocess.Popen(command, preexec_fn=os.setsid)
    stop_requested = {"value": False}

    def stop_recording(signum, _frame):
        if stop_requested["value"]:
            return
        stop_requested["value"] = True
        rospy.logwarn("stopping rosbag after signal %s", signum)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)

    signal.signal(signal.SIGINT, stop_recording)
    signal.signal(signal.SIGTERM, stop_recording)
    try:
        while process.poll() is None:
            time.sleep(0.2)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=20)
        manifest["finished_at_utc"] = utc_now()
        manifest["rosbag_returncode"] = process.returncode
        manifest["published_topics_at_finish"] = published_topics()
        manifest["bag_exists"] = (bag_base.with_suffix(".bag")).is_file()
        atomic_json(manifest_path, manifest)

    if process.returncode != 0:
        raise RuntimeError("rosbag exited with {}".format(process.returncode))
    rospy.loginfo("bag finalized: %s", bag_base.with_suffix(".bag"))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        rospy.logfatal("mpc_develop identification recorder: %s", error)
        raise SystemExit(1)
