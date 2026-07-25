import argparse
import hashlib
import json
import re
from pathlib import Path

import rospy
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath

from trajectory_validator.csv_io import load_run_csv
from trajectory_validator.history_publisher import append_ego_point, gt_trajectory_from_rows
from trajectory_validator.visualization import make_path


SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "base_ws" / "src").is_dir() and (parent / "kcity_map").is_dir():
            return parent
    raise RuntimeError("cannot locate AjouNice2026 repository root")


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class FinalPathPublisher:
    def __init__(self, traj_sample, run_id, topic_prefix="/trajectory_validator"):
        if not SAFE_NAME.fullmatch(traj_sample) or not SAFE_NAME.fullmatch(run_id):
            raise ValueError("traj sample and run ID must match [A-Za-z0-9_-]+")
        run_directory = (
            repository_root()
            / "base_ws/src/control/trajectory_validator/history"
            / traj_sample
            / run_id
        )
        gt_path = run_directory / "gt.csv"
        control_path = run_directory / "control_cycles.csv"
        response_path = run_directory / "vehicle_responses.csv"
        metadata_path = run_directory / "metadata.json"
        if not all(path.is_file() for path in (gt_path, control_path, response_path, metadata_path)):
            raise FileNotFoundError("final path requires all schema-v3 run files")
        with metadata_path.open() as stream:
            metadata = json.load(stream)
        if metadata.get("schema_version") != 3:
            raise ValueError("final path requires schema_version 3")
        if metadata.get("traj_sample") != traj_sample or metadata.get("run_id") != run_id:
            raise ValueError("history metadata identity does not match requested run")
        integrity = metadata.get("integrity", {})
        if integrity.get("gt_csv_sha256") != sha256_file(gt_path):
            raise ValueError("history GT CSV SHA-256 does not match metadata")
        if integrity.get("control_cycles_csv_sha256") != sha256_file(control_path):
            raise ValueError("history control cycles CSV SHA-256 does not match metadata")
        if integrity.get("vehicle_responses_csv_sha256") != sha256_file(response_path):
            raise ValueError("history vehicle responses CSV SHA-256 does not match metadata")
        gt_rows, sample_rows, _ = load_run_csv(gt_path, control_path, response_path)
        if not sample_rows:
            raise ValueError("history run has no samples")
        frame_id = metadata.get("frame_id") or "map"
        self.gt_trajectory = gt_trajectory_from_rows(gt_rows, frame_id)
        self.ego_trajectory = Trajectory()
        self.ego_trajectory.header.frame_id = frame_id
        for row in sample_rows:
            append_ego_point(self.ego_trajectory, row)

        topic_prefix = "/" + topic_prefix.strip("/")
        self.topic_prefix = topic_prefix
        self.gt_publisher = rospy.Publisher(
            topic_prefix + "/gt_path", RosPath, queue_size=1, latch=True
        )
        self.ego_publisher = rospy.Publisher(
            topic_prefix + "/ego_path", RosPath, queue_size=1, latch=True
        )
        self.pose_publisher = rospy.Publisher(
            topic_prefix + "/ego_pose", PoseStamped, queue_size=1, latch=True
        )

    def publish(self):
        rospy.sleep(0.2)
        stamp = rospy.Time.now()
        self.gt_publisher.publish(make_path(self.gt_trajectory, stamp))
        self.ego_publisher.publish(make_path(self.ego_trajectory, stamp))
        pose = PoseStamped()
        pose.header.frame_id = self.ego_trajectory.header.frame_id
        pose.header.stamp = stamp
        pose.pose = self.ego_trajectory.points[-1].pose
        self.pose_publisher.publish(pose)
        rospy.loginfo(
            "published final paths on %s: GT=%d Ego=%d",
            self.topic_prefix,
            len(self.gt_trajectory.points),
            len(self.ego_trajectory.points),
        )


def main():
    parser = argparse.ArgumentParser(description="Publish one recorded run as complete paths")
    parser.add_argument("--traj-sample")
    parser.add_argument("--run-id")
    arguments = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("trajectory_final_path_publisher")
    try:
        traj_sample = arguments.traj_sample or rospy.get_param("~traj_sample")
        run_id = arguments.run_id or rospy.get_param("~run_id")
        topic_prefix = rospy.get_param("~topic_prefix", "/trajectory_validator")
        publisher = FinalPathPublisher(traj_sample, run_id, topic_prefix)
        publisher.publish()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("final_path: %s", error)
        raise SystemExit(1)
