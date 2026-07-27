"""Publish the latest completed ego paths for the active trajectory sample."""

import json
from datetime import datetime

import rospy
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Path as RosPath

from trajectory_validator.history_publisher import (
    SAFE_NAME,
    append_ego_point,
    load_history_artifacts,
)
from trajectory_validator.recorder import discover_trajectory_file, repository_root
from trajectory_validator.visualization import make_path


def resolve_history_root(root, value):
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("history_dir must stay inside repository") from error
    return path


def load_completed_ego_trajectory(run_directory, traj_sample, run_id):
    """Return one validated ego trajectory, or raise when a run is not usable."""
    metadata_path = run_directory / "metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError("history run is missing metadata.json")

    with metadata_path.open() as stream:
        metadata = json.load(stream)
    if metadata.get("status") != "complete":
        raise ValueError("history run is not complete")
    if metadata.get("traj_sample") != traj_sample or metadata.get("run_id") != run_id:
        raise ValueError("history metadata identity does not match directory")

    root = repository_root()
    _, sample_rows, _ = load_history_artifacts(root, run_directory, metadata)
    if not sample_rows:
        raise ValueError("history run has no ego samples")
    trajectory = Trajectory()
    trajectory.header.frame_id = metadata.get("frame_id") or "map"
    for row in sample_rows:
        append_ego_point(trajectory, row)
    return trajectory


def latest_completed_runs(history_root, traj_sample):
    """Find newest valid completed run directories for one trajectory sample."""
    sample_directory = history_root / traj_sample
    if not sample_directory.is_dir():
        return []

    candidates = []
    for run_directory in sample_directory.iterdir():
        if not run_directory.is_dir() or not SAFE_NAME.fullmatch(run_directory.name):
            continue
        metadata_path = run_directory / "metadata.json"
        try:
            with metadata_path.open() as stream:
                metadata = json.load(stream)
            if (
                metadata.get("status") != "complete"
                or metadata.get("traj_sample") != traj_sample
                or metadata.get("run_id") != run_directory.name
            ):
                continue
            finished_at = metadata.get("finished_at")
            if not isinstance(finished_at, str):
                continue
            candidates.append((datetime.fromisoformat(finished_at), run_directory.name))
        except (OSError, ValueError, json.JSONDecodeError):
            continue

    candidates.sort(reverse=True)
    return [sample_directory / run_id for _, run_id in candidates]


class RecentEgoOverlayPublisher:
    def __init__(self):
        self.root = repository_root()
        _, trajectory_file = discover_trajectory_file(self.root)
        self.traj_sample = trajectory_file.stem
        if not SAFE_NAME.fullmatch(self.traj_sample):
            raise ValueError("trajectory sample basename contains unsupported characters")

        history_dir = rospy.get_param(
            "~storage/history_dir", "base_ws/src/control/trajectory_validator/history"
        )
        self.history_root = resolve_history_root(self.root, history_dir)
        self.count = int(rospy.get_param("~history_overlay/count", 3))
        if self.count < 1:
            raise ValueError("history_overlay/count must be positive")
        self.topic_root = str(
            rospy.get_param("~history_overlay/topic_root", "/trajectory_validator/ego_path")
        ).rstrip("/")
        if not self.topic_root.startswith("/"):
            raise ValueError("history_overlay/topic_root must be an absolute topic")

    def publish(self):
        run_directories = latest_completed_runs(self.history_root, self.traj_sample)
        if not run_directories:
            rospy.loginfo(
                "trajectory history overlay: no completed runs for %s", self.traj_sample
            )
            return

        publishers = []
        for run_directory in run_directories:
            try:
                trajectory = load_completed_ego_trajectory(
                    run_directory, self.traj_sample, run_directory.name
                )
            except (OSError, ValueError, json.JSONDecodeError) as error:
                rospy.logwarn(
                    "trajectory history overlay skipped %s: %s", run_directory.name, error
                )
                continue
            history_id = "his{}".format(len(publishers) + 1)
            topic = "{}/{}".format(self.topic_root, history_id)
            publisher = rospy.Publisher(topic, RosPath, queue_size=1, latch=True)
            publishers.append(publisher)
            publisher.publish(make_path(trajectory, rospy.Time.now()))
            rospy.loginfo(
                "trajectory history overlay %s <- %s (%d points)",
                topic, run_directory.name, len(trajectory.points)
            )
            if len(publishers) == self.count:
                break

        # Keep latched publishers alive until traj_val stops, so RViz opened later
        # receives every selected path.
        self.publishers = publishers


def main():
    rospy.init_node("trajectory_recent_ego_overlay")
    try:
        publisher = RecentEgoOverlayPublisher()
        publisher.publish()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("trajectory history overlay: %s", error)
        raise SystemExit(1)
