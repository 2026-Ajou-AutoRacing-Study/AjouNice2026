#!/usr/bin/env python3

"""Start and stop a fixed planner-debug rosbag through ROS services."""

from datetime import datetime
import os
import shutil
import signal
import subprocess
import threading
import time

import rospy
from std_srvs.srv import Trigger, TriggerResponse


DEFAULT_TOPICS = (
    "/planning/scenario_planning/lane_driving/behavior_planning/path",
    "/planning/path_reference/static_obstacle_avoidance",
    "/planning/path_candidate/static_obstacle_avoidance",
    "/planning/debug/bpp/module_status",
    "/planning/debug/bpp/static_obstacle_avoidance",
    "/planning/scenario_planning/status/stop_reasons",
    "/planning/debug/behavior_velocity_planner/status",
    "/planning/planning_factors/behavior_velocity_planner",
    "/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path",
    "/planning/scenario_planning/lane_driving/motion_planning/path_smoother/trajectory",
    "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory",
    "/path_optimizer/debug/status",
    "/path_optimizer/debug/processing_time_ms",
    "/elastic_band_smoother/debug/processing_time_ms",
    "/planning/scenario_planning/lane_driving/motion_planning/"
    "motion_velocity_planner/raw_trajectory",
    "/control/trajectory",
    "/localization/kinematic_state",
    "/localization/acceleration",
    "/perception/object_recognition/objects",
    "/perception/traffic_light_recognition/traffic_signals",
    "/planning/virtual_traffic_light_states",
    "/tf",
    "/tf_static",
)


def normalize_topics(topics):
    """Return non-empty unique absolute topic names in input order."""
    normalized = []
    seen = set()
    for topic in topics:
        topic = str(topic).strip()
        if not topic:
            continue
        if not topic.startswith("/"):
            topic = "/" + topic
        if topic in seen:
            continue
        seen.add(topic)
        normalized.append(topic)
    return tuple(normalized)


def timestamped_bag_path(output_directory, prefix, now=None):
    """Allocate a timestamped bag filename without overwriting an existing run."""
    now = datetime.now() if now is None else now
    timestamp = now.strftime("%Y-%m-%d-%H-%M-%S")
    base_path = os.path.join(output_directory, "{}_{}".format(prefix, timestamp))
    candidate = base_path + ".bag"
    suffix = 2
    while os.path.exists(candidate) or os.path.exists(candidate + ".active"):
        candidate = "{}_{:02d}.bag".format(base_path, suffix)
        suffix += 1
    return candidate


class PlannerBagRecorder:
    def __init__(self):
        self.output_directory = os.path.abspath(
            os.path.expanduser(
                str(rospy.get_param("~output_dir", "/home/ajou/planner_bags"))
            )
        )
        self.filename_prefix = str(
            rospy.get_param("~filename_prefix", "stopline_debug")
        ).strip()
        if not self.filename_prefix:
            self.filename_prefix = "stopline_debug"
        self.topics = normalize_topics(rospy.get_param("~topics", list(DEFAULT_TOPICS)))
        self.compression = str(rospy.get_param("~compression", "lz4")).strip()
        self.stop_timeout_sec = max(float(rospy.get_param("~stop_timeout_sec", 15.0)), 1.0)
        self.startup_check_sec = max(
            float(rospy.get_param("~startup_check_sec", 0.25)),
            0.0,
        )

        self._lock = threading.Lock()
        self._process = None
        self._output_path = None
        self._start_wall_time = None
        self._closed = False

        self._start_service = rospy.Service("~start", Trigger, self._on_start)
        self._stop_service = rospy.Service("~stop", Trigger, self._on_stop)
        self._monitor_timer = rospy.Timer(rospy.Duration(1.0), self._monitor_process)
        rospy.on_shutdown(self.close)

        rospy.loginfo(
            "planner_bag_recorder: ready; output=%s topics=%d start=%s stop=%s",
            self.output_directory,
            len(self.topics),
            rospy.resolve_name("~start"),
            rospy.resolve_name("~stop"),
        )

    def _record_command(self, output_path):
        command = ["rosbag", "record"]
        if self.compression:
            command.append("--{}".format(self.compression))
        command.extend(["-O", output_path])
        command.extend(self.topics)
        return command

    @staticmethod
    def _process_is_running(process):
        return process is not None and process.poll() is None

    def _start_recording(self):
        with self._lock:
            if self._closed:
                return False, "Bag recorder node is shutting down"
            if self._process_is_running(self._process):
                return False, "Bag recording is already active: {}".format(
                    self._output_path
                )
            if not self.topics:
                return False, "No rosbag topics are configured"
            if shutil.which("rosbag") is None:
                return False, "rosbag executable was not found"

            try:
                os.makedirs(self.output_directory, exist_ok=True)
            except OSError as error:
                return False, "Cannot create bag directory {}: {}".format(
                    self.output_directory,
                    error,
                )
            if not os.access(self.output_directory, os.W_OK):
                return False, "Bag directory is not writable: {}".format(
                    self.output_directory
                )

            output_path = timestamped_bag_path(
                self.output_directory,
                self.filename_prefix,
            )
            command = self._record_command(output_path)
            try:
                process = subprocess.Popen(command, start_new_session=True)
            except OSError as error:
                return False, "Failed to start rosbag record: {}".format(error)

            if self.startup_check_sec > 0.0:
                time.sleep(self.startup_check_sec)
            return_code = process.poll()
            if return_code is not None:
                return False, "rosbag record exited immediately with code {}".format(
                    return_code
                )

            self._process = process
            self._output_path = output_path
            self._start_wall_time = time.monotonic()
            message = "Recording bag: {}".format(output_path)
            rospy.loginfo("planner_bag_recorder: %s", message)
            return True, message

    def _stop_process(self, process):
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            return process.wait(timeout=self.stop_timeout_sec), "SIGINT"
        except subprocess.TimeoutExpired:
            rospy.logwarn(
                "planner_bag_recorder: rosbag did not stop within %.1fs; sending SIGTERM",
                self.stop_timeout_sec,
            )
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            return process.wait(timeout=5.0), "SIGTERM"
        except subprocess.TimeoutExpired:
            rospy.logerr("planner_bag_recorder: rosbag ignored SIGTERM; sending SIGKILL")
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return process.wait(timeout=2.0), "SIGKILL"

    def _stop_recording(self):
        with self._lock:
            process = self._process
            output_path = self._output_path
            if not self._process_is_running(process):
                self._process = None
                return False, "Bag recording is not active"

            return_code, stop_signal = self._stop_process(process)
            self._process = None
            elapsed = (
                time.monotonic() - self._start_wall_time
                if self._start_wall_time is not None
                else 0.0
            )
            self._start_wall_time = None

            active_path = output_path + ".active"
            if os.path.exists(active_path):
                message = (
                    "rosbag stopped with {} but active file remains: {}"
                ).format(stop_signal, active_path)
                rospy.logerr("planner_bag_recorder: %s", message)
                return False, message
            if not os.path.exists(output_path):
                message = "rosbag stopped but output file was not created: {}".format(
                    output_path
                )
                rospy.logerr("planner_bag_recorder: %s", message)
                return False, message

            size_mb = os.path.getsize(output_path) / (1024.0 * 1024.0)
            message = "Saved bag ({:.1f}s, {:.1f} MB): {}".format(
                elapsed,
                size_mb,
                output_path,
            )
            if return_code not in (0, -signal.SIGINT):
                message += " [exit={}]".format(return_code)
            rospy.loginfo("planner_bag_recorder: %s", message)
            return True, message

    def _monitor_process(self, _event):
        with self._lock:
            if self._process is None or self._process.poll() is None:
                return
            rospy.logerr(
                "planner_bag_recorder: rosbag exited unexpectedly with code %s (%s)",
                self._process.returncode,
                self._output_path,
            )
            self._process = None
            self._start_wall_time = None

    def _on_start(self, _request):
        success, message = self._start_recording()
        return TriggerResponse(success=success, message=message)

    def _on_stop(self, _request):
        success, message = self._stop_recording()
        return TriggerResponse(success=success, message=message)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            process = self._process
            self._process = None
        if self._process_is_running(process):
            self._stop_process(process)


def main():
    rospy.init_node("planner_bag_recorder")
    PlannerBagRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
