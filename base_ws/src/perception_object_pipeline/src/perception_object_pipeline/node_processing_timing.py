"""Opt-in callback diagnostics, separate from observation/prediction semantics.

The publish call includes ROS serialization/locking/enqueue work, not a pure
serialization timer or downstream delivery time. Header age on a 30 Hz track
is NOT age since the last detector observation. Keep those names explicit.
"""
from __future__ import annotations

import time


class NodeProcessingTiming:
    def __init__(self):
        self._last_start = None
        self._last_stamp_ns = None
        self._callback_count = 0

    def start(self, message, received_ros_ns):
        wall = time.perf_counter()
        cpu = time.thread_time()
        stamp_ns = message.header.stamp.to_nsec()
        sample = {
            "schema_version": 1,
            "callback_index": self._callback_count,
            "input_stamp_ns": stamp_ns,
            "input_frame_id": message.header.frame_id,
            "received_ros_ns": received_ros_ns,
            "input_header_age_ms": (received_ros_ns-stamp_ns)/1.e6 if stamp_ns else None,
            "callback_start_gap_ms": (wall-self._last_start)*1000 if self._last_start is not None else None,
            "input_header_step_ms": (stamp_ns-self._last_stamp_ns)/1.e6 if self._last_stamp_ns is not None else None,
            "input_objects": len(message.objects),
            "header_age_is_detector_observation_age": False,
        }
        self._last_start = wall
        self._last_stamp_ns = stamp_ns
        self._callback_count += 1
        return sample, wall, cpu

    @staticmethod
    def finish(token, make_start, make_end, publish_end, **counts):
        sample, wall, cpu = token
        sample.update(counts)
        sample.update({
            "message_make_ms": (make_end-make_start)*1000,
            "synchronous_publish_ms": (publish_end-make_end)*1000,
            "callback_wall_ms_excluding_diagnostics": (time.perf_counter()-wall)*1000,
            "callback_thread_cpu_ms_excluding_diagnostics": (time.thread_time()-cpu)*1000,
            "timed_callback_body_includes_diagnostic_bookkeeping": True,
        })
        return sample
