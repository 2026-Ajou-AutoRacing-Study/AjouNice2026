#!/usr/bin/env python3
"""Read-only traffic-light camera viewer and diagnostic-only overlay helpers.

No detector, native API, vehicle control, TF, or planner publisher is used here.
The viewer subscribes only to an already annotated compressed image. Existing
Validator desktop recording can capture this window on the same DISPLAY.
"""
import math
import os
import threading
import time

import cv2
import numpy as np


WINDOW_TITLE = "AjouNice Traffic Light | YOLO + CNN"
MISSION_NAMES = {201351: "A", 201365: "B", 201290: "C (2 heads)", 201326: "D"}
ALL_BOX_COLOR = (155, 155, 155)
SELECTED_BOX_COLOR = (255, 225, 0)
CROP_BOX_COLOR = (0, 180, 255)


def _bool_label(value):
    return "ON" if value is True else "OFF" if value is False else "?"


def _age_label(value):
    return "?" if not isinstance(value, (int, float)) or not math.isfinite(value) else "%.3fs" % value


def overlay_lines(metadata):
    """Keep observed/model/profile/output provenance distinct, including missing."""
    group = metadata.get("group_id", 0)
    timing = metadata.get("observation_timing") or {}
    observed = timing.get("output_is_observed") is True
    held = timing.get("output_is_held") is True
    selection = str(metadata.get("selection_status", "unknown"))
    state = ("HELD OBSERVATION" if held else "CURRENT OBSERVATION") if observed else "FALLBACK - NOT A LAMP OBSERVATION"
    if selection == "missing":
        state = "MISSING | " + state
    params = metadata.get("planner_parameters") or {}
    age, ttl = timing.get("output_observation_age_sec"), params.get("tl_state_timeout")
    if (observed and isinstance(age, (int, float)) and isinstance(ttl, (int, float))
            and math.isfinite(age) and math.isfinite(ttl) and age > ttl):
        state += " | LAMP AGE EXCEEDS CONFIGURED TTL"
    raw = metadata.get("observed_lamps", "UNKNOWN")
    if metadata.get("classifier_known") is not True:
        raw = "UNKNOWN (no trusted crop/classification)"
    return [
        "MISSION %s | gid=%s | selected head=%s" % (MISSION_NAMES.get(group, "NONE"), group, metadata.get("head_id") or "NONE"),
        "RAW %s: %s" % (str(metadata.get("classifier_mode", "classifier")).upper(), raw),
        "PROFILE/CORRECTION: %s" % metadata.get("normalized_lamps", "UNKNOWN"),
        "OUTPUT LAMPS: %s | conf=%.2f | NOT a planner GO decision" % (metadata.get("final_lamps", "UNKNOWN"), float(metadata.get("confidence", 0.))),
        "STATE: %s | association=%s (%s)" % (state, selection, metadata.get("selection_reason", "?")),
        "JPEG age=%s | adopted lamp age=%s | origin=%s" % (_age_label(metadata.get("image_age_sec")), _age_label(timing.get("output_observation_age_sec")) if observed else "NONE", timing.get("output_origin", "?")),
        "JPEG ns=%s | adopted ns=%s" % (metadata.get("source_stamp_ns", "?"), timing.get("adopted_observation_source_stamp_ns") if observed else "NONE"),
        "PROFILE: %s | TRANSITION: %s" % (metadata.get("policy_reason", "?"), metadata.get("transition_reason", "?")),
        "Planner CONFIG ONLY: guard=%s yellow_passthrough=%s TTL=%s | flag is NOT GO permission" % (_bool_label(params.get("enable_signal_compliance_guard")), _bool_label(params.get("enable_yellow_passthrough")), _age_label(params.get("tl_state_timeout"))),
        "BOXES: gray=all YOLO candidates | cyan=selected housing | orange=CNN crop | near retry=%s" % metadata.get("near_retry", "OFF"),
    ]


def render_node_overlay(image, boxes, selected_bbox, crop_bbox, metadata):
    """Draw on a copy, after canonical publishing; never alter inference input."""
    frame = image.copy()
    height, width = frame.shape[:2]
    for index, box in enumerate(boxes):
        x1, y1, x2, y2, confidence = box[4]
        pt1, pt2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(frame, pt1, pt2, ALL_BOX_COLOR, 1)
        cv2.putText(frame, "#%d %.2f" % (index, confidence), (max(0, pt1[0]), min(height-4, max(15, pt1[1]-4))), cv2.FONT_HERSHEY_SIMPLEX, .38, ALL_BOX_COLOR, 1, cv2.LINE_AA)
    if crop_bbox is not None:
        x1, y1, x2, y2 = [int(x) for x in crop_bbox]
        cv2.rectangle(frame, (x1, y1), (x2, y2), CROP_BOX_COLOR, 1)
    if selected_bbox is not None:
        x1, y1, x2, y2 = [int(x) for x in selected_bbox[:4]]
        cv2.rectangle(frame, (x1, y1), (x2, y2), SELECTED_BOX_COLOR, 3)
    lines = overlay_lines(metadata)
    panel = np.full((40+len(lines)*23, width, 3), (22, 25, 29), dtype=np.uint8)
    for i, color in enumerate(((0, 0, 240), (0, 200, 255), (0, 210, 0))):
        cv2.circle(panel, (14+i*17, 17), 5, color, -1)
    cv2.putText(panel, "AJOUNICE  |  TRAFFIC LIGHT - camera / model / policy", (68, 24), cv2.FONT_HERSHEY_SIMPLEX, .58, (240, 240, 240), 1, cv2.LINE_AA)
    for index, line in enumerate(lines):
        # Original camera FOV is not covered by the diagnostic text panel.
        color = (0, 200, 255) if index == 4 and (metadata.get("selection_status") == "missing"
            or (metadata.get("observation_timing") or {}).get("output_is_held")
            or not (metadata.get("observation_timing") or {}).get("output_is_observed")) else (225, 225, 225)
        cv2.putText(panel, line, (10, 53+index*23), cv2.FONT_HERSHEY_SIMPLEX, .49, color, 1, cv2.LINE_AA)
    return np.vstack((frame, panel))


def frame_freshness(now_monotonic, receipt_monotonic, now_ros_ns, source_ns, stale_sec=.5):
    """Wall time also detects frozen ROS time; never present an old green as live."""
    if receipt_monotonic is None:
        return {"stale": True, "reason": "WAITING FOR DEBUG IMAGE", "receipt_age_sec": None, "source_age_sec": None}
    receipt_age = now_monotonic-receipt_monotonic
    if not math.isfinite(receipt_age) or receipt_age < 0:
        return {"stale": True, "reason": "INVALID RECEIPT CLOCK", "receipt_age_sec": receipt_age, "source_age_sec": None}
    if type(source_ns) is not int or source_ns <= 0 or type(now_ros_ns) is not int or now_ros_ns <= 0:
        return {"stale": True, "reason": "UNVERIFIED IMAGE TIME", "receipt_age_sec": receipt_age, "source_age_sec": None}
    source_age = (now_ros_ns-source_ns)/1e9
    reason = "LIVE DEBUG IMAGE"
    if source_age < -.05:
        reason = "IMAGE CLOCK IN FUTURE"
    elif receipt_age > stale_sec or source_age > stale_sec:
        reason = "STALE - LAST IMAGE IS NOT CURRENT SIGNAL"
    return {"stale": reason != "LIVE DEBUG IMAGE", "reason": reason, "receipt_age_sec": receipt_age, "source_age_sec": source_age}


def render_viewer_frame(image, freshness, error=None):
    if image is None:
        image = np.zeros((720, 1280, 3), np.uint8)
    frame = image.copy()
    stale = freshness["stale"] or bool(error)
    if stale:
        # Desaturate the entire frozen overlay too: a retained green lamp label
        # must not look like a current permission during camera/node loss.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.cvtColor((gray*.4).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    bar = np.full((60, frame.shape[1], 3), (35, 35, 120) if stale else (35, 60, 35), dtype=np.uint8)
    text = ("DECODE ERROR - LAST IMAGE IS NOT CURRENT SIGNAL" if error else freshness["reason"])
    if not stale:
        text += " - lamp freshness/held status below, NOT planner permission"
    cv2.putText(bar, text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .62, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(bar, "receive age=%s | JPEG age=%s | read-only view | Q / Esc: close this viewer" % (_age_label(freshness["receipt_age_sec"]), _age_label(freshness["source_age_sec"])), (12, 48), cv2.FONT_HERSHEY_SIMPLEX, .45, (235, 235, 235), 1, cv2.LINE_AA)
    return np.vstack((bar, frame))


class FrameMailbox:
    """Bounded latest-frame mailbox; JPEG decoding stays out of ROS callbacks."""
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None
        self._sequence = 0

    def accept(self, payload, source_ns, receipt_monotonic):
        with self._lock:
            self._sequence += 1
            self._latest = (self._sequence, bytes(payload), source_ns, receipt_monotonic)

    def snapshot(self):
        with self._lock:
            return self._latest


def window_closed():
    """OpenCV4.2 GTK lacks VISIBLE (-1 even for a real, visible window)."""
    try:
        visible = cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE)
        if visible >= 0:
            return visible < 1
        return cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_AUTOSIZE) < 0
    except cv2.error:
        return True


def main():
    # System ROS/OpenCV only: importing this module for pure tests has no ROS
    # or window side effects, and never imports PyTorch/Ultralytics.
    import rospy
    from sensor_msgs.msg import CompressedImage
    rospy.init_node("traffic_light_debug_view")
    if not os.environ.get("DISPLAY"):
        rospy.logerr("Traffic light viewer needs DISPLAY; use display:=:0 or the recording display.")
        return 2
    topic = rospy.get_param("~image_topic", "/traffic_light_node/debug_image/compressed")
    stale_sec = float(rospy.get_param("~stale_sec", .5))
    if not math.isfinite(stale_sec) or not .1 <= stale_sec <= 5.:
        raise ValueError("stale_sec must be finite within0.1..5.0")
    width, height = int(rospy.get_param("~window_width", 1100)), int(rospy.get_param("~window_height", 850))
    mailbox = FrameMailbox()

    def callback(message):
        mailbox.accept(message.data, int(message.header.stamp.to_nsec()), time.monotonic())

    subscriber = rospy.Subscriber(topic, CompressedImage, callback, queue_size=1, buff_size=2**24)
    last_sequence = None
    decoded = None
    decoded_receipt = None
    decoded_source = None
    decode_error = None
    try:
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_TITLE, width, height)
        rospy.loginfo("Read-only traffic light window: %s (%s)", WINDOW_TITLE, topic)
        while not rospy.is_shutdown():
            latest = mailbox.snapshot()
            if latest is not None and latest[0] != last_sequence:
                last_sequence, payload, source_ns, receipt = latest
                try:
                    frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        raise ValueError("JPEG decoder returned no image")
                    decoded, decoded_source, decoded_receipt = frame, source_ns, receipt
                    decode_error = None
                except (ValueError, cv2.error) as error:
                    # A corrupt new message cannot refresh the old frame age.
                    decode_error = str(error)
                    rospy.logwarn_throttle(5., "Traffic light debug JPEG decode failed")
            freshness = frame_freshness(time.monotonic(), decoded_receipt, int(rospy.Time.now().to_nsec()), decoded_source, stale_sec)
            cv2.imshow(WINDOW_TITLE, render_viewer_frame(decoded, freshness, decode_error))
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q"), ord("Q")) or window_closed():
                rospy.signal_shutdown("Only the traffic light debug viewer was closed")
                break
            time.sleep(.03)
    except cv2.error as error:
        rospy.logerr("Traffic light viewer needs GUI-enabled system python3-opencv: %s", error)
        return 2
    finally:
        subscriber.unregister()
        try:
            cv2.destroyWindow(WINDOW_TITLE)
        except cv2.error:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
