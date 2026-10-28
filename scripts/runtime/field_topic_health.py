#!/usr/bin/env python3
"""Read-only, bounded terminal telemetry; never a vehicle safety decision.

The state model has no ROS imports. ``create_topic_health`` optionally attaches
to existing small topics after the console has launched its owned process.
No publisher, parameter change, image decode, cloud parsing or prediction-path
subscription is used here.
"""

from collections import Counter, deque
from dataclasses import dataclass, field
import math
import os
import struct
import threading
import time
import xmlrpc.client


COMPONENTS = {
    'udp': 'udp', 'udp_lan': 'udp', 'sensing': 'sensing',
    'sensing_lan': 'sensing', 'localization': 'localization',
    'bevfusion': 'bevfusion', 'tracking': 'tracking', 'traffic': 'traffic',
}
GROUPS = {201351: 'A', 201365: 'B', 201290: 'C', 201326: 'D', 201397: 'D1'}
STATIC_CHILDREN = frozenset((
    'lidar1', 'camera_front', 'camera_front_left', 'camera_front_right',
    'camera_back', 'gps', 'imu',
))


def _stamp_seconds(stamp):
    if stamp is None:
        return None
    try:
        if hasattr(stamp, 'to_sec'):
            value = float(stamp.to_sec())
        elif hasattr(stamp, 'secs'):
            value = float(stamp.secs) + float(stamp.nsecs) / 1e9
        elif hasattr(stamp, 'sec'):
            value = float(stamp.sec) + float(stamp.nanosec) / 1e9
        else:
            value = float(stamp)
    except (ValueError, TypeError, AttributeError, OverflowError):
        return None
    return value if math.isfinite(value) and value > 0 else None


def _header_stamp(message):
    return _stamp_seconds(getattr(getattr(message, 'header', None), 'stamp', None))


def _ros_now_seconds(now_ros):
    return _stamp_seconds(now_ros)


def _counts(message):
    result = Counter(V=0, P=0, O=0)
    for obj in getattr(message, 'objects', ()):
        classes = getattr(obj, 'classification', ())
        dominant = max(classes, key=lambda item: getattr(item, 'probability', 0), default=None)
        label = getattr(dominant, 'label', 0)
        # Repository contract: UNKNOWN is the generic obstacle detector class.
        result['V' if label in (1, 2, 3, 4) else 'P' if label == 7 else 'O'] += 1
    return 'V%d P%d O%d' % (result['V'], result['P'], result['O'])


def _lamps(tuples):
    labels = []
    for row in tuples or ():
        if not isinstance(row, (list, tuple)) or len(row) < 2:
            continue
        color, shape = str(row[0]).lower(), str(row[1]).lower()
        initial = {'red': 'R', 'yellow': 'Y', 'green': 'G'}.get(color)
        if initial is None:
            continue
        # CLASS_TUPLES uses (color, shape, is_left); the third field is a
        # boolean direction, not classifier confidence. Malformed directions
        # must not be invented as a left arrow by the display.
        suffix = (('LA' if row[2] is True else 'RA' if row[2] is False else 'A?')
                  if len(row) > 2 else 'A?') if shape == 'arrow' else 'C'
        labels.append(initial + suffix)
    return '+'.join(labels) if labels else 'unreadable'


@dataclass
class _Sample:
    receipt: float
    source: float = None
    detail: str = ''
    valid: bool = True
    times: deque = field(default_factory=lambda: deque(maxlen=256))


class TopicHealth:
    """Pure state model. Call ``observe`` from a short read-only callback.

    ``snapshot`` accepts injected clocks for deterministic checks. Receipt age
    uses monotonic time; source age uses ROS time only when both are valid.
    Every queue has a fixed bound. A zero-object output is still a fresh frame.
    """

    def __init__(self, component, clock=None):
        self.component = COMPONENTS.get(component, component)
        self._clock = clock or time.monotonic
        self.started = self._clock()
        self.samples = {}
        self.static_children = set()
        self.selection = {}
        self.route_group_id = None
        self.frame_group_id = None
        self.parse_errors = 0
        self._lock = threading.RLock()
        self._subscriptions = []
        self._ros_now = None
        self._closed = False

    def _record(self, key, receipt, source=None, detail='', valid=True):
        previous = self.samples.get(key)
        sample = _Sample(receipt, source, detail, valid,
                         previous.times if previous is not None else deque(maxlen=256))
        if sample.times and receipt < sample.times[-1]:
            sample.times.clear()
        sample.times.append(receipt)
        self.samples[key] = sample

    def observe(self, key, message, receipt=None):
        """Accept an existing ROS-like message; malformed telemetry stays local."""
        receipt = self._clock() if receipt is None else float(receipt)
        try:
            with self._lock:
                if self._closed:
                    return
                source = _header_stamp(message)
                if key in ('od', 'trk'):
                    self._record(key, receipt, source, _counts(message))
                elif key == 'gps':
                    latitude, longitude = float(message.latitude), float(message.longitude)
                    valid = (math.isfinite(latitude) and math.isfinite(longitude)
                             and -90 <= latitude <= 90 and -180 <= longitude <= 180
                             and not (latitude == 0 and longitude == 0))
                    self._record(key, receipt, source, valid=valid)
                elif key in ('comp', 'imu', 'global', 'smooth'):
                    self._record(key, receipt, source)
                elif key == 'route':
                    self.route_group_id = int(message.data)
                    self._record(key, receipt, detail=str(self.route_group_id))
                elif key == 'lidar':
                    # VelodyneScan's first field is Header. AnyMsg avoids parsing
                    # every packet. No cloud/JPEG subscription is introduced.
                    buffer = getattr(message, '_buff', b'')
                    if len(buffer) >= 12:
                        secs, nsecs = struct.unpack_from('<II', buffer, 4)
                        source = _stamp_seconds(secs + nsecs / 1e9)
                    self._record(key, receipt, source)
                elif key == 'tf':
                    for transform in getattr(message, 'transforms', ()):
                        if getattr(transform.header, 'frame_id', '').strip('/') == 'base_link':
                            self.static_children.add(transform.child_frame_id.strip('/'))
                    # Static transforms are latched and intentionally timeless.
                    self._record(key, receipt)
                elif key in ('tl', 'selection'):
                    import json
                    packet = json.loads(message.data)
                    if not isinstance(packet, dict):
                        raise ValueError('expected JSON object')
                    if key == 'selection':
                        self.selection = packet
                        return
                    decision = packet.get('frame_decision')
                    stamp_ns = packet.get('source_stamp_ns')
                    # Latched startup metadata is not a processed camera frame.
                    if not isinstance(decision, dict) or not stamp_ns:
                        return
                    source = _stamp_seconds(float(stamp_ns) / 1e9)
                    group_id = decision.get('group_id')
                    self.frame_group_id = group_id
                    mission = GROUPS.get(group_id, '-')
                    status = decision.get('selection_status')
                    gate = decision.get('observation_distance_gate') or {}
                    if group_id == 0:
                        # Confirmed against the independent selector heartbeat
                        # in snapshot: camera fallback also uses group 0.
                        detail = 'no mission'
                    elif gate.get('suppressed'):
                        detail = mission + ' far'
                    elif status != 'matched':
                        reason = str(self.selection.get('reason', ''))
                        detail = mission + (' route-none' if 'route' in reason else ' no-box')
                    else:
                        detail = mission + ' ' + _lamps(decision.get('final_tuples'))
                    self._record(key, receipt, source, detail,
                                 valid=(status == 'matched' and bool(decision.get('final_tuples')))
                                 or bool(gate.get('suppressed')) or group_id == 0)
        except (AttributeError, TypeError, ValueError, OverflowError, struct.error):
            with self._lock:
                self.parse_errors += 1

    def _rate(self, key, now):
        sample = self.samples.get(key)
        if sample is None:
            return 0.
        times = [stamp for stamp in sample.times if stamp >= now - 3.]
        if len(times) < 2 or times[-1] <= times[0]:
            return 0.
        return (len(times) - 1) / (times[-1] - times[0])

    def _age(self, key, now, ros_now):
        sample = self.samples.get(key)
        if sample is None:
            return None
        receipt = max(0., now - sample.receipt)
        source = (ros_now - sample.source
                  if ros_now is not None and sample.source is not None else None)
        # Negative source ages mean clock mismatch/reset, not stale receipt.
        return receipt, source

    def snapshot(self, now_monotonic=None, now_ros=None):
        with self._lock:
            if self._closed or self.component not in set(COMPONENTS.values()):
                return None
            now = self._clock() if now_monotonic is None else float(now_monotonic)
            if now_ros is None and self._ros_now is not None:
                try:
                    now_ros = self._ros_now()
                except Exception:
                    pass
            ros_now = _ros_now_seconds(now_ros)
            if self.component == 'sensing':
                count = len(self.static_children & STATIC_CHILDREN)
                return ('static %d/%d' % (count, len(STATIC_CHILDREN)),
                        'INFO' if count == len(STATIC_CHILDREN) else 'WARN')
            required = {
                'udp': ('comp', 'gps', 'imu'),
                'localization': ('global', 'smooth', 'gps', 'comp'),
                'bevfusion': ('od',), 'tracking': ('trk',), 'traffic': ('tl', 'route'),
            }[self.component]
            missing = [key for key in required if key not in self.samples]
            names = {'comp': 'Competition', 'gps': 'GPS', 'imu': 'IMU',
                     'global': 'global', 'smooth': 'smooth', 'od': 'output',
                     'trk': 'output', 'tl': 'output', 'route': 'route selector'}
            if missing:
                return ('waiting ' + names[missing[0]],
                        'WARN' if now - self.started < 5. else 'ERROR')
            threshold = .5 if self.component == 'localization' else 1.
            stale = []
            clock_bad = False
            for key in required:
                receipt_age, source_age = self._age(key, now, ros_now)
                age = max(receipt_age, source_age or 0.)
                if source_age is not None and source_age < -.5:
                    clock_bad = True
                if age > threshold:
                    stale.append((key, age))
            if stale:
                key, age = max(stale, key=lambda pair: pair[1])
                return ('%s stale %.1fs' % (names[key], age), 'ERROR')
            if clock_bad:
                return ('source clock differs', 'WARN')
            if self.component == 'traffic':
                # UInt64 has no Header and is latched. Require two receipts
                # from the 10Hz selector, rather than treating one old latch as
                # an independently verified healthy selector stream.
                if len(self.samples['route'].times) < 2:
                    return ('waiting route stream', 'WARN')
                if self.frame_group_id != self.route_group_id:
                    # Normal near a mission handoff; do not pair an old camera
                    # frame with a different newly selected mission.
                    return ('mission updating', 'WARN')
            level = 'INFO'
            if self.component == 'udp':
                detail = 'C%.0f G%.0f I%.0f' % tuple(self._rate(key, now) for key in required)
                if any(self._rate(key, now) < minimum for key, minimum in
                       (('comp', 10.), ('gps', 5.), ('imu', 10.))):
                    level = 'WARN'
                if 'lidar' in self.samples:
                    age = self._age('lidar', now, ros_now)
                    detail += ' L%.1f' % self._rate('lidar', now)
                    if max(age[0], age[1] or 0.) > 1.:
                        detail += ' stale'
                        level = 'ERROR'
                    elif self._rate('lidar', now) < 8.:
                        level = 'WARN'
                else:
                    detail += ' L-wait'
                    level = 'WARN'
                if not self.samples['gps'].valid:
                    detail += ' GPS shadow'
                    if level != 'ERROR':
                        level = 'WARN'
            elif self.component == 'localization':
                detail = '%.0fHz GPS %s' % (self._rate('global', now),
                    'valid' if self.samples['gps'].valid else 'shadow')
                if not self.samples['gps'].valid:
                    level = 'WARN'
                if self._rate('global', now) < 10. or self._rate('smooth', now) < 10.:
                    level = 'WARN'
            else:
                key = required[0]
                detail = self.samples[key].detail + ' %.1fHz' % self._rate(key, now)
                source_age = self._age(key, now, ros_now)[1]
                if source_age is not None:
                    detail += ' %dms' % round(max(0., source_age) * 1000)
                if self._rate(key, now) < 2. and self.samples[key].detail != 'no mission':
                    level = 'WARN'
                if self.component == 'traffic' and not self.samples[key].valid:
                    level = 'WARN'
            return detail, level

    def close(self):
        with self._lock:
            self._closed = True
            subscriptions, self._subscriptions = self._subscriptions, []
        for subscriber in subscriptions:
            try:
                subscriber.unregister()
            except Exception:
                pass


class _TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .35
        return connection


def create_topic_health(component):
    """Attach optional ROS telemetry, or return None without affecting child.

    Call after starting the actual command. ROS is imported only here. The
    bounded master probe avoids waiting for a master that has not been started.
    No ROS publisher (including rosout) is registered by this observer.
    """
    if component not in COMPONENTS:
        return None
    health = TopicHealth(component)
    try:
        import rospy
        master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                                          transport=_TimeoutTransport())
        code, _, _ = master.getUri('/field_topic_health_probe')
        if code != 1:
            return None
        from std_msgs.msg import String, UInt64
        normalized = health.component
        specs = []
        if normalized in ('udp', 'localization'):
            from morai_msgs.msg import EgoVehicleStatus, GPSMessage
            specs.extend((('comp', '/Competition_topic', EgoVehicleStatus),
                          ('gps', '/sensor/gps', GPSMessage)))
        if normalized == 'udp':
            from sensor_msgs.msg import Imu
            specs.extend((('imu', '/sensor/imu', Imu),
                          ('lidar', '/sensor/lidar1/packets', rospy.AnyMsg)))
        elif normalized == 'localization':
            from nav_msgs.msg import Odometry
            specs.extend((('global', '/localization/kinematic_state', Odometry),
                          ('smooth', '/localization/control_kinematic_state', Odometry)))
        elif normalized in ('bevfusion', 'tracking'):
            from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
            specs.append(('od', '/perception/object_recognition/detection/objects', DetectedObjects)
                         if normalized == 'bevfusion' else
                         ('trk', '/perception/object_recognition/tracking/objects', TrackedObjects))
        elif normalized == 'traffic':
            specs.extend((('tl', '/traffic_light_node/debug_runtime', String),
                          ('selection', '/traffic_light_node/debug_selection', String),
                          ('route', '/perception/traffic_light_recognition/route_group_id', UInt64)))
        elif normalized == 'sensing':
            from tf2_msgs.msg import TFMessage
            specs.append(('tf', '/tf_static', TFMessage))
        if not rospy.core.is_initialized():
            rospy.init_node('field_topic_health_' + normalized, anonymous=True,
                            disable_signals=True, disable_rosout=True)
        health._ros_now = rospy.Time.now
        for key, topic, message_type in specs:
            health._subscriptions.append(rospy.Subscriber(
                topic, message_type, lambda message, key=key: health.observe(key, message),
                queue_size=1, buff_size=262144))
        return health
    except Exception:
        health.close()
        return None
