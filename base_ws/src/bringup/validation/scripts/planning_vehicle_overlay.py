#!/usr/bin/env python3
"""Read-only speed and commanded brake overlay for planning RViz."""
import math
import time
import threading

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA


class VehicleOverlay:
    def __init__(self):
        self.lock = threading.Lock()
        self.speed = self.brake = None
        self.speed_at = self.brake_at = 0.0
        self.pub = rospy.Publisher('/planning/debug/vehicle_overlay', OverlayText, queue_size=1)
        self.ego_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.on_ego, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/ctrl_cmd', CtrlCmd, self.on_command, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)

    def on_ego(self, msg):
        with self.lock:
            self.speed = math.hypot(msg.velocity.x, msg.velocity.y) * 3.6
            self.speed_at = time.monotonic()

    def on_command(self, msg):
        with self.lock:
            self.brake = msg.brake
            self.brake_at = time.monotonic()

    def publish(self, _event):
        now = time.monotonic()
        with self.lock:
            speed = f'{self.speed:5.1f}' if self.speed is not None and now - self.speed_at < 1.0 else '   --'
            brake = f'{self.brake:.3f}' if self.brake is not None and now - self.brake_at < 1.0 else '--'
        msg = OverlayText()
        msg.width, msg.height = 250, 60
        msg.horizontal_alignment, msg.vertical_alignment = OverlayText.LEFT, OverlayText.BOTTOM
        msg.horizontal_distance, msg.vertical_distance = 12, 12
        msg.text_size, msg.font = 13, 'DejaVu Sans Mono'
        msg.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        msg.bg_color = ColorRGBA(0.04, 0.06, 0.08, 0.65)
        msg.text = f'Speed      {speed} km/h\nBrake cmd  {brake} / 1.0'
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('planning_vehicle_overlay')
    overlay = VehicleOverlay()
    rospy.spin()
