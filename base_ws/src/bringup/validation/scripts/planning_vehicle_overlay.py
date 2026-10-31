#!/usr/bin/env python3
"""Read-only speed, front-wheel steering, throttle and brake command overlay."""
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
        self.speed = self.steer = self.throttle = self.brake = None
        self.speed_at = self.command_at = 0.0
        self.pub = rospy.Publisher('/planning/debug/vehicle_overlay', OverlayText, queue_size=1)
        status_topic = rospy.get_param('~vehicle_status_topic', '/Competition_topic')
        self.status_sub = rospy.Subscriber(status_topic, EgoVehicleStatus, self.on_status, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/ctrl_cmd', CtrlCmd, self.on_command, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)

    def on_status(self, msg):
        with self.lock:
            self.speed = math.hypot(msg.velocity.x, msg.velocity.y) * 3.6
            self.speed_at = time.monotonic()

    def on_command(self, msg):
        with self.lock:
            self.steer = math.degrees(msg.steering)
            self.throttle = msg.accel
            self.brake = msg.brake
            self.command_at = time.monotonic()

    def publish(self, _event):
        now = time.monotonic()
        with self.lock:
            speed = f'{self.speed:.1f}' if self.speed is not None and now - self.speed_at < 1.0 else '--'
            command_fresh = now - self.command_at < 1.0
            steer = f'{self.steer:+.1f}' if self.steer is not None and math.isfinite(self.steer) and command_fresh else '--'
            throttle = f'{self.throttle:.3f}' if self.throttle is not None and command_fresh else '--'
            brake = f'{self.brake:.3f}' if self.brake is not None and command_fresh else '--'
        msg = OverlayText()
        msg.width, msg.height = 300, 105
        msg.horizontal_alignment, msg.vertical_alignment = OverlayText.LEFT, OverlayText.BOTTOM
        msg.horizontal_distance, msg.vertical_distance = 12, 12
        msg.text_size, msg.font = 13, 'DejaVu Sans Mono'
        msg.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        msg.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
        msg.text = (
            f'{"Speed":<10}{speed} km/h\n'
            f'{"Steer":<10}{steer} °\n'
            f'{"Accel cmd":<10}{throttle}\n'
            f'{"Brake cmd":<10}{brake}'
        )
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('planning_vehicle_overlay')
    overlay = VehicleOverlay()
    rospy.spin()
