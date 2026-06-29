#!/usr/bin/env python3
import os
import socket
import struct

import rospy
import yaml
from sensor_msgs.msg import CompressedImage


MORAI_CAMERA_HEADER_BYTES = 19
MORAI_CAMERA_END_TAIL = b"EI"


def load_camera_config(path, camera_name):
    if not path:
        return {}

    config_path = os.path.abspath(os.path.expanduser(path))
    with open(config_path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}

    cameras = data.get("cameras", {})
    if camera_name not in cameras:
        raise KeyError("camera %r not found in %s" % (camera_name, config_path))
    return cameras[camera_name] or {}


class MoraiCameraUdpReceiver:
    def __init__(self):
        camera_config = rospy.get_param("~camera_config", "")
        camera_name = rospy.get_param("~camera_name", "front")
        self.config = load_camera_config(camera_config, camera_name)

        self.bind_ip = self.param("bind_ip", "0.0.0.0")
        self.udp_port = int(self.param("udp_port", 1232))
        self.topic = self.param("topic", "/sensor/camera")
        self.frame_id = self.param("frame_id", "Camera-2")
        self.image_format = self.param("image_format", "jpeg")
        self.max_packet_size = int(self.param("max_packet_size", 65000))
        self.max_frame_bytes = int(self.param("max_frame_bytes", 8 * 1024 * 1024))
        self.receive_buffer_bytes = int(self.param("receive_buffer_bytes", 4 * 1024 * 1024))
        self.socket_timeout_sec = float(self.param("socket_timeout_sec", 0.5))
        self.warn_on_timeout = self.bool_param("warn_on_timeout", False)

        self.publisher = rospy.Publisher(self.topic, CompressedImage, queue_size=2)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.receive_buffer_bytes)
        self.socket.bind((self.bind_ip, self.udp_port))
        self.socket.settimeout(self.socket_timeout_sec)

        rospy.loginfo(
            "MORAI camera UDP receiver: %s:%d -> %s frame_id=%s",
            self.bind_ip,
            self.udp_port,
            self.topic,
            self.frame_id,
        )

    def param(self, name, default):
        return rospy.get_param("~" + name, self.config.get(name, default))

    def bool_param(self, name, default):
        value = self.param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def parse_packet(self, packet):
        if len(packet) < MORAI_CAMERA_HEADER_BYTES + 2:
            return None

        unit_idx = struct.unpack("<i", packet[11:15])[0]
        unit_size = struct.unpack("<i", packet[15:19])[0]
        unit_data = packet[MORAI_CAMERA_HEADER_BYTES:-2]
        unit_tail = packet[-2:]
        return unit_idx, unit_size, unit_data, unit_tail

    def publish_frame(self, frame_bytes):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.format = self.image_format
        msg.data = bytes(frame_bytes)
        self.publisher.publish(msg)

    def spin(self):
        frame = bytearray()
        packet_count = 0

        while not rospy.is_shutdown():
            try:
                packet, _sender = self.socket.recvfrom(self.max_packet_size)
            except socket.timeout:
                if frame:
                    rospy.logwarn_throttle(
                        2.0,
                        "Dropping incomplete MORAI camera frame: %d bytes, %d packets",
                        len(frame),
                        packet_count,
                    )
                    frame = bytearray()
                    packet_count = 0
                elif self.warn_on_timeout:
                    rospy.logwarn_throttle(5.0, "No MORAI camera UDP packets on port %d", self.udp_port)
                continue

            parsed = self.parse_packet(packet)
            if parsed is None:
                rospy.logwarn_throttle(2.0, "Skipping short MORAI camera UDP packet: %d bytes", len(packet))
                continue

            _unit_idx, unit_size, unit_data, unit_tail = parsed

            if unit_size > 0 and unit_size != len(unit_data):
                rospy.logdebug(
                    "MORAI camera UDP size field differs from payload: field=%d payload=%d",
                    unit_size,
                    len(unit_data),
                )

            if len(frame) + len(unit_data) > self.max_frame_bytes:
                rospy.logwarn(
                    "Dropping oversized MORAI camera frame over %d bytes",
                    self.max_frame_bytes,
                )
                frame = bytearray()
                packet_count = 0
                continue

            frame.extend(unit_data)
            packet_count += 1

            if unit_tail == MORAI_CAMERA_END_TAIL:
                self.publish_frame(frame)
                frame = bytearray()
                packet_count = 0


def main():
    rospy.init_node("morai_camera_udp_receiver")
    receiver = MoraiCameraUdpReceiver()
    receiver.spin()


if __name__ == "__main__":
    main()
