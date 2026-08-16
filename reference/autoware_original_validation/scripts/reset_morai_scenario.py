#!/usr/bin/env python3
"""Publish the verified MORAI competition scenario-load request once."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from morai_msgs.msg import ScenarioLoad


def main() -> None:
    rclpy.init()
    node = Node("reset_morai_competition_scenario")
    publisher = node.create_publisher(ScenarioLoad, "/ScenarioLoad", 10)
    try:
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                raise RuntimeError("no /ScenarioLoad subscriber within 5 seconds")

        message = ScenarioLoad()
        message.file_name = "2026_molit_comp_sample_scene"
        message.load_network_connection_data = False
        message.delete_all = True
        message.load_ego_vehicle_data = True
        message.load_surrounding_vehicle_data = True
        message.load_pedestrian_data = True
        message.load_obstacle_data = True
        message.set_pause = False
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=1.0)
        print("scenario load request published")
        print("Reconnect both MORAI network tabs to ROS 127.0.0.1:9090.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
