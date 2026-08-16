#!/usr/bin/env python3
"""Set one non-looping phase of the K-City competition route in Autoware."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node

from autoware_adapi_v1_msgs.msg import RoutePrimitive, RouteSegment
from autoware_adapi_v1_msgs.srv import ClearRoute, SetRoute


PHASES = {
    1: {
        "lane_ids": [
            200889, 200392, 200178, 200337, 200287, 200348, 200346,
            200291, 200292, 200305, 200841, 200950, 200310, 200308,
            200258, 200261, 200315, 200445,
        ],
        "goal": (2.410767879498, 109.926622194897, 28.420080105143,
                 0.618377239869, 0.785881409128),
    },
    2: {
        "lane_ids": [
            200445, 200311, 200968, 201095, 201098, 201096, 201087,
            201086, 200101, 200128, 200102, 200134, 200360, 201399,
            200088, 201401, 200183, 200037, 200471, 201071, 200182,
            200361, 200181, 200892,
        ],
        "goal": (-139.501593313, -442.328259018, 28.526676781,
                 0.509686828359, 0.860360004299),
    },
}


def call(node: Node, client, request, timeout: float = 20.0):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"service unavailable: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"service timeout: {client.srv_name}")
    return future.result()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", type=int, choices=sorted(PHASES))
    parser.add_argument(
        "--keep-current-route",
        action="store_true",
        help="do not call clear_route before setting this phase",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node(f"set_kcity_competition_route_phase_{args.phase}")
    try:
        if not args.keep_current_route:
            response = call(
                node,
                node.create_client(ClearRoute, "/api/routing/clear_route"),
                ClearRoute.Request(),
            )
            if not response.status.success:
                raise RuntimeError(
                    f"clear_route failed: {response.status.code} "
                    f"{response.status.message}"
                )

        phase = PHASES[args.phase]
        request = SetRoute.Request()
        request.header.stamp = node.get_clock().now().to_msg()
        request.header.frame_id = "map"
        request.option.allow_goal_modification = False

        x, y, z, qz, qw = phase["goal"]
        request.goal.position.x = x
        request.goal.position.y = y
        request.goal.position.z = z
        request.goal.orientation.z = qz
        request.goal.orientation.w = qw

        for lane_id in phase["lane_ids"]:
            segment = RouteSegment()
            segment.preferred = RoutePrimitive(id=lane_id, type="lane")
            segment.alternatives = []
            request.segments.append(segment)

        response = call(
            node,
            node.create_client(SetRoute, "/api/routing/set_route"),
            request,
        )
        if not response.status.success:
            raise RuntimeError(
                f"set_route failed: {response.status.code} "
                f"{response.status.message}"
            )

        lane_ids = phase["lane_ids"]
        print(
            f"phase {args.phase} set: {len(lane_ids)} segments, "
            f"{lane_ids[0]} -> {lane_ids[-1]}"
        )
        print("This script does not request autonomous driving.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
