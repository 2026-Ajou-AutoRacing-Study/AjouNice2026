#!/usr/bin/env python3
"""Control MORAI Direct Rosbag Replay through ``/Morai_SimProc``.

MORAI only accepts replay options after the simulator has entered Direct
Rosbag Replay mode.  This utility deliberately checks that precondition so a
batch job cannot silently send commands that the simulator ignores.
"""

from __future__ import annotations

import argparse
import os
import threading
import sys
import time
from pathlib import Path
from typing import Iterable, Tuple


SIM_STATUS_PLAY = 0x01
SIM_STATUS_PAUSE = 0x10
SIM_STATUS_STOP = 0x20

REPLAY_MODE = 0x10
REPLAY_STATUS_PLAYING = 0x01
REPLAY_STATUS_PAUSED = 0x10
REPLAY_STATUS_EOF = 0x20

OPTION_LOAD = 0x0001
OPTION_TARGET = 0x0010
OPTION_START_TIME = 0x0100
OPTION_SPEED = 0x1000

TARGET_BITS = {
    "ego": 0x0001,
    "npc": 0x0010,
    "pedestrian": 0x0100,
}

DEFAULT_SERVICE = "/Morai_SimProc"
DEFAULT_STATUS_TOPIC = "/sim/process/state/msg/MoraiSimProcStatus"
DEFAULT_ROSBAG_DIR = Path(
    "~/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Rosbag"
).expanduser()


def parse_targets(value: str) -> int:
    names = [item.strip().lower() for item in value.split(",") if item.strip()]
    if not names:
        raise ValueError("at least one replay target is required")
    unknown = sorted(set(names) - set(TARGET_BITS))
    if unknown:
        raise ValueError(
            "unknown replay target(s): "
            + ", ".join(unknown)
            + "; choose from ego,npc,pedestrian"
        )
    bits = 0
    for name in names:
        bits |= TARGET_BITS[name]
    return bits


def build_replay_option(*, start_time_ms: int, replay_speed: int) -> int:
    option = OPTION_LOAD | OPTION_TARGET
    if start_time_ms:
        option |= OPTION_START_TIME
    if replay_speed != 1:
        option |= OPTION_SPEED
    return option


def resolve_bag(value: str, rosbag_dir: Path) -> Tuple[Path, str]:
    requested = Path(value).expanduser()
    candidates = []
    if requested.is_absolute() or requested.parent != Path("."):
        candidates.append(requested.resolve())
    candidates.append((rosbag_dir / requested.name).resolve())

    for candidate in candidates:
        if candidate.is_file():
            # MORAI's service resolves files inside SaveFile/Rosbag by name.
            return candidate, candidate.name
    checked = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(f"replay bag not found; checked: {checked}")


def mode_name(value: int) -> str:
    return {0x00: "unreported", 0x01: "simulation", REPLAY_MODE: "replay"}.get(
        value, f"unknown({value})"
    )


def status_name(value: int) -> str:
    return {
        0x00: "unreported",
        REPLAY_STATUS_PLAYING: "playing",
        REPLAY_STATUS_PAUSED: "paused",
        REPLAY_STATUS_EOF: "eof",
    }.get(value, f"unknown({value})")


def command_result_name(value: int) -> str:
    return {
        0x00: "initial",
        0x01: "success",
        0x10: "command_failed",
        0x20: "bag_load_failed",
        0x30: "bag_not_loaded",
    }.get(value, f"unknown({value})")


def format_status(message) -> str:
    return (
        f"mode={mode_name(int(message.current_mode))}({int(message.current_mode)}) "
        f"status={status_name(int(message.current_status))}({int(message.current_status)}) "
        f"command={command_result_name(int(message.command_result))}"
        f"({int(message.command_result)})"
    )


def import_ros():
    try:
        import rospy
        from morai_msgs.msg import MoraiSimProcStatus
        from morai_msgs.srv import MoraiSimProcSrv, MoraiSimProcSrvRequest
    except ImportError as exc:
        raise RuntimeError(
            "ROS imports failed. Source /opt/ros/noetic/setup.bash, "
            "base_ws/devel/setup.bash and morai_personal_ws/devel/setup.bash."
        ) from exc
    return rospy, MoraiSimProcStatus, MoraiSimProcSrv, MoraiSimProcSrvRequest


def wait_for_status(rospy, status_type, topic: str, timeout: float):
    try:
        return rospy.wait_for_message(topic, status_type, timeout=timeout)
    except Exception as exc:
        raise RuntimeError(f"no MORAI replay status on {topic} within {timeout:.1f}s") from exc


def require_replay_mode(status, *, allow_unreported_mode: bool = False) -> None:
    if int(status.current_mode) != REPLAY_MODE:
        if allow_unreported_mode and int(status.current_mode) == 0:
            print(
                "warning: MORAI publishes current_mode=0 even though the UI may "
                "be in Direct Replay; probing the service as requested",
                file=sys.stderr,
                flush=True,
            )
            return
        raise RuntimeError(
            "MORAI is not in Direct Rosbag Replay mode. In MORAI select "
            "PlayMode > Replay > Rosbag Replay, load any compatible bag, "
            "press Start to open the Replayer Control Panel, and leave it paused."
        )


def make_request(request_type, *, sim_status: int, replay_option: int = 0,
                 bag_name: str = "", target_bits: int = 0,
                 replay_speed: int = 1, start_time_ms: int = 0):
    request = request_type()
    request.request.sim_process_status = sim_status
    request.request.replay_option = replay_option
    request.request.rosbag_file_name = bag_name
    request.request.replay_target_option = target_bits
    request.request.replay_speed = replay_speed
    request.request.start_time = start_time_ms
    return request


def call_service(rospy, service_type, request, service_name: str, timeout: float):
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception as exc:
        raise RuntimeError(f"MORAI service {service_name} unavailable") from exc
    proxy = rospy.ServiceProxy(service_name, service_type, persistent=False)
    result = []
    errors = []

    def invoke():
        try:
            result.append(proxy(request))
        except BaseException as exc:  # Propagate rospy transport errors below.
            errors.append(exc)

    worker = threading.Thread(target=invoke, daemon=True)
    worker.start()
    worker.join(timeout)
    if worker.is_alive():
        proxy.close()
        raise RuntimeError(
            f"MORAI service {service_name} accepted the ROS connection but did "
            f"not return a response within {timeout:.1f}s. The request may still "
            "be queued inside MORAI: do not retry it immediately. Wait for the "
            "Replay UI to settle, then reconnect Simulator Network if no action "
            "occurs."
        )
    if errors:
        raise RuntimeError(f"MORAI service call failed: {errors[0]}") from errors[0]
    response = result[0]
    if not bool(response.response.result):
        raise RuntimeError(f"MORAI rejected command through {service_name}")
    return response


def monitor_until_eof(rospy, status_type, topic: str, timeout: float,
                      poll_timeout: float = 2.0) -> None:
    started = time.monotonic()
    last_line = None
    while not rospy.is_shutdown():
        if timeout > 0 and time.monotonic() - started > timeout:
            raise RuntimeError(f"replay did not reach EOF within {timeout:.1f}s")
        status = wait_for_status(rospy, status_type, topic, poll_timeout)
        line = format_status(status)
        if line != last_line:
            print(line, flush=True)
            last_line = line
        if int(status.current_status) == REPLAY_STATUS_EOF:
            print("direct replay reached EOF", flush=True)
            return
        if int(status.command_result) in (0x10, 0x20, 0x30):
            raise RuntimeError(f"MORAI replay command failed: {line}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default=DEFAULT_SERVICE)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--status-timeout-sec", type=float, default=5.0)
    parser.add_argument(
        "--service-timeout-sec",
        type=float,
        default=60.0,
        help="wait for MORAI service response; Direct Replay file load can take ~30s",
    )
    parser.add_argument(
        "--allow-unreported-mode",
        action="store_true",
        help=(
            "allow a service probe when MORAI publishes current_mode=0; use only "
            "after manually opening and pausing Direct Rosbag Replay"
        ),
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="print current MORAI mode and replay status")

    play = subparsers.add_parser("play", help="load and play a Direct Replay bag")
    play.add_argument("--bag", required=True)
    play.add_argument("--rosbag-dir", type=Path, default=DEFAULT_ROSBAG_DIR)
    play.add_argument("--targets", default="ego,npc,pedestrian")
    play.add_argument("--speed", type=int, default=1)
    play.add_argument("--start-time-ms", type=int, default=0)
    play.add_argument("--wait-for-eof", action="store_true")
    play.add_argument("--eof-timeout-sec", type=float, default=0.0)

    for command in ("pause", "resume", "stop"):
        subparsers.add_parser(command)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        rospy, status_type, service_type, request_type = import_ros()
        rospy.init_node("morai_direct_replay_runner", anonymous=True, disable_signals=True)
        status = wait_for_status(
            rospy, status_type, args.status_topic, args.status_timeout_sec
        )
        print(format_status(status), flush=True)

        if args.command == "status":
            return 0

        require_replay_mode(
            status, allow_unreported_mode=args.allow_unreported_mode
        )

        if args.command == "play":
            if args.speed <= 0:
                raise ValueError("--speed must be positive")
            if args.start_time_ms < 0:
                raise ValueError("--start-time-ms must be non-negative")
            bag_path, bag_name = resolve_bag(args.bag, args.rosbag_dir.expanduser())
            target_bits = parse_targets(args.targets)
            replay_option = build_replay_option(
                start_time_ms=args.start_time_ms, replay_speed=args.speed
            )
            print(
                f"loading direct replay: path={bag_path} service_name={bag_name} "
                f"targets=0x{target_bits:04x} option=0x{replay_option:04x} "
                f"speed={args.speed} start_ms={args.start_time_ms}",
                flush=True,
            )
            request = make_request(
                request_type,
                sim_status=SIM_STATUS_PLAY,
                replay_option=replay_option,
                bag_name=bag_name,
                target_bits=target_bits,
                replay_speed=args.speed,
                start_time_ms=args.start_time_ms,
            )
        else:
            sim_status = {
                "pause": SIM_STATUS_PAUSE,
                "resume": SIM_STATUS_PLAY,
                "stop": SIM_STATUS_STOP,
            }[args.command]
            request = make_request(request_type, sim_status=sim_status)

        call_service(
            rospy, service_type, request, args.service, args.service_timeout_sec
        )
        print(f"MORAI accepted {args.command} command", flush=True)

        # Some MORAI launcher builds finish OPTION_LOAD but stay paused even
        # when SIM_STATUS_PLAY was included in the load request. Send exactly
        # one separate resume only after the load call has returned. A timed-out
        # load must never be retried immediately because MORAI may execute it
        # later from its internal queue.
        if args.command == "play":
            resume_request = make_request(
                request_type, sim_status=SIM_STATUS_PLAY
            )
            call_service(
                rospy,
                service_type,
                resume_request,
                args.service,
                args.service_timeout_sec,
            )
            print("MORAI accepted post-load resume command", flush=True)

        if args.command == "play" and args.wait_for_eof:
            monitor_until_eof(
                rospy,
                status_type,
                args.status_topic,
                timeout=args.eof_timeout_sec,
            )
        else:
            updated = wait_for_status(
                rospy, status_type, args.status_topic, args.status_timeout_sec
            )
            print(format_status(updated), flush=True)
        return 0
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
