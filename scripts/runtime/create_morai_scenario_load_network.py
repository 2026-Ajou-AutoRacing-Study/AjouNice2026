#!/usr/bin/env python3
"""Create a complete MORAI ROS ScenarioLoad network profile.

MORAI's Network File Manager stores one configuration entry per interface.
The AJOUNICE profile already contains the MORAISLSubscriber entry (type 783),
but the Ego P/S network is configured for UDP and therefore cannot receive
``/ScenarioLoad``. This utility writes a *new* profile that sends Ego status,
objects, collision data, and ScenarioLoad over ROS, and also preserves a
usable Simulator Network ROS entry. The UDP MoraiCmdController ports remain
untouched.
"""

import argparse
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict


DEFAULT_NETWORK_DIR = (
    Path.home()
    / "Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network"
    / "25.S4.MolitComp03"
)
# ``NetworkInfo...`` is MORAI's active, mutable file. It can be rewritten
# with an incomplete UI state when a connection attempt is cancelled. The
# team's original network profile is the stable source for a fresh profile.
DEFAULT_SOURCE = DEFAULT_NETWORK_DIR / "ego_network_1.json"
DEFAULT_OUTPUT = DEFAULT_NETWORK_DIR / "ajounice_full_ros_scenario_load.json"
MORAI_SCENARIO_LOAD_SUBSCRIBER = 783
ROS_TELEMETRY_TYPES = (261, 295, 770, 772)


def ego_profile(document: Dict[str, Any]) -> Dict[str, Any]:
    for ego in document.get("egoNetworkData", []):
        if int(ego.get("UNIQUEID", -1)) == 0:
            return ego
    raise ValueError("Ego-0 network profile was not found in the source profile")


def simulator_profile(document: Dict[str, Any]) -> Dict[str, Any]:
    for profile in document.get("egoNetworkData", []):
        if int(profile.get("UNIQUEID", -1)) == -100:
            return profile
    raise ValueError("Simulator Network profile was not found in the source profile")


def scenario_load_entry(document: Dict[str, Any]) -> Dict[str, Any]:
    for entry in ego_profile(document).get("listEgoNetworkInfo", []):
        if int(entry.get("PUBSUB_TYPE", -1)) == MORAI_SCENARIO_LOAD_SUBSCRIBER:
            return entry
    raise ValueError(
        "MORAISLSubscriber (PUBSUB_TYPE 783) was not found in the source profile"
    )


def configure_scenario_load(
    document: Dict[str, Any], bridge_host: str, bridge_port: int
) -> None:
    # Select ROS for the Ego P/S network.  EgoCtrlConfig is intentionally not
    # touched: it remains on the existing UDP MoraiCmdController ports.
    ego = ego_profile(document)
    ego["connectedType"] = 3

    for entry in ego.get("listEgoNetworkInfo", []):
        if int(entry.get("PUBSUB_TYPE", -1)) not in ROS_TELEMETRY_TYPES:
            continue
        config = entry.setdefault("config", {})
        ros_config = config.setdefault("rosConfig", {})
        entry["netType"] = 3
        config["commType"] = 3
        ros_config.update(
            {
                "handlerType": 0,
                "ConnectionTimeout": 1000,
                "RosBridgeServerUrl": "ws://{}:{}".format(
                    bridge_host, bridge_port
                ),
                "RosBridgeIP": bridge_host,
                "RosBridgePort": bridge_port,
                "RosMessageOnOff": True,
            }
        )

    entry = scenario_load_entry(document)
    config = entry.setdefault("config", {})
    ros_config = config.setdefault("rosConfig", {})

    # MORAI enum values: 3 = ROS, handler type 1 = subscriber.
    entry["netType"] = 3
    entry["Division_TYPE"] = 2
    config["commType"] = 3
    ros_config.update(
        {
            "handlerType": 1,
            "ConnectionTimeout": 1000,
            "RosBridgeServerUrl": "ws://{}:{}".format(bridge_host, bridge_port),
            "RosBridgeIP": bridge_host,
            "RosBridgePort": bridge_port,
            "Topic": "/ScenarioLoad",
            "RosMessageOnOff": True,
        }
    )

    # The Simulator Network has its own connection selector. Leaving it in
    # UDP (or empty) makes MORAI's profile loader wait indefinitely for a
    # network that will never connect. MultiEgoTransformHandler is a ROS
    # subscriber in the working team configuration, so it uses this bridge.
    simulator = simulator_profile(document)
    simulator["connectedType"] = 3
    simulator_entries = simulator.get("listEgoNetworkInfo", [])
    if not simulator_entries:
        raise ValueError("Simulator Network profile has no connection entries")
    for simulator_entry in simulator_entries:
        simulator_config = simulator_entry.setdefault("config", {})
        simulator_ros = simulator_config.setdefault("rosConfig", {})
        simulator_entry["netType"] = 3
        simulator_config["commType"] = 3
        simulator_ros.update(
            {
                "handlerType": 1,
                "ConnectionTimeout": 1000,
                "RosBridgeServerUrl": "ws://{}:{}".format(
                    bridge_host, bridge_port
                ),
                "RosBridgeIP": bridge_host,
                "RosBridgePort": bridge_port,
                "Topic": "/ego_setting",
                "RosMessageOnOff": True,
            }
        )


def write_json_new(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(
            "refusing to overwrite existing profile: {}".format(path)
        )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary_path, path)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a non-destructive MORAI ROS /ScenarioLoad profile."
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--bridge-host", default="192.168.0.5")
    parser.add_argument("--bridge-port", type=int, default=9090)
    args = parser.parse_args()
    if not args.bridge_host.strip():
        parser.error("--bridge-host must not be empty")
    if not 1 <= args.bridge_port <= 65535:
        parser.error("--bridge-port must be between 1 and 65535")
    return args


def main() -> int:
    args = parse_args()
    try:
        with args.source.open("r", encoding="utf-8-sig") as stream:
            source = json.load(stream)
        if not isinstance(source, dict):
            raise ValueError("source profile root is not a JSON object")
        configure_scenario_load(source, args.bridge_host, args.bridge_port)
        write_json_new(args.output, source)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print("ERROR: {}".format(error))
        return 1

    print("Created MORAI ScenarioLoad profile: {}".format(args.output))
    print(
        "Ego telemetry and MORAISLSubscriber use ROS topic /ScenarioLoad via "
        "ws://{}:{}; UDP MoraiCmdController is unchanged".format(
            args.bridge_host, args.bridge_port
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
