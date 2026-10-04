#!/usr/bin/env python3
"""Generate a minimal ROS Ego Network profile for the legacy MR launcher.

The MR S4 launcher becomes progressively slower in the full 23-entry ROS Ego
Network configuration while the same simulator remains stable with the UDP
profile.  This probe retains only the interfaces required by the rolling
scenario runner.  It never modifies the source profile.
"""

import argparse
import json
from pathlib import Path


KEEP_PUBSUB_TYPES = {
    770: "/Ego_topic",
    772: "/Object_topic",
    783: "/ScenarioLoad",
    780: "/Service_MoraiEventCmd",
}
CTRL_PUBSUB_TYPE = 771
CTRL_TOPIC = "/ctrl_cmd"


def parse_args():
    home = Path.home()
    default_source = (
        home
        / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network"
        / "25.S4.MolitComp03/ss_ego_only.json"
    )
    default_install = default_source.parent / "ss_ego_minimal_hybrid.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=default_source)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("configs/network/ego_network_mr_minimal_hybrid.json"),
    )
    parser.add_argument("--install", type=Path, default=default_install)
    return parser.parse_args()


def main():
    args = parse_args()
    payload = json.loads(args.source.read_text())
    records = payload.get("egoNetworkData")
    if not isinstance(records, list):
        raise ValueError("source has no egoNetworkData list")
    ego_records = [record for record in records if record.get("UNIQUEID") == 0]
    if len(ego_records) != 1:
        raise ValueError("expected exactly one UNIQUEID=0 record")

    ego = ego_records[0]
    source_entries = ego.get("listEgoNetworkInfo")
    if not isinstance(source_entries, list):
        raise ValueError("UNIQUEID=0 has no listEgoNetworkInfo list")

    selected = []
    observed = {}
    for entry in source_entries:
        pubsub_type = entry.get("PUBSUB_TYPE")
        config = entry.get("config") or {}
        ego_ctrl = entry.get("EgoCtrlConfig") or {}
        if pubsub_type == CTRL_PUBSUB_TYPE:
            topic = (ego_ctrl.get("rosConfig") or {}).get("Topic")
            if ego_ctrl.get("commType") != 3 or topic != CTRL_TOPIC:
                raise ValueError(
                    "CtrlCmd entry is not ROS /ctrl_cmd: commType={!r}, topic={!r}"
                    .format(ego_ctrl.get("commType"), topic)
                )
            selected.append(entry)
            observed[CTRL_TOPIC] = pubsub_type
        elif pubsub_type in KEEP_PUBSUB_TYPES:
            expected_topic = KEEP_PUBSUB_TYPES[pubsub_type]
            topic = (config.get("rosConfig") or {}).get("Topic")
            if config.get("commType") != 3 or topic != expected_topic:
                raise ValueError(
                    "entry {} is not ROS {}: commType={!r}, topic={!r}"
                    .format(
                        pubsub_type,
                        expected_topic,
                        config.get("commType"),
                        topic,
                    )
                )
            selected.append(entry)
            observed[expected_topic] = pubsub_type

    required = {CTRL_TOPIC, *KEEP_PUBSUB_TYPES.values()}
    missing = sorted(required - set(observed))
    if missing:
        raise ValueError("source is missing required interfaces: {}".format(missing))
    if len(selected) != len(required):
        raise ValueError(
            "expected {} selected entries, got {}".format(len(required), len(selected))
        )

    ego["connectedType"] = 3
    ego["listEgoNetworkInfo"] = selected
    output_payload = {"egoNetworkData": [ego]}
    rendered = json.dumps(output_payload, ensure_ascii=False, indent=2) + "\n"

    for destination in (args.output, args.install):
        destination = destination.expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(rendered)
        print("wrote {}".format(destination))

    print("interfaces:")
    for topic in sorted(required):
        print("  {}".format(topic))


if __name__ == "__main__":
    main()
