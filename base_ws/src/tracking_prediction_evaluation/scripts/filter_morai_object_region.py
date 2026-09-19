#!/usr/bin/env python3
"""Copy a MORAI bag while retaining NPCs inside a configured XY region."""

import argparse
import copy
import json
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--xmin", type=float, required=True)
    parser.add_argument("--xmax", type=float, required=True)
    parser.add_argument("--ymin", type=float, required=True)
    parser.add_argument("--ymax", type=float, required=True)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def main():
    import rosbag

    args = parse_args()
    args.output_bag.parent.mkdir(parents=True, exist_ok=True)
    frames = 0
    retained_instances = 0
    removed_instances = 0
    retained_ids = set()
    removed_ids = set()

    with rosbag.Bag(str(args.input_bag), "r") as source, rosbag.Bag(
        str(args.output_bag), "w", compression="lz4"
    ) as target:
        for topic, message, record_time in source.read_messages():
            if topic == args.object_topic:
                message = copy.deepcopy(message)
                retained = []
                for actor in message.npc_list:
                    inside = (
                        args.xmin <= float(actor.position.x) <= args.xmax
                        and args.ymin <= float(actor.position.y) <= args.ymax
                    )
                    actor_id = int(getattr(actor, "unique_id", -1))
                    if inside:
                        retained.append(actor)
                        retained_instances += 1
                        retained_ids.add(actor_id)
                    else:
                        removed_instances += 1
                        removed_ids.add(actor_id)
                message.npc_list = retained
                message.num_of_npcs = len(retained)
                frames += 1
            target.write(topic, message, record_time)

    report = {
        "schema_version": 1,
        "input_bag": str(args.input_bag.resolve()),
        "output_bag": str(args.output_bag.resolve()),
        "object_topic": args.object_topic,
        "retained_region": {
            "xmin": args.xmin,
            "xmax": args.xmax,
            "ymin": args.ymin,
            "ymax": args.ymax,
        },
        "object_frames": frames,
        "retained_npc_instances": retained_instances,
        "removed_npc_instances": removed_instances,
        "retained_unique_ids": sorted(retained_ids),
        "removed_unique_ids": sorted(removed_ids),
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
