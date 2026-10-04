#!/usr/bin/env python3
"""Live progress monitor for MORAI highway random-spawn birth events."""

import argparse
import json
import os
import signal
import time
from collections import Counter
from pathlib import Path


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--target-births", type=int, default=1000)
    parser.add_argument("--max-spawn-lateral-distance-m", type=float, default=15.0)
    parser.add_argument("--reappearance-gap-sec", type=float, default=2.0)
    parser.add_argument("--progress-json", type=Path, required=True)
    parser.add_argument("--print-every", type=int, default=25)
    parser.add_argument("--stop-pid", type=int, help="send SIGINT when the target is reached")
    return parser.parse_args()


def main():
    args = arguments()
    import rospy
    from morai_msgs.msg import ObjectStatusList

    scenario = json.loads(args.scenario.read_text(encoding="utf-8"))
    catalog = json.loads(args.catalog.read_text(encoding="utf-8"))
    spawns = {
        int(row["UNIQUEID"]): (float(row["pos"]["x"]), float(row["pos"]["y"]))
        for row in scenario["spawnPointList"]
        if int(row["UNIQUEID"]) in (6, 8, 10)
    }
    families = {row["name"]: row["subtype"] for row in catalog["vehicles"]}
    active_previous = set()
    last_seen = {}
    names = Counter()
    family_counts = Counter()
    spawn_counts = Counter()
    rejected = 0
    started = time.time()

    args.progress_json.parent.mkdir(parents=True, exist_ok=True)

    def save():
        payload = {
            "target_births": args.target_births,
            "accepted_births": sum(names.values()),
            "rejected_births": rejected,
            "unique_models": len(names),
            "elapsed_sec": time.time() - started,
            "name_counts": dict(names),
            "family_counts": dict(family_counts),
            "spawn_counts": {str(key): value for key, value in spawn_counts.items()},
        }
        temporary = args.progress_json.with_suffix(args.progress_json.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(args.progress_json)

    def callback(message):
        nonlocal active_previous, rejected
        now = rospy.get_time()
        active_now = set()
        changed = False
        for actor in message.npc_list:
            uid = int(actor.unique_id)
            active_now.add(uid)
            previous = last_seen.get(uid)
            birth = uid not in active_previous and (
                previous is None or now - previous >= args.reappearance_gap_sec
            )
            last_seen[uid] = now
            if not birth:
                continue
            x, y = float(actor.position.x), float(actor.position.y)
            # Object_topic may first expose a spawned NPC only after it has
            # travelled far downstream.  The sample highway is longitudinal
            # in map y, while spawn IDs 6/8/10 occupy distinct lateral x
            # positions.  Match by lateral x and reject the roundabout route.
            spawn_id, distance = min(
                ((key, abs(x - pos[0])) for key, pos in spawns.items()),
                key=lambda item: item[1],
            )
            if distance > args.max_spawn_lateral_distance_m:
                rejected += 1
                changed = True
                continue
            name = str(actor.name).strip() or "<empty>"
            names[name] += 1
            family_counts[families.get(name, "unknown")] += 1
            if abs(y - spawns[spawn_id][1]) <= 50.0:
                spawn_counts[spawn_id] += 1
            else:
                spawn_counts["downstream_unassigned"] += 1
            changed = True
            total = sum(names.values())
            if total == 1 or total % args.print_every == 0:
                elapsed = time.time() - started
                rate = total / elapsed * 60.0 if elapsed else 0.0
                eta = (args.target_births - total) / rate if rate else float("inf")
                print(
                    "births={}/{} unique_models={} rate={:.1f}/min eta={:.1f}min top={}".format(
                        total, args.target_births, len(names), rate, eta,
                        names.most_common(5),
                    ),
                    flush=True,
                )
            if total >= args.target_births:
                save()
                if args.stop_pid:
                    os.kill(args.stop_pid, signal.SIGINT)
                rospy.signal_shutdown("target births reached")
                break
        active_previous = active_now
        if changed:
            save()

    rospy.init_node("monitor_morai_highway_random_spawn_live", anonymous=True)
    rospy.Subscriber("/Object_topic", ObjectStatusList, callback, queue_size=50)
    save()
    rospy.spin()
    save()


if __name__ == "__main__":
    main()
