#!/usr/bin/env python3
"""Prepare full-start scene clock variants, without loading or controlling MORAI.

The native scene schema stores only tlList index/state/deltaTime. It does not
store phase durations. These variants change the saved deltaTime globally;
they do NOT shorten yellow, install a controller program, or certify how the
running Unity version interprets that saved clock. Verify live native states.
All states, ego start, actors, map data and other original fields are preserved.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTPUT_ROOT = ROOT / "artifacts/evaluation"
DEFAULT_OFFSETS = (0.0, 8.0, 16.0)


def numeric(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + " must be numeric")
    if not math.isfinite(value) or value < 0:
        raise ValueError(name + " must be finite and nonnegative")
    return float(value)


def validate_scene(scene):
    if not isinstance(scene, dict) or not isinstance(scene.get("egoVehicle"), dict):
        raise ValueError("Expected a native scene with egoVehicle")
    rows = scene.get("tlList")
    if not isinstance(rows, list) or not rows:
        raise ValueError("Expected nonempty native tlList")
    seen = set()
    for row in rows:
        if not isinstance(row, dict) or set(row) != {"index", "state", "deltaTime"}:
            raise ValueError("Only the inspected index/state/deltaTime schema is supported")
        for key in ("index", "state"):
            if isinstance(row[key], bool) or not isinstance(row[key], int) or row[key] < 0:
                raise ValueError("tlList " + key + " must be a nonnegative integer")
        if row["index"] in seen:
            raise ValueError("Duplicate native controller index")
        seen.add(row["index"])
        numeric(row["deltaTime"], "deltaTime")
    return scene


def shifted_scene(scene, offset_sec):
    """Modify only each existing saved clock; never infer controller identities."""
    validate_scene(scene)
    offset_sec = numeric(offset_sec, "offset_sec")
    if offset_sec > 20.0:
        raise ValueError("Use bounded clock offsets of at most 20 seconds")
    result = copy.deepcopy(scene)
    for row in result["tlList"]:
        row["deltaTime"] = numeric(row["deltaTime"], "deltaTime") + offset_sec
    return result


def sha(data):
    return hashlib.sha256(data).hexdigest()


def prepare(source, output, offsets=DEFAULT_OFFSETS, allowed_root=OUTPUT_ROOT):
    source = Path(source).expanduser().resolve(strict=True)
    raw = source.read_bytes()
    scene = validate_scene(json.loads(raw))
    offsets = tuple(numeric(value, "offset_sec") for value in offsets)
    if not 1 <= len(offsets) <= 12 or len(set(offsets)) != len(offsets):
        raise ValueError("Use 1..12 distinct bounded offsets")
    # Validate every variant before creating any output directory.
    variants = [shifted_scene(scene, offset) for offset in offsets]
    requested = Path(output).expanduser()
    output = requested.resolve()
    try:
        output.relative_to(Path(allowed_root).resolve())
    except ValueError as error:
        raise ValueError("Output must remain inside the fresh evaluation artifact tree") from error
    if requested.exists() or requested.is_symlink() or output.exists():
        raise FileExistsError("Output must be fresh; prior evidence is never overwritten")
    if output == Path(allowed_root).resolve():
        raise ValueError("Output must be a fresh child directory")
    output.mkdir(parents=True)
    rows = []
    for index, (offset, variant) in enumerate(zip(offsets, variants), start=1):
        filename = "full_start_clock_%02d.json" % index
        data = (json.dumps(variant, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        (output / filename).write_bytes(data)
        rows.append({"file": filename, "sha256": sha(data), "delta_time_added_sec": offset})
    receipt = {
        "schema_version": 1,
        "source_scene": str(source),
        "source_sha256": sha(raw),
        "source_preserved": source.read_bytes() == raw,
        "variants": rows,
        "only_changed_field": "tlList[*].deltaTime",
        "controller_index_to_mission_mapping": "not inferred",
        "native_phase_durations_in_scene": False,
        "shorter_period_applied": False,
        "simulator_loaded": False,
        "native_signal_SET": False,
        "ego_start_map_actors_world_time": "unchanged from the selected source",
        "limitations": [
            "No yellow or red/green duration fields exist in the inspected scene schema.",
            "Adding deltaTime changes only the stored clock, not controller state or phase program.",
            "Native readback and rendered images must establish actual phase/clock behavior after load.",
            "First-yellow and green-to-yellow encounters are not guaranteed by preparation alone.",
            "C physical-head synchronization is retained in the original program, not redefined here.",
        ],
    }
    (output / "preparation.json").write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")
    return receipt


def prepare_campaigns(base_yaml, scenario_dir, output, allowed_root=OUTPUT_ROOT):
    """Fresh native-stress plans/YAMLs; preserve the full original goal/route."""
    import yaml
    from control_traffic_light_short_cycles import GROUP_IDS, NATURAL_CYCLES, validate_plan

    base_yaml = Path(base_yaml).expanduser().resolve(strict=True)
    base_bytes = base_yaml.read_bytes()
    base = yaml.safe_load(base_bytes)
    if not isinstance(base, dict) or not isinstance(base.get('default_case'), dict):
        raise ValueError('A full-course validator campaign YAML is required')
    scenario_dir = Path(scenario_dir).expanduser().resolve(strict=True)
    scenarios = [scenario_dir / ('full_start_clock_%02d.json' % index) for index in (1, 2, 3)]
    for source in scenarios:
        validate_scene(json.loads(source.read_bytes()))
    requested = Path(output).expanduser()
    output = requested.resolve()
    output.relative_to(Path(allowed_root).resolve())
    if requested.exists() or requested.is_symlink() or output.exists():
        raise FileExistsError('Campaign output must be fresh')
    if output == Path(allowed_root).resolve():
        raise ValueError('A fresh child output directory is required')
    records = []
    prepared = []
    for index, (source, yellow) in enumerate(zip(scenarios, (2., 3., 2.)), start=1):
        doc = copy.deepcopy(base)
        doc['campaign_name'] = 'traffic_lanelet_yellow_short_%02d' % index
        doc['scenarios'] = dict(directory=str(scenario_dir), glob='*.json', repeat=1, include=[source.name])
        doc['stack'].update(pose_source='localization', detector='od_gt', traffic_light=True,
                            traffic_light_profile='v5_candidate', planviz=True,
                            traffic_light_yellow_passthrough=True,
                            signal_compliance_yellow_deceleration_mps2=8.0,
                            enable_signal_compliance_guard=True,
                            signal_compliance_unknown_grace_sec=.3,
                            preserve_unreadable_color_evidence=True,
                            planner_compliance_observations=True,
                            stamped_vote_window_size=0,
                            signal_compliance_required_green_observations=1)
        recording = doc['recording']
        excluded = ('/perception/traffic_light_recognition/map_rois',
                    '/sensor/camera/front/compressed', '/diagnostics')
        recording.update(bag_enabled=True, include_sensors=False, minimum_free_disk_gib=.8,
                         selected_topics=[topic for topic in recording['selected_topics']
                                          if topic not in excluded and not topic.startswith('/evaluation/')],
                         video=dict(enabled=True, display=':0', fps=10, focus_planviz=False,
                                    window_layout='split', bottom_bar_px=100))
        recording.pop('analysis_mode', None)
        doc['upload']['enabled'] = False
        doc['resume'] = False
        if doc['default_case'] != base['default_case']:
            raise AssertionError('Full course goal/waypoints must remain unchanged')
        group_rows = []
        offsets = ((0, 3, 7, 11), (5, 9, 2, 15), (12, 1, 15, 4))[index - 1]
        for group_index, (group_id, colors) in enumerate(NATURAL_CYCLES.items()):
            normal = 8. if group_id == 201326 else float(8 + ((index + group_index) % 3) * 2)
            group_rows.append(dict(group_id=group_id, native_ids=list(GROUP_IDS[group_id]),
                                   phase_offset_sec=offsets[group_index],
                                   phases=[dict(color=color, seconds=yellow if color in (4, 5) else normal)
                                           for color in colors]))
        plan = validate_plan(dict(schema_version=1, name='full_start_short_%02d' % index,
                                  groups=group_rows))
        config_name, plan_name = 'run_%02d.yaml' % index, 'short_cycles_%02d.json' % index
        prepared.append((config_name, yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode()))
        prepared.append((plan_name, (json.dumps(plan, indent=2) + '\n').encode()))
        records.append(dict(config=config_name, native_plan=plan_name, scene=source.name,
                            scene_sha256=sha(source.read_bytes()), requested_yellow_seconds=yellow))
    output.mkdir(parents=True)
    for filename, data in prepared:
        (output / filename).write_bytes(data)
    receipt = dict(schema_version=1, base_yaml=str(base_yaml), base_sha256=sha(base_bytes),
                   source_preserved=base_yaml.read_bytes() == base_bytes,
                   original_goal_waypoints_preserved=True, campaigns=records,
                   minimum_free_disk_gib=.8, executed=False,
                   shortened_phases='native UDP stress plans only; requires the explicit wrapper driver',
                   recording_stop='wrapper must stop after D; original route is not shortened')
    (output / 'preparation.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--source", type=Path)
    mode.add_argument("--base-yaml", type=Path)
    parser.add_argument("--scenario-dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--offsets-sec", type=float, nargs="+", default=DEFAULT_OFFSETS)
    args = parser.parse_args()
    if args.base_yaml is not None:
        if args.scenario_dir is None:
            parser.error('--base-yaml requires --scenario-dir')
        receipt = prepare_campaigns(args.base_yaml, args.scenario_dir, args.output)
    else:
        receipt = prepare(args.source, args.output, args.offsets_sec)
    print(json.dumps(receipt, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
