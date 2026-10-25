#!/usr/bin/env python3
"""Prepare, but never run, an object-free real-planner/localization trial.

Copies the existing full-course goal/waypoints without hand-picked shortcuts.
Original scenes, private workspace YAMLs, signal policies and defaults stay intact.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/full_course/base/sample_only.yaml'
sys.path.insert(0, str(ROOT / 'scripts/evaluation'))
from traffic_light_stationary_probe import normalized_scene


def prepare(output, models_dir, python):
    output = Path(output).resolve()
    output.relative_to(ROOT / 'artifacts/evaluation')
    if output.exists():
        raise FileExistsError('Use a fresh directory; old trial evidence is preserved')
    doc = yaml.safe_load(BASE.read_text())
    source = ROOT / doc['scenarios']['directory'] / doc['scenarios']['include'][0]
    scene = normalized_scene(json.loads(source.read_text()))
    doc['campaign_name'] = 'traffic_light_v5_no_objects'
    doc['scenarios'] = dict(directory=str(output / 'scenarios'), glob='*.json', repeat=1,
                            include=['no_objects.json'])
    doc['stack'].update(detector='od_gt', pose_source='localization', traffic_light=True,
                        traffic_light_profile='v5_candidate', traffic_light_runtime='virtualenv',
                        traffic_light_models_root=str(Path(models_dir).resolve()),
                        traffic_light_python=str(Path(python).expanduser().absolute()),
                        traffic_light_device='0', planviz=False)
    # All missions are still driven by the real planner/controller and real
    # localization; no forced group ID or native light state enters perception.
    profile = ROOT / doc['recording']['topic_profile']
    topics = json.loads(profile.read_text()) + [
        '/sensor/camera/front/compressed',
        '/perception/traffic_light_recognition/route_group_id',
        '/perception/traffic_light_recognition/map_rois',
        '/traffic_light_node/debug_selection', '/traffic_light_node/debug_classifier',
        '/traffic_light_node/debug_runtime', '/traffic_light_node/debug_policy_reason',
        '/traffic_light_node/debug_observed_label', '/traffic_light_node/debug_raw_label']
    doc['recording'].update(bag_enabled=True, include_sensors=False,
                            selected_topics=list(dict.fromkeys(topics)),
                            video=dict(enabled=False))
    doc['upload']['enabled'] = False
    doc['validation']['localization_safety'] = dict(enabled=True)
    doc['output_root'] = 'planner_logs/validation'
    output.mkdir(parents=True)
    (output / 'scenarios').mkdir()
    (output / 'scenarios/no_objects.json').write_text(json.dumps(scene, indent=2) + '\n')
    config = output / 'trial.yaml'
    config.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False))
    receipt = dict(base=str(BASE), base_sha256=hashlib.sha256(BASE.read_bytes()).hexdigest(),
                   source_scene=str(source), source_scene_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   goal_and_waypoints='unchanged from full-course base',
                   planning_signal_policy='unchanged', executed=False)
    (output / 'preparation.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--models-dir', type=Path, required=True)
    parser.add_argument('--python', required=True)
    args = parser.parse_args()
    print(prepare(args.output, args.models_dir, args.python))


if __name__ == '__main__':
    main()
