#!/usr/bin/env python3
"""Fresh E25 inference and current tracker replay for historical mission bags.

Writes a separate run tree; never overwrites historical results. Scale-variant
CargoBox back/forth is not mixed into the scale-one obstacle comparison.
"""
import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path

from run_teammate_manual_evaluation import E25, ROOT, run


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT /
                        'artifacts/evaluation/bevfusion/old_mission_current_contract_261004')
    parser.add_argument('--missions', nargs='+', default=['highway', 'roundabout', 'pedestrian', 'obstacle'])
    parser.add_argument('--manifest-only', action='store_true')
    args = parser.parse_args()
    historical = ROOT / 'artifacts/evaluation/bevfusion/bf_e40fresh_e25_class_thresholds_040_015_010_260918'
    cases = [
        ('vehicle_highway_merge', 'highway', -1.347),
        ('vehicle_highway1', 'highway', -1.347),
        ('vehicle_roundabout_new', 'roundabout', -1.347),
        ('pedestrian_crossing', 'pedestrian', 0.0),
        ('pedestrian_distance', 'pedestrian', 0.0),
        ('obstacle_base3_multiasset', 'obstacle', -1.347),
    ]
    catalog = ROOT / 'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json'
    args.output.mkdir(parents=True, exist_ok=True)
    runtime_files = [catalog, E25,
        ROOT / 'kcity_map/lanelet_map/lanelet2_map_v85.osm',
        ROOT / 'base_ws/install_aw149/lib/ekf_multi_object_tracking/ekf_multi_object_tracking']
    (args.output / 'runtime_contract.json').write_text(json.dumps(dict(
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        sha256={str(p.relative_to(ROOT)): sha256_file(p) for p in runtime_files},
        thresholds=[0.40, 0.15, 0.10], range_m=30, nms_rotated_bev_iou=0.5,
        tracker_xy_noise_m=0.4, maximum_prediction_age_sec=0.5,
        map='v85', detector_velocity_fusion=False,
        excluded_cases={'obstacle_backforth': 'Scaled sample CargoBox lacks a directly audited scale-specific catalog entry; compare scale-one multiasset bags instead.'}
    ), indent=2) + '\n')
    if args.manifest_only:
        return
    actor_catalog = json.loads((ROOT / 'dataset/morai_od/configs/morai_asset_catalog_260807.json').read_text())
    names_by_id = {int(v['data_id']): v['name'] for values in actor_catalog.values()
                   if isinstance(values, list) for v in values if isinstance(v, dict) and 'data_id' in v}
    scenario_root = Path.home() / 'MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025'
    env = os.environ.copy()
    env.update(AJOUNICE_ROOT=str(ROOT), BEVFUSION_IMAGE='ajounice/bevfusion:team-cu128-v1',
               BEVFUSION_CHECKPOINT=str(E25),
               BEVFUSION_MODEL_CONFIG='/workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py',
               BEVFUSION_EVAL_GT_REFERENCE='asset-catalog-bbox-center-v1',
               BEVFUSION_EVAL_ASSET_BBOX_CATALOG=str(catalog),
               BEVFUSION_PIPELINE_EVAL_MAXIMUM_TRACKING_AGE_SEC='0.5',
               BEVFUSION_PIPELINE_EVAL_LAUNCH=str(ROOT / 'base_ws/install_aw149/share/perception_object_pipeline/launch/tracking_prediction.launch'),
               BEVFUSION_EVAL_QUERY_MEMORY_CONTEXT='false',
               BEVFUSION_EVAL_PUBLISH_DETECTOR_VELOCITY='false',
               BEVFUSION_EVAL_LIDAR_DESKEW_MODE='off')
    compose = ['docker', 'compose', '-f', str(ROOT / 'docker/bevfusion/compose.yaml')]
    prepared = []
    for label, mission, heading in cases:
        if mission not in args.missions:
            continue
        report = json.loads((historical / label / 'summary.json').read_text())
        bag = Path(report['source']['raw_bag'])
        if not bag.is_file():
            bag = Path.home() / 'pointpainting_online_eval_rosbags_7_260831' / bag.name
        if not bag.is_file():
            raise FileNotFoundError(bag)
        output = args.output / 'e25' / label
        output.mkdir(parents=True, exist_ok=True)
        names = {}
        provenance = {}
        if mission in ('pedestrian', 'obstacle'):
            scenario = (scenario_root / (bag.stem + '.json') if mission == 'pedestrian' else
                        ROOT / 'configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json')
            payload = scenario.read_bytes()
            document = json.loads(payload)
            field = 'pedestrianList' if mission == 'pedestrian' else 'objectList'
            for obj in document[field]:
                if any(abs(float(obj.get('scale', {}).get(axis, 1)) - 1) > 1e-6 for axis in 'xyz'):
                    raise ValueError('nonunit scale in ' + str(scenario))
                names[f"{mission}:{obj['UNIQUEID']}"] = names_by_id[int(obj['DataID'])]
            provenance = dict(scenario=str(scenario), scenario_sha256=hashlib.sha256(payload).hexdigest())
        name_map = output / 'asset_name_map.json'
        name_map.write_text(json.dumps(dict(asset_names=names, **provenance), indent=2) + '\n')
        (output / 'case_contract.json').write_text(json.dumps(dict(
            model='e25', mission=mission, role='historical_comparison', raw_bag=str(bag),
            checkpoint=str(E25), heading_correction_deg=heading,
            asset_name_map=str(name_map), gt_reference='asset-catalog-bbox-center-v1',
            asset_bbox_catalog=str(catalog), tracking_maximum_age_sec=0.5,
            detection_playback_rate=0.5, pipeline_playback_rate=1.0,
            lidar_deskew='off', detector_velocity_fusion=False,
            note='Fresh current inference; pedestrian launcher heading retains historical 0-degree convention.'
        ), indent=2) + '\n')
        prepared.append((label, mission, heading, bag, output, name_map))
    run(compose + ['up', '-d', '--force-recreate', '--pull', 'never', 'inference'], env)
    try:
        health = env.copy()
        health['PYTHONPATH'] = str(ROOT / 'docker/bevfusion')
        for _ in range(120):
            check = subprocess.run(['/usr/bin/python3', str(ROOT / 'docker/bevfusion/scripts/healthcheck.py')],
                                   cwd=ROOT, env=health, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if check.returncode == 0:
                break
            time.sleep(1)
        else:
            raise RuntimeError('inference did not become ready')
        for label, mission, heading, bag, output, name_map in prepared:
            case_env = env.copy()
            case_env.update(BEVFUSION_EVAL_ASSET_NAME_MAP=str(name_map),
                            BEVFUSION_EVAL_MORAI_HEADING_CORRECTION_DEG=str(heading),
                            BEVFUSION_EVAL_CLASS_NAME='vehicle' if mission in ('highway', 'roundabout') else mission)
            if not (output / 'summary.json').is_file():
                run(['bash', str(ROOT / 'perception/detection3d/bevfusion/scripts/run_rosbag_detection_evaluation.sh'),
                     str(bag), str(output), label, '0.5'], case_env, output / 'detection_runner.log')
            if not (output / 'pipeline/summary_cv_stable.json').is_file():
                run(['bash', str(ROOT / 'perception/detection3d/bevfusion/scripts/run_recorded_detections_pipeline_evaluation.sh'),
                     str(bag), str(output / 'detections.bag'), str(output / 'pipeline'), label, '1.0'],
                    case_env, output / 'pipeline_runner.log')
            print('COMPLETE', label, flush=True)
    finally:
        run(compose + ['stop', 'inference'], env)


if __name__ == '__main__':
    main()
