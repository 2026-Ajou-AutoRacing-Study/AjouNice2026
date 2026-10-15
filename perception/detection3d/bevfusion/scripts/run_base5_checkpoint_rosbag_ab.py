#!/usr/bin/env python3
"""Replay E1/E2 against frozen old/new E25 case manifests, without overwriting them."""
import argparse
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

from run_old_mission_current_contract import ROOT, sha256_file
from run_teammate_manual_evaluation import run

RUN = ROOT / 'artifacts/detection3d/bevfusion/team_training/sw/bf-sw-base5-nearfield-replay-e2-pilot-rtx5060ti-261004-214609'
CATALOG = ROOT / 'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json'
SOURCES = {
    'old': ROOT / 'artifacts/evaluation/bevfusion/old_mission_current_contract_261004/e25',
    'new': ROOT / 'artifacts/evaluation/bevfusion/teammate_manual_261004/e25',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/evaluation/bevfusion/base5_nearfield_replay_e2_261005')
    parser.add_argument('--models', nargs='+', choices=['e1', 'e2'], default=['e1', 'e2'])
    parser.add_argument('--missions', nargs='+', default=['pedestrian', 'obstacle', 'highway', 'roundabout'])
    parser.add_argument('--manifest-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = []
    for cohort, source in SOURCES.items():
        for path in sorted(source.glob('*/case_contract.json')):
            contract = json.loads(path.read_text())
            if contract['mission'] not in args.missions:
                continue
            assert Path(contract['raw_bag']).is_file(), contract['raw_bag']
            assert (path.parent / 'summary_status_size_v2.json').is_file(), path
            cases.append((cohort + '__' + path.parent.name, path.parent, contract))
    # Mission order is deliberate: the current failure modes before vehicle regressions.
    cases.sort(key=lambda c: (args.missions.index(c[2]['mission']), c[0]))
    manifest = dict(checkpoints={model: dict(path=str(RUN / f'epoch_{model[1:]}.pth'),
        sha256=sha256_file(RUN / f'epoch_{model[1:]}.pth')) for model in args.models},
        cases=[dict(label=label, source=str(source), **contract) for label, source, contract in cases],
        catalog_sha256=sha256_file(CATALOG),
        tracker_sha256=sha256_file(ROOT / 'base_ws/install_aw149/lib/ekf_multi_object_tracking/ekf_multi_object_tracking'),
        map_sha256=sha256_file(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v85.osm'),
        thresholds=[.40, .15, .10], detection_rate=.5, pipeline_rate=1.,
        deskew='off', velocity_fusion=False)
    (args.output / 'runtime_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    if args.manifest_only:
        print('MANIFEST', len(cases), 'cases', flush=True)
        return
    env = os.environ.copy()
    env.update(AJOUNICE_ROOT=str(ROOT), BEVFUSION_IMAGE='ajounice/bevfusion:team-cu128-v1',
        BEVFUSION_MODEL_CONFIG='/workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py',
        BEVFUSION_EVAL_ASSET_BBOX_CATALOG=str(CATALOG),
        BEVFUSION_EVAL_GT_REFERENCE='asset-catalog-bbox-center-v1',
        BEVFUSION_PIPELINE_EVAL_MAXIMUM_TRACKING_AGE_SEC='0.5',
        BEVFUSION_PIPELINE_EVAL_LAUNCH=str(ROOT / 'base_ws/install_aw149/share/perception_object_pipeline/launch/tracking_prediction.launch'),
        BEVFUSION_EVAL_LIDAR_DESKEW_MODE='off', BEVFUSION_EVAL_QUERY_MEMORY_CONTEXT='false',
        BEVFUSION_EVAL_PUBLISH_DETECTOR_VELOCITY='false',
        BEVFUSION_EVAL_TEMPORAL_SWEEPS='0', BEVFUSION_EVAL_TEMPORAL_FEATURE_CACHE='false',
        TRACKER_FAST_CONFIRMED_VEHICLE_PROMOTION='false', TRACKER_CENTER_MOTION_VELOCITY_FUSION='false',
        TRACKER_DETECTION_VELOCITY_FUSION_MODE='0', TRACKER_MEAS_NOISE_STD_XY_M='0.4',
        BEVFUSION_EVAL_VEHICLE_THRESHOLD='0.40', BEVFUSION_EVAL_PEDESTRIAN_THRESHOLD='0.15',
        BEVFUSION_EVAL_OBSTACLE_THRESHOLD='0.10', BEVFUSION_EVAL_SCORE_THRESHOLD='0.10')
    compose = ['docker', 'compose', '-f', str(ROOT / 'docker/bevfusion/compose.yaml')]
    for model in args.models:
        env['BEVFUSION_CHECKPOINT'] = manifest['checkpoints'][model]['path']
        run(compose + ['up', '-d', '--force-recreate', '--pull', 'never', 'inference'], env)
        try:
            health = dict(env, PYTHONPATH=str(ROOT / 'docker/bevfusion'))
            for attempt in range(120):
                check = subprocess.run(['/usr/bin/python3', str(ROOT / 'docker/bevfusion/scripts/healthcheck.py')],
                    cwd=ROOT, env=health, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if check.returncode == 0:
                    break
                time.sleep(1)
            else:
                raise RuntimeError('inference not ready')
            for label, source, contract in cases:
                out = args.output / model / label
                out.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source / 'asset_name_map.json', out / 'asset_name_map.json')
                updated = dict(contract, model=model, checkpoint=env['BEVFUSION_CHECKPOINT'],
                    asset_name_map=str(out / 'asset_name_map.json'), baseline_case=str(source),
                    cohort=label.split('__')[0], asset_bbox_catalog=str(CATALOG))
                contract_path = out / 'case_contract.json'
                if contract_path.exists() and json.loads(contract_path.read_text()) != updated:
                    raise RuntimeError('refuse mismatched resumed case: ' + str(out))
                contract_path.write_text(json.dumps(updated, indent=2) + '\n')
                case_env = dict(env, BEVFUSION_EVAL_ASSET_NAME_MAP=str(out / 'asset_name_map.json'),
                    BEVFUSION_EVAL_MORAI_HEADING_CORRECTION_DEG=str(contract['heading_correction_deg']),
                    BEVFUSION_EVAL_CLASS_NAME=('vehicle' if contract['mission'] in ('highway', 'roundabout') else contract['mission']))
                if not (out / 'summary.json').exists():
                    run(['bash', str(ROOT / 'perception/detection3d/bevfusion/scripts/run_rosbag_detection_evaluation.sh'),
                        contract['raw_bag'], str(out), label, '0.5'], case_env, out / 'detection_runner.log')
                if not (out / 'pipeline/summary_cv_stable.json').exists():
                    run(['bash', str(ROOT / 'perception/detection3d/bevfusion/scripts/run_recorded_detections_pipeline_evaluation.sh'),
                        contract['raw_bag'], str(out / 'detections.bag'), str(out / 'pipeline'), label, '1.0'],
                        case_env, out / 'pipeline_runner.log')
                scoring = dict(case_env, PYTHONPATH=str(ROOT / 'base_ws/src/tracking_prediction_evaluation/src') + ':' + case_env.get('PYTHONPATH', ''))
                run(['/usr/bin/python3', str(ROOT / 'perception/detection3d/bevfusion/scripts/reevaluate_teammate_resolved_frames.py'),
                    '--root', str(args.output), '--catalog', str(CATALOG), '--suffix', '_status_size_v2'],
                    scoring, out / 'rescore_runner.log')
                print('COMPLETE', model, label, flush=True)
        finally:
            run(compose + ['stop', 'inference'], env)


if __name__ == '__main__':
    main()
