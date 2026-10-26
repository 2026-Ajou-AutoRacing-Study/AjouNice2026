#!/usr/bin/env python3
"""Prepare fresh static-paint-proxy trials; never load a scene or start ROS.

Frozen historical virtual-line geometry/configs stay unchanged. Native GET
crossings in these derivatives refer to original MGeo paint, not official
penalty volumes. A/B/C/D vehicle signals still come from real YOLO/CNN.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'artifacts/evaluation/traffic_light_robustness_261025'


def prepare(output):
    output = output.resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    source = BASE/'production_trials_abcd_261026_v4'
    profile = ROOT/'base_ws/src/perception/traffic_light_detection/config/traffic_signal_stop_lines.yaml'
    contracts = yaml.safe_load(profile.read_text())['signal_stop_line_contracts']
    groups_path = source/'groups.json'
    geometry = json.loads(groups_path.read_text())
    preparation = copy.deepcopy(geometry)
    output.mkdir(parents=True, exist_ok=False)
    receipt = dict(scope='original_MGeo_paint_proxy_not_official_penalty_certification', inputs={})
    def record(path):
        receipt['inputs'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    record(profile); record(groups_path)
    for group in geometry['groups']:
        contract = next(c for c in contracts if c['regulatory_id'] == group['group_id'])
        xy = contract['crossing_line_xy']
        group['historical_virtual_stop_line_xy'] = group['stop_line_xy']
        group['stop_line_xy'] = [xy[i:i+2] for i in range(0, len(xy), 2)]
        prep = next(g for g in preparation['groups'] if g['group_id'] == group['group_id'])
        a, b = prep['stop_line_xy'][0], prep['stop_line_xy'][-1]
        nx, ny = -(b[1]-a[1]), b[0]-a[0]
        length = math.hypot(nx, ny); nx /= length; ny /= length
        if (xy[0]-a[0])*nx+(xy[1]-a[1])*ny < 0:
            nx, ny = -nx, -ny
        margin = contract['additional_stop_margin_m']
        prep['stop_line_xy'] = [[p[0]-margin*nx, p[1]-margin*ny] for p in prep['stop_line_xy']]
        prep['preparation_additional_stop_margin_m'] = margin
    geometry['crossing_reference'] = receipt['scope']
    (output/'groups_original_paint.json').write_text(json.dumps(geometry, indent=2)+'\n')
    preparation['crossing_reference'] = 'preparation_only_adjusted_virtual_stop_target_not_penalty_line'
    (output/'groups_preparation_stop_targets.json').write_text(json.dumps(preparation, indent=2)+'\n')
    for mission in ('A', 'B', 'C', 'D', 'full_regression'):
        name = mission+'_focused' if mission != 'full_regression' else mission
        original = source/(name+'.yaml'); record(original)
        cfg = copy.deepcopy(yaml.safe_load(original.read_text()))
        cfg['campaign_name'] = 'signal_line_separation_261026_'+mission
        cfg['stack'].update(traffic_light_yellow_passthrough=False,
            signal_compliance_enable_timed_yellow=False,
            signal_compliance_required_green_observations=3)
        if mission == 'B':
            cfg['default_case']['goal'] = dict(x=71.4, y=450., z=28.4, yaw_deg=90.)
        if mission == 'C':
            cfg['default_case']['goal'] = dict(x=71.4, y=565., z=28.4, yaw_deg=90.)
        cfg['default_case']['timeout_sec'] = 100 if mission != 'full_regression' else 360
        (output/(mission+'.yaml')).write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
    (output/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    prepare(parser.parse_args().output)
