#!/usr/bin/env python3
"""Generate fresh trial inputs from frozen campaigns, without starting ROS."""
import argparse
import hashlib
import json
from pathlib import Path

import yaml

from prepare_traffic_light_compliance_trials import stopline_groups


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-trials', type=Path, required=True)
    parser.add_argument('--map-file', type=Path, required=True)
    parser.add_argument('--projector', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output; frozen trial inputs must never be rewritten')
    groups = stopline_groups(args.map_file, args.projector)
    campaigns = {}
    for name in ('A_focused', 'B_focused', 'C_focused', 'D_focused', 'full_regression'):
        cfg = yaml.safe_load((args.source_trials/(name+'.yaml')).read_text())
        cfg['campaign_name'] += '_map_candidate'
        cfg['resume'] = False
        cfg['recording']['selected_topics'].append('/sensor/camera/front/compressed')
        # End focused trials before the next intersection, not before this signal.
        if name == 'B_focused':
            cfg['default_case']['goal'].update(x=71.4, y=450., z=28.4, yaw_deg=90.)
        if name == 'C_focused':
            cfg['default_case']['goal'].update(x=71.4, y=565., z=28.4, yaw_deg=90.)
        campaigns[name] = cfg
    args.output.mkdir(parents=True)
    (args.output/'groups.json').write_text(json.dumps(dict(schema_version=1, frame='map', groups=groups), indent=2)+'\n')
    for name, cfg in campaigns.items():
        (args.output/(name+'.yaml')).write_text(yaml.safe_dump(cfg, sort_keys=False))
    (args.output/'preparation.json').write_text(json.dumps(dict(
        source_trials=str(args.source_trials.resolve()), map_file=str(args.map_file.resolve()),
        map_sha256=hashlib.sha256(args.map_file.read_bytes()).hexdigest(),
        source_scenarios_unchanged=True, live_driving_performed=False,
        required_launcher='run_traffic_light_compliance_trial.py --map-file <this map>'), indent=2)+'\n')


if __name__ == '__main__':
    main()
