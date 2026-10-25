#!/usr/bin/env python3
"""Create fresh derivative dev trials without modifying frozen source scenarios."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'artifacts/evaluation/traffic_light_robustness_261025'


def prepare(output):
    output.mkdir(parents=True, exist_ok=False)
    source = BASE/'production_trials_abcd_261026_v4'
    receipt = {'scope': 'development only; no live changes', 'inputs': {}, 'configs': []}
    for name in ('A_focused', 'B_focused', 'C_focused', 'D_focused', 'full_regression'):
        path = source/(name+'.yaml')
        receipt['inputs'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        original = yaml.safe_load(path.read_text())
        for timed in (False, True):
            cfg = copy.deepcopy(original)
            label = name+('_timed' if timed else '_off')
            cfg['campaign_name'] = 'traffic_261026_'+label
            if name == 'B_focused':
                # Finish >60m beyond B but before C. The historical B-focused
                # goal included an unrelated signal wait at C, confounding the
                # bounded native-control experiment and prolonging exposure.
                cfg['default_case']['goal'] = dict(x=71.4, y=450., z=28.4, yaw_deg=90.)
            if name == 'C_focused':
                # Likewise end beyond C, without the later right turn or D.
                cfg['default_case']['goal'] = dict(x=71.4, y=565., z=28.4, yaw_deg=90.)
            cfg['stack'].update(
                traffic_light_yellow_passthrough=timed,
                signal_compliance_enable_timed_yellow=timed,
                signal_compliance_required_green_observations=2,
                signal_compliance_min_yellow_sec=2.0,
                signal_compliance_yellow_time_margin_sec=.2)
            target = output/(label+'.yaml')
            target.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
            receipt['configs'].append(str(target))
    (output/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    prepare(parser.parse_args().output.resolve())
