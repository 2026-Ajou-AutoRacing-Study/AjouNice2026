#!/usr/bin/python3
"""Read-only bag/frozen-profile audit, including older executions; no model launch."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator.config import load_campaign
from planning_scenario_validator.recording_audit import audit_bag, profile_contract_warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--execution', type=Path)
    source.add_argument('--case-folder', type=Path, help='Imported case with input.json/result JSON but no execution snapshot')
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--output', type=Path, help='Only writes to this explicitly requested new report')
    args = parser.parse_args()
    directory = (args.execution or args.case_folder).resolve()
    if args.execution:
        cfg = load_campaign(directory / 'config_snapshot.yaml', check_install=False)
        state = json.loads((directory / 'campaign_state.json').read_text()) if (directory / 'campaign_state.json').exists() else {}
        preset = (state.get('model_contract') or {}).get('preset')
        if preset and preset != 'od_gt':
            cfg['stack']['bevfusion_model'] = preset
    else:
        saved = json.loads((directory / 'input.json').read_text())
        cfg = {'stack': {'detector': saved['detector'], 'pose_source': saved['pose_source'],
                         'bevfusion_model': (saved.get('model') or {}).get('preset', 'e25')},
               'recording': {'bag_enabled': saved.get('bag_enabled', True),
                             'topics': saved['recorded_topics'],
                             'topic_profile': saved.get('topic_profile', ''),
                             'topic_regex': saved.get('recorded_topic_regex', [])}}
    bags = [args.bag] if args.bag else sorted(directory.rglob('*.bag'))
    report = {'execution': str(directory), 'profile_warnings': profile_contract_warnings(cfg),
              'requested_topics': cfg['recording']['topics'], 'bags': []}
    for path in bags:
        # Do not infer driving_started solely from PASS/FAIL. Use saved drive evidence.
        result = path.with_suffix('.json')
        data = json.loads(result.read_text()) if result.exists() else {}
        metrics = data.get('metrics', {})
        started = bool(metrics.get('driving_started', metrics.get('drive_observation_available', False)))
        report['bags'].append(audit_bag(path, cfg, driving_started=started))
    text = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        if args.output.exists():
            raise ValueError('기존 보고서는 덮어쓰지 않습니다: {}'.format(args.output))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
