#!/usr/bin/env python3
"""Check runtime map references in source and optional Catkin install spaces."""
import argparse
from pathlib import Path
import re
import hashlib
import subprocess

ROOT = Path(__file__).resolve().parents[2]
VERSION = 94
SHA = '3d11cd820b2f4d755960ae565dbc267e4cf5826307ef2df1149e10922c138cbf'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--installed', action='store_true')
args = parser.parse_args()
# Operational stack only; frozen dataset/model qualification scripts have their own map contracts.
roots = ['base_ws/src', 'planning_ws/src', 'configs', 'scripts/runtime']

if args.installed:
    roots += ['base_ws/install_aw149', 'base_ws/install_planning_addons', 'planning_ws/install_isolated']
# Frozen qualification programs keep the map used to produce their evidence.
# Workspace campaign snapshots are ignored by Git and must remain resumable.
frozen_programs = {'generate_highway_lane_change_samples.py'}
source_files = set(subprocess.check_output(
    ['git', '-C', str(ROOT), 'ls-files', '-c', '-o', '--exclude-standard'],
    text=True).splitlines())
issues = []
count = 0
for directory in roots:
    for path in (ROOT/directory).rglob('*'):
        if (not path.is_file() or 'docs' in path.parts or 'legacy' in path.parts or
                path.name in frozen_programs or
                ('tracking_prediction_evaluation' in path.parts and path.suffix != '.launch')):
            continue
        if '/install' not in str(path) and str(path.relative_to(ROOT)) not in source_files:
            continue
        # Installed copies of user campaign YAMLs are historical experiment
        # inputs, not current defaults. Preserve their original map contracts.
        if ('planning_scenario_validator' in path.parts and 'config' in path.parts
                and 'workspace' in path.parts):
            continue
        # Released versioned configs remain available for rollback, but are not
        # runtime consumers unless a non-versioned launch/config points to them.
        historical_config = re.search(r'(?:kcity|competition_planning|competition_rule_monitor|competition_route_requester)_v(\d+)', path.name)
        if historical_config and int(historical_config[1]) != VERSION:
            continue
        if path.suffix not in {'.py', '.yaml', '.yml', '.launch', '.sh', '.rviz', '.cpp', '.hpp', ''}:
            continue
        try:
            content = path.read_text()
        except (UnicodeError, OSError):
            continue
        for line_number, line in enumerate(content.splitlines(), 1):
            if line.lstrip().startswith('#'):
                continue
            for match in re.finditer(r'(?:lanelet2_map_v|kcity_v)(\d+)', line):
                count += 1
                if int(match[1]) != VERSION:
                    issues.append(f'{path.relative_to(ROOT)}:{line_number}: {match[0]}')
map_path = ROOT/f'kcity_map/lanelet_map/lanelet2_map_v{VERSION}.osm'
assert hashlib.sha256(map_path.read_bytes()).hexdigest() == SHA, 'Map SHA mismatch'
if issues:
    raise SystemExit('\n'.join(issues))
print(f'PASS: {count} runtime version references are v{VERSION}; map SHA matches')
