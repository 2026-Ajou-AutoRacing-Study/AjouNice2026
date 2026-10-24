#!/usr/bin/env python3
"""Check runtime map references in source and optional Catkin install spaces."""
import argparse
from pathlib import Path
import re
import hashlib

ROOT = Path(__file__).resolve().parents[2]
VERSION = 87
SHA = '70f2cdf7ee9dee3333d6553787525890c0a73c4fbcdfdcc9fc95a5c92c687575'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--installed', action='store_true')
args = parser.parse_args()
# Operational stack only; frozen dataset/model qualification scripts have their own map contracts.
roots = ['base_ws/src', 'planning_ws/src', 'configs', 'scripts']
historical_tools = {
    'base_ws/src/planning_addons/competition_route_requester/tools/generate_route_visualization.py',
    'base_ws/src/planning_addons/competition_route_requester/tools/generate_route_activation_windows.py',
}

if args.installed:
    roots += ['base_ws/install_aw149', 'base_ws/install_planning_addons', 'planning_ws/install_isolated']
issues = []
count = 0
for directory in roots:
    for path in (ROOT/directory).rglob('*'):
        if not path.is_file() or 'docs' in path.parts or path.relative_to(ROOT).as_posix() in historical_tools:
            continue
        # Released versioned configs remain available for rollback, but are not
        # runtime consumers unless a non-versioned launch/config points to them.
        historical_config = re.search(r'(?:kcity|competition_planning|competition_route_requester)_v(\d+)', path.name)
        if historical_config and int(historical_config[1]) != VERSION:
            continue
        # Preserved v85 speed caps are no longer selected by the default map launch.
        if path.name == 'lanelet_speed_limits.yaml':
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
