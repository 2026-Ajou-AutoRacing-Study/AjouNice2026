#!/usr/bin/env python3
"""Check runtime map references in source and optional Catkin install spaces."""
import argparse
from pathlib import Path
import re
import hashlib

ROOT = Path(__file__).resolve().parents[2]
VERSION = 84
SHA = '36369766a0fade01cbd64a49d4d9c33248b15770eb1433ce334d6becaddf3e4d'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--installed', action='store_true')
args = parser.parse_args()
roots = ['base_ws/src', 'planning_ws/src', 'configs', 'scripts', 'perception', 'dataset']
if args.installed:
    roots += ['base_ws/install_aw149', 'base_ws/install_planning_addons', 'planning_ws/install_isolated']
issues = []
count = 0
for directory in roots:
    for path in (ROOT/directory).rglob('*'):
        if not path.is_file() or 'docs' in path.parts:
            continue
        # Released versioned configs remain available for rollback, but are not
        # runtime consumers unless a non-versioned launch/config points to them.
        historical_config = re.search(r'kcity_v(\d+)', path.name)
        if historical_config and int(historical_config[1]) != VERSION:
            continue
        if path.suffix not in {'.py', '.yaml', '.yml', '.launch', '.sh', '.rviz', '.cpp', '.hpp', ''}:
            continue
        try:
            content = path.read_text()
        except (UnicodeError, OSError):
            continue
        for line_number, line in enumerate(content.splitlines(), 1):
            for match in re.finditer(r'(?:lanelet2_map_v|kcity_v)(\d+)', line):
                count += 1
                if int(match[1]) != VERSION:
                    issues.append(f'{path.relative_to(ROOT)}:{line_number}: {match[0]}')
map_path = ROOT/f'kcity_map/lanelet_map/lanelet2_map_v{VERSION}.osm'
assert hashlib.sha256(map_path.read_bytes()).hexdigest() == SHA, 'Map SHA mismatch'
if issues:
    raise SystemExit('\n'.join(issues))
print(f'PASS: {count} runtime version references are v{VERSION}; map SHA matches')
