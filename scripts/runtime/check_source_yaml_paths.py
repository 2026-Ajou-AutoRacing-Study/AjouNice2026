#!/usr/bin/env python3
"""Resolve installed launch trees without starting nodes; reject installed YAML copies."""
import argparse
import json
import os
from pathlib import Path

import roslaunch
import rospkg


class YamlAudit(roslaunch.xmlloader.XmlLoader):
    def __init__(self):
        super().__init__()
        self.yaml_files = set()

    def load_rosparam(self, context, config, cmd, param, file, value,
                      verbose=True, subst_function=None):
        if file and cmd == 'load':
            self.yaml_files.add(str(Path(file).absolute()))
        return super().load_rosparam(
            context, config, cmd, param, file, value, verbose, subst_function)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', type=Path, help='Write resolved configuration sources')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    os.environ['AJOUNICE_ROOT'] = str(root)
    packages = rospkg.RosPack()
    entries = [
        ('planning', 'ajounice_planning_integration', 'planning.launch'),
        ('stack', 'ajounice_planning_integration', 'morai_planning_stack.launch'),
        ('control', 'trajectory_follower', 'pp_pi_autoware.launch'),
    ]
    report = {}
    errors = []
    for name, package, launch in entries:
        entry = Path(packages.get_path(package)) / 'launch' / launch
        loader = YamlAudit()
        config = roslaunch.config.ROSLaunchConfig()
        loader.load(str(entry), config, verbose=False)
        file_params = {
            key: param.value for key, param in config.params.items()
            if isinstance(param.value, str) and param.value.endswith(('.yaml', '.yml'))
        }
        paths = loader.yaml_files | set(file_params.values())
        for path in paths:
            candidate = Path(path)
            if not candidate.is_absolute() or not candidate.is_file():
                errors.append(f'{name}: missing or relative YAML: {path}')
            elif any(part.startswith(('install_', 'devel_')) or part in ('install', 'devel')
                     for part in candidate.parts):
                errors.append(f'{name}: installed YAML copy: {path}')
        report[name] = {
            'entry': str(entry), 'yaml_files': sorted(loader.yaml_files),
            'yaml_file_parameters': file_params,
            'steering_rate': {
                k: v.value for k, v in config.params.items()
                if k.endswith('/adaptive/max_steering_rate_deg_s')
            },
        }
        print(f'{name}: {len(paths)} YAML sources resolved')
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + '\n')
    if errors:
        raise SystemExit('\n'.join(errors))
    print('PASS: all checked YAML paths read source files; no nodes started.')


if __name__ == '__main__':
    main()
