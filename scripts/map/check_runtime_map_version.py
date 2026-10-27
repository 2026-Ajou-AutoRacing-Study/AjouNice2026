#!/usr/bin/env python3
"""Audit v97 runtime defaults and the pinned requester/map contract."""
import argparse
import ast
import hashlib
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
VERSION = 97
SHA = 'da22c593a980a8fe02e136a004e1918ccf46a82381024854a4fa344eb992e0d9'
PROFILE_VERSION = 97
PROFILE_SHA = 'da22c593a980a8fe02e136a004e1918ccf46a82381024854a4fa344eb992e0d9'
SOURCE_ROOTS = ('base_ws/src', 'planning_ws/src', 'configs', 'scripts/runtime')
INSTALL_ROOTS = ('base_ws/install_aw149', 'base_ws/install_planning_addons',
                 'planning_ws/install_isolated')
PROFILE_TEST = ('base_ws/src/planning_addons/competition_route_requester/'
                'test/test_v97_profile.py')
UPDATE_WRAPPER = 'scripts/runtime/update_competition_runtime.sh'
# Exact retired install paths are deletion targets, never runtime consumers.
RETIRED_REQUESTER_CLEANUP = (
    'for retired in competition_route_requester/config/competition_route_requester_v88.param.yaml '
    'competition_route_requester/config/kcity_v88_competition_route.yaml '
    'ajounice_planning_integration/launch/competition_planning_v88.launch; do')
PROFILE_PREPARER_REFERENCES = {UPDATE_WRAPPER, 'scripts/runtime/test_update_competition_runtime.py'}
PROFILE_PREPARER = re.compile(r'(?<![\w])prepare_kcity_v(?:94|95|96|97)\.py(?![\w.])')
VALIDATOR_MISSIONS = ('base_ws/src/planning_addons/planning_scenario_validator/'
                      'config/missions')
VERSION_REF = re.compile(r'(?:lanelet2_map_v|kcity_v)(\d+)')
HISTORICAL_CONFIG = re.compile(
    r'(?:kcity|competition_planning|competition_rule_monitor|competition_route_requester)_v(\d+)')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_yaml(path):
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict):
        raise ValueError('YAML must be a mapping')
    return value


def check_profile(root, installed=False):
    """Validate the v97 manifest, requester configuration and explicit entrypoint."""
    issues = []
    manifest_path = root / 'configs/map/kcity_v97_asset.yaml'
    try:
        manifest = read_yaml(manifest_path)
        expected = {
            'version': PROFILE_VERSION,
            'file': 'kcity_map/lanelet_map/lanelet2_map_v97.osm',
            'projector_file': 'kcity_map/lanelet_map/map_projector_info_v1.yaml',
            'sha256': PROFILE_SHA,
        }
        actual = dict(version=manifest.get('map_version'), file=manifest.get('map_file'),
                      projector_file=manifest.get('projector_file'), sha256=manifest.get('map_sha256'))
        if actual != expected:
            raise ValueError('explicit v97 manifest version/path/projector/SHA contract mismatch')
    except (OSError, ValueError, yaml.YAMLError) as error:
        return [f'{manifest_path.relative_to(root)}: {error}']

    bundles = [('source', root / 'base_ws/src/planning_addons')]
    if installed:
        for directory in INSTALL_ROOTS:
            share = root / directory / 'share'
            if any((share / package).is_dir() for package in
                   ('competition_route_requester', 'ajounice_planning_integration')):
                bundles.append((directory, share))
    for label, folder in bundles:
        config = folder / 'competition_route_requester/config'
        route_path = config / 'kcity_v97_competition_route.yaml'
        param_path = config / 'competition_route_requester_v97.param.yaml'
        launch_path = folder / 'ajounice_planning_integration/launch/competition_planning_v97.launch'
        try:
            route = read_yaml(route_path)
            if route.get('map') != expected:
                raise ValueError('explicit v97 route map contract differs from manifest')
            if read_yaml(param_path).get('route_definition_file') != route_path.name:
                raise ValueError('explicit v97 requester selects a different route definition')
            launch = ET.parse(launch_path).getroot()
            include = next((node for node in launch.findall('include')
                if node.get('file') == '$(find ajounice_planning_integration)/launch/planning.launch'), None)
            if (include is None or not any(arg.get('name') == 'map_version' and
                    arg.get('value') == str(PROFILE_VERSION) for arg in include.findall('arg'))):
                raise ValueError('explicit v97 launcher does not select planning map_version=97')
        except (OSError, ValueError, yaml.YAMLError, ET.ParseError) as error:
            issues.append(f'{label} v97 profile: {error}')
    asset = root / expected['file']
    # v97 is supplied separately from Git. Verify metadata when present;
    # check_runtime also requires the default asset and verifies its SHA.
    if asset.is_file():
        if digest(asset) != PROFILE_SHA:
            issues.append('explicit v97 map SHA mismatch')
        else:
            try:
                metadata = next(element for _, element in ET.iterparse(asset, events=('start',))
                                if element.tag == 'MetaInfo')
                if metadata.get('map_version') != str(PROFILE_VERSION):
                    issues.append('explicit v97 OSM MetaInfo version mismatch')
            except (OSError, ET.ParseError, StopIteration) as error:
                issues.append(f'explicit v97 OSM metadata unavailable: {error}')
    return issues


def sha_lookup(value):
    """Read the launch's literal hash dictionary without executing roslaunch eval."""
    if not (value.startswith('$(eval ') and value.endswith(')')):
        raise ValueError('unsupported map SHA selector')
    expression = ast.parse(value[7:-1], mode='eval').body
    if not isinstance(expression, ast.Subscript) or not isinstance(expression.value, ast.Dict):
        raise ValueError('map SHA selector must index a literal dictionary')
    # Python 3.8 wraps Subscript.slice in Index; Python 3.9+ does not.
    index = expression.slice
    if isinstance(index, ast.Index):
        index = index.value
    if ast.dump(index) != ast.dump(ast.parse("str(arg('map_version'))", mode='eval').body):
        raise ValueError('map SHA selector must use map_version')
    result = ast.literal_eval(expression.value)
    if not isinstance(result, dict):
        raise ValueError('invalid map SHA lookup')
    return result


def check_launch_defaults(path, content, root):
    """Dynamic map paths must not hide a changed no-argument runtime default."""
    issues = []
    label = path.relative_to(root)
    try:
        launch = ET.fromstring(content)
        args = {node.get('name'): node for node in launch.findall('arg')}
        version_arg = args.get('map_version')
        if version_arg is not None:
            default = version_arg.get('value', version_arg.get('default'))
            if default != str(VERSION):
                issues.append(f'{label}: map_version default must remain {VERSION}, got {default!r}')
        if '$(arg map_version)' in content and version_arg is None:
            issues.append(f'{label}: dynamic map path has no local map_version default')
        for node in launch.findall('.//include/arg'):
            if node.get('name') == 'map_version' and node.get('value') not in (
                    str(VERSION), '$(arg map_version)'):
                issues.append(f'{label}: ordinary include overrides map_version: {node.get("value")!r}')
        hash_arg = args.get('map_sha256')
        if hash_arg is not None:
            value = hash_arg.get('value', hash_arg.get('default', ''))
            if value.startswith('$(eval '):
                hashes = sha_lookup(value)
                if hashes.get(str(VERSION)) != SHA:
                    issues.append(f'{label}: default map SHA selector mismatch')
                if hashes.get(str(PROFILE_VERSION)) != PROFILE_SHA:
                    issues.append(f'{label}: explicit v97 map SHA selector mismatch')
            elif value != SHA:
                issues.append(f'{label}: default map SHA mismatch')
    except (ET.ParseError, ValueError, SyntaxError) as error:
        issues.append(f'{label}: cannot validate launch map defaults: {error}')
    return issues


def frozen_workspace_copy(path, root, ignored_source_files):
    """Only exempt byte-identical installed copies of Git-ignored personal inputs."""
    for directory in INSTALL_ROOTS:
        missions = root / directory / 'share/planning_scenario_validator/config/missions'
        try:
            tail = path.relative_to(missions)
        except ValueError:
            continue
        if len(tail.parts) < 3 or tail.parts[1] != 'workspace':
            return False
        source = root / VALIDATOR_MISSIONS / tail
        return (str(source.relative_to(root)) in ignored_source_files and source.is_file()
                and digest(path) == digest(source))
    return False


def check_runtime(root=ROOT, *, installed=False, source_files=None, ignored_source_files=None):
    root = Path(root)
    if source_files is None:
        source_files = set(subprocess.check_output(
            ['git', '-C', str(root), 'ls-files', '-c', '-o', '--exclude-standard'],
            text=True).splitlines())
    if ignored_source_files is None:
        ignored_source_files = (set(subprocess.check_output(
            ['git', '-C', str(root), 'ls-files', '-o', '-i', '--exclude-standard', '--',
             VALIDATOR_MISSIONS], text=True).splitlines()) if installed else set())
    # Keep the existing operational scope and historical exclusions intact.
    roots = SOURCE_ROOTS + (INSTALL_ROOTS if installed else ())
    frozen_programs = {'generate_highway_lane_change_samples.py', 'test_v94_profile.py', 'test_v95_profile.py', 'test_v96_profile.py'}
    issues = check_profile(root, installed)
    count = 0
    for directory in roots:
        for path in (root / directory).rglob('*'):
            if (not path.is_file() or 'docs' in path.parts or 'legacy' in path.parts or
                    path.name in frozen_programs or
                    ('tracking_prediction_evaluation' in path.parts and path.suffix != '.launch'
                     and path.name not in {'evaluate_pipeline_prediction.py', 'evaluate_velocity_oracle_ablation.py',
                                           'audit_lanelet_centerline_bias.py', 'evaluate_map_centerline_snap_ablation.py',
                                           'evaluate_oracle_prediction.py'})):
                continue
            relative = str(path.relative_to(root))
            # The manifest's runtime contract was checked above; its base-map
            # and stop-line provenance deliberately names historical assets.
            if relative == 'configs/map/kcity_v97_asset.yaml':
                continue
            if '/install' not in str(path) and relative not in source_files:
                continue
            if path.parent == root / 'configs/map' and path.name.endswith('_stopline_candidate.yaml'):
                continue
            historical = HISTORICAL_CONFIG.search(path.name)
            if historical and int(historical[1]) != VERSION:
                # Explicit v97 source/install bundles were checked above;
                # older rollback configs keep the original historical handling.
                continue
            if path.suffix not in {'.py', '.yaml', '.yml', '.launch', '.sh', '.rviz', '.cpp', '.hpp', ''}:
                continue
            if installed and frozen_workspace_copy(path, root, ignored_source_files):
                continue
            try:
                content = path.read_text()
            except (UnicodeError, OSError):
                continue
            if path.suffix == '.launch':
                issues.extend(check_launch_defaults(path, content, root))
            if relative == UPDATE_WRAPPER:
                definitions = re.findall(r'^MAP_VERSION=(.*)$', content, flags=re.MULTILINE)
                if len(definitions) != 1 or definitions[0].strip().strip('\'"') != str(VERSION):
                    issues.append(f'{relative}: wrapper MAP_VERSION default must remain {VERSION}')
            # The current profile test compares unchanged route policy with v96.
            # Continue scanning it: any other version is still an error.
            allowed = {96, VERSION} if relative == PROFILE_TEST else {VERSION}
            # Validator START provenance and recorded-profile fixtures intentionally pin v94.
            if relative in {
                                        'base_ws/src/planning_addons/planning_scenario_validator/tests/test_route_profile.py',
}:
                allowed.add(94)
            for line_number, line in enumerate(content.splitlines(), 1):
                if line.lstrip().startswith('#'):
                    continue
                if relative == UPDATE_WRAPPER and line.strip() == RETIRED_REQUESTER_CLEANUP:
                    continue
                for match in VERSION_REF.finditer(line):
                    count += 1
                    if relative in PROFILE_PREPARER_REFERENCES and any(
                            tool.start() <= match.start() and match.end() <= tool.end()
                            for tool in PROFILE_PREPARER.finditer(line)):
                        continue
                    if int(match[1]) not in allowed:
                        issues.append(f'{relative}:{line_number}: {match[0]}')
    map_path = root / f'kcity_map/lanelet_map/lanelet2_map_v{VERSION}.osm'
    try:
        if digest(map_path) != SHA:
            issues.append('Map SHA mismatch')
    except OSError as error:
        issues.append(f'Map unavailable: {error}')
    return count, issues


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--installed', action='store_true')
    args = parser.parse_args(argv)
    count, issues = check_runtime(installed=args.installed)
    if issues:
        parser.exit(1, '\n'.join(issues) + '\n')
    print(f'PASS: {count} runtime map references checked; default v{VERSION} SHA matches; '
          f'explicit v{PROFILE_VERSION} profile contract matches')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
