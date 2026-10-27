"""Run only temporary mock workspaces: no real build, downloads or ROS nodes."""
import json
import os
from pathlib import Path
import subprocess

import pytest


SCRIPT = Path(__file__).with_name('update_competition_runtime.sh')
PLANNING = ['autoware_route_handler', 'autoware_behavior_path_lane_change_module',
            'autoware_behavior_path_avoidance_by_lane_change_module',
            'autoware_behavior_path_external_request_lane_change_module', 'autoware_behavior_path_planner',
            'autoware_motion_velocity_obstacle_cruise_module', 'autoware_path_optimizer',
            'autoware_launch']
ASSETS = ['bringup', 'competition/competition_rule_monitor', 'lanelet_map_visualizer',
          'localization/localization_manager', 'tracking_prediction_evaluation',
          'control/trajectory_batch_runner', 'control/trajectory_generator']
LOGGER = '''import json, os, sys
with open(os.environ['MOCK_LOG'], 'a') as stream:
    stream.write(json.dumps(sys.argv[1:]) + '\\n')
sys.exit(17 if os.environ.get('MOCK_FAIL_STAGE') == sys.argv[1] else 0)
'''


@pytest.fixture
def workspace(tmp_path):
    def write(relative, content):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    wrapper = write('scripts/runtime/update_competition_runtime.sh', SCRIPT.read_text())
    write('planning_ws/install_isolated/setup.bash', ':\n')
    write('base_ws/install_aw149/setup.bash', ':\n')
    write('scripts/workspace/source_aw149.sh', ':\n')
    write('mock_logger.py', LOGGER)
    for path, stage in [('planning_ws/tools/build.sh', 'planning'),
                        ('scripts/runtime/install_traffic_light_candidate.sh', 'installer')]:
        write(path, f'exec /usr/bin/python3 "$MOCK_LOGGER" {stage} "$@"\n')
    write('scripts/map/prepare_kcity_v98.py', LOGGER.replace('sys.argv[1:]',
          "['prepare'] + sys.argv[1:]").replace("sys.argv[1] else 0", "'prepare' else 0"))
    write('scripts/map/prepare_kcity_v97.py', LOGGER.replace('sys.argv[1:]',
          "['prepare'] + sys.argv[1:]").replace("sys.argv[1] else 0", "'prepare' else 0"))
    write('scripts/map/prepare_kcity_v96.py', LOGGER.replace('sys.argv[1:]',
          "['prepare'] + sys.argv[1:]").replace("sys.argv[1] else 0", "'prepare' else 0"))
    write('scripts/map/prepare_kcity_v95.py', LOGGER.replace('sys.argv[1:]',
          "['prepare'] + sys.argv[1:]").replace("sys.argv[1] else 0", "'prepare' else 0"))
    write('scripts/map/prepare_kcity_v94.py', LOGGER.replace('sys.argv[1:]',
          "['prepare'] + sys.argv[1:]").replace("sys.argv[1] else 0", "'prepare' else 0"))
    for command in ['cmake', 'catkin_make_isolated']:
        path = write('mock_bin/' + command,
                     f'#!/bin/bash\nexec /usr/bin/python3 "$MOCK_LOGGER" {command} "$@"\n')
        path.chmod(0o755)
    for relative in ASSETS:
        package = relative.rsplit('/', 1)[-1]
        write(f'base_ws/build_aw149/{package}/CMakeCache.txt',
              f'CMAKE_HOME_DIRECTORY:INTERNAL={tmp_path}/base_ws/src/{relative}\n'
              f'CMAKE_INSTALL_PREFIX:PATH={tmp_path}/base_ws/install_aw149\n')
    env = dict(os.environ, MOCK_LOG=str(tmp_path / 'commands.jsonl'),
               MOCK_LOGGER=str(tmp_path / 'mock_logger.py'),
               PATH=str(tmp_path / 'mock_bin') + ':' + os.environ['PATH'])

    def run(*arguments, fail=None):
        current = dict(env)
        if fail:
            current['MOCK_FAIL_STAGE'] = fail
        result = subprocess.run(['bash', str(wrapper), *arguments], env=current,
                                capture_output=True, text=True)
        log = Path(env['MOCK_LOG'])
        return result, [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []

    return tmp_path, run


def test_default_builds_all_scoped_packages_then_installer_after_verifying_v98_map(workspace):
    root, run = workspace
    result, calls = run('--device', 'cpu', '--models-dir', str(root / 'models with spaces'))
    assert result.returncode == 0, result.stderr
    assert [call[0] for call in calls] == ['prepare', 'planning'] + ['cmake'] * 7 + ['installer']
    assert calls[0] == ['prepare']
    assert calls[1][1:] == ['--pkg', *PLANNING, '--make-args', '-j2', '-l2']
    assert calls[-1][1:] == ['--device', 'cpu', '--models-dir', str(root / 'models with spaces')]
    for call, relative in zip(calls[2:-1], ASSETS):
        assert call[1:] == ['--build', str(root / 'base_ws/build_aw149' / relative.rsplit('/', 1)[-1]),
                            '--target', 'install', '--', '-j2']
    for name in ('planning', 'traffic', 'tracking'):
        assert f'{name} map_version:=98' in result.stdout


@pytest.mark.parametrize('remote,expected', [(None, []), ('teammate_private', ['--remote', 'teammate_private'])])
def test_v94_provisions_before_build_and_map_remote_is_not_model_remote(workspace, remote, expected):
    _, run = workspace
    options = ['--map-version', '94', '--remote', 'model_private']
    if remote:
        options += ['--map-remote', remote]
    result, calls = run(*options)
    assert result.returncode == 0, result.stderr
    assert calls[0] == ['prepare', *expected]
    assert calls[1][0] == 'planning'
    assert calls[-1] == ['installer', '--remote', 'model_private']
    assert 'planning map_version:=94' in result.stdout


def test_missing_any_asset_cache_uses_one_scoped_catkin_fallback(workspace):
    root, run = workspace
    (root / 'base_ws/build_aw149/bringup/CMakeCache.txt').unlink()
    result, calls = run()
    assert result.returncode == 0, result.stderr
    assert [call[0] for call in calls] == ['prepare', 'planning', 'catkin_make_isolated', 'installer']
    fallback = calls[2]
    start = fallback.index('--pkg') + 1
    assert fallback[start:start + 7] == [relative.rsplit('/', 1)[-1] for relative in ASSETS]
    assert fallback[-3:] == ['--make-args', '-j2', '-l2']


@pytest.mark.parametrize('arguments', [
    ['--check'], ['--models-only'], ['--skip-build'], ['--unknown'],
    ['--map-version', '87'], ['--map-version', '88'], ['--map-version'], ['--device', 'oops'],
    ['--archive', '--device', 'cpu'], ['--map-remote', 'name:path'],
])
def test_invalid_or_partial_options_fail_before_any_action(workspace, arguments):
    _, run = workspace
    result, calls = run(*arguments)
    assert result.returncode != 0
    assert not calls
    assert '[INSTALLED]' not in result.stdout


@pytest.mark.parametrize('stage', ['prepare', 'planning', 'cmake', 'installer'])
def test_failed_stage_never_claims_install_complete(workspace, stage):
    _, run = workspace
    result, calls = run('--map-version', '94', fail=stage)
    assert result.returncode != 0
    assert calls[-1][0] == stage
    assert '[INSTALLED]' not in result.stdout
    assert 'planning map_version:=' not in result.stdout


@pytest.mark.parametrize('key', ['CMAKE_HOME_DIRECTORY', 'CMAKE_INSTALL_PREFIX'])
def test_foreign_cache_fails_before_preparation_even_if_another_cache_is_missing(workspace, key):
    root, run = workspace
    (root / 'base_ws/build_aw149/bringup/CMakeCache.txt').unlink()
    cache = root / 'base_ws/build_aw149/competition_rule_monitor/CMakeCache.txt'
    cache.write_text('\n'.join(f'{key}:PATH=/foreign/workspace' if line.startswith(key + ':') else line
                               for line in cache.read_text().splitlines()))
    result, calls = run('--map-version', '94')
    assert result.returncode != 0
    assert 'Refusing foreign/moved CMake cache' in result.stderr
    assert not calls


def test_existing_planning_cache_is_validated_before_any_action(workspace):
    root, run = workspace
    cache = root / 'planning_ws/build_isolated/autoware_route_handler/CMakeCache.txt'
    cache.parent.mkdir(parents=True)
    cache.write_text('CMAKE_HOME_DIRECTORY:INTERNAL=/foreign/planning\n')
    result, calls = run()
    assert result.returncode != 0
    assert not calls


@pytest.mark.parametrize('workspace_name,relative', [
    ('planning_ws', 'autoware_behavior_velocity_traffic_light_module'),
    ('base_ws', 'perception/traffic_light_detection'),
    ('base_ws', 'planning_addons/planning_scenario_validator'),
    ('base_ws', 'planning_addons/ajounice_planning_integration'),
    ('base_ws', 'planning_addons/competition_route_requester'),
    ('base_ws', 'perception_object_pipeline'),
])
def test_downstream_installer_cache_is_checked_before_provisioning(workspace, workspace_name, relative):
    root, run = workspace
    build = 'build_isolated' if workspace_name == 'planning_ws' else 'build_aw149'
    cache = root / workspace_name / build / relative.rsplit('/', 1)[-1] / 'CMakeCache.txt'
    cache.parent.mkdir(parents=True)
    cache.write_text('CMAKE_HOME_DIRECTORY:INTERNAL=/foreign/installer\n')
    result, calls = run('--map-version', '94')
    assert result.returncode != 0
    assert 'Refusing foreign/moved CMake cache' in result.stderr
    assert not calls


def test_existing_workspace_is_required(workspace):
    root, run = workspace
    (root / 'base_ws/install_aw149/setup.bash').unlink()
    result, calls = run()
    assert result.returncode != 0
    assert 'Existing workspace required' in result.stderr
    assert not calls


def test_help_explains_partial_modes_and_does_nothing(workspace):
    _, run = workspace
    result, calls = run('--help')
    assert result.returncode == 0
    assert '--check, --models-only and --skip-build are rejected' in result.stdout
    assert not calls
