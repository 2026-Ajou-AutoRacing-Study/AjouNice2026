"""Read-only launch regression: no master or nodes are started.

Run with AJOUNICE_TEST_LAUNCH_DUMP=1 after sourcing source_aw149.sh. The v101
default stays selected, but explicit valid versions/paths are not parse-blocked.
Actual file/checksum, parsing and coordinate checks remain separate checks.
"""
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
I = 'base_ws/src/planning_addons/ajounice_planning_integration/launch/'
P = 'base_ws/src/perception_object_pipeline/launch/'
T = 'base_ws/src/perception/traffic_light_detection/launch/'
ENTRYPOINTS = [I + n for n in ('competition_planning.launch', 'planning.launch', 'morai_planning_stack.launch')]
ENTRYPOINTS += [P + n for n in ('object_pipeline.launch', 'tracking_prediction.launch', 'object_prediction.launch')]
ENTRYPOINTS += [T + n for n in ('traffic_light_node.launch', 'traffic_light_v5_candidate.launch')]
pytestmark = pytest.mark.skipif(os.environ.get('AJOUNICE_TEST_LAUNCH_DUMP') != '1',
                                reason='Set AJOUNICE_TEST_LAUNCH_DUMP=1 in the AW149 environment')


def dump(relative, *arguments):
    # Nested includes must resolve the candidate source, not old installed files.
    packages = [ROOT / I / '..', ROOT / P / '..', ROOT / T / '..']
    env = dict(os.environ, AJOUNICE_ROOT=str(ROOT), ROS_PACKAGE_PATH=':'.join(str(path.resolve()) for path in packages)
               + ':' + os.environ.get('ROS_PACKAGE_PATH', ''))
    script = """import sys, yaml, roslaunch.config, roslaunch.substitution_args
from pathlib import Path
# ROS's $(find pkg)/resource lookup prefers installed shares even when
# ROS_PACKAGE_PATH points at source. Override only these candidate packages in
# this configuration-only subprocess; do not install or alter the live overlay.
import os
root = Path(os.environ['AJOUNICE_ROOT'])
sources = {
    'ajounice_planning_integration': root / 'base_ws/src/planning_addons/ajounice_planning_integration',
    'perception_object_pipeline': root / 'base_ws/src/perception_object_pipeline',
    'traffic_light_detection': root / 'base_ws/src/perception/traffic_light_detection',
}
original_find = roslaunch.substitution_args._find
def source_find(resolved, command, args, context):
    if len(args) == 1 and args[0] in sources:
        return resolved.replace('$(' + command + ')', str(sources[args[0]]))
    return original_find(resolved, command, args, context)
roslaunch.substitution_args._find = source_find
config = roslaunch.config.load_config_default(
    [(sys.argv[1], sys.argv[2:])], None, verbose=False, assign_machines=False)
print(yaml.safe_dump({key: param.value for key, param in config.params.items()}))
"""
    return subprocess.run([sys.executable, '-c', script, str(ROOT / relative), *arguments],
                          env=env, capture_output=True, text=True, timeout=30)


def accepted(result):
    assert result.returncode == 0, result.stderr
    return yaml.safe_load(result.stdout) or {}


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS)
def test_default_and_explicit_v101_are_accepted(entrypoint):
    for args in ((), ('map_version:=101',)):
        params = accepted(dump(entrypoint, *args))
        for key, value in params.items():
            if key.endswith(('/lanelet2_map_path', '/map_file', '/lanelet2_map_file')):
                assert str(value).endswith('lanelet2_map_v101.osm'), (key, value)


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS)
def test_explicit_available_version_is_not_artificially_rejected(entrypoint):
    params = accepted(dump(entrypoint, 'map_version:=98'))
    for key, value in params.items():
        if key.endswith(('/lanelet2_map_path', '/map_file', '/lanelet2_map_file')):
            assert str(value).endswith('lanelet2_map_v98.osm'), (key, value)


@pytest.mark.parametrize('entrypoint,argument', [
    (I + 'planning.launch', 'map_file'), (I + 'morai_planning_stack.launch', 'map_file'),
    (P + 'object_pipeline.launch', 'map_file'), (P + 'tracking_prediction.launch', 'map_file'),
    (P + 'object_prediction.launch', 'map_file'),
    (T + 'traffic_light_node.launch', 'lanelet2_map_file'),
    (T + 'traffic_light_v5_candidate.launch', 'lanelet2_map_file'),
])
def test_explicit_map_path_can_be_selected(entrypoint, argument):
    old = str(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v98.osm')
    accepted(dump(entrypoint, argument + ':=' + old))


def test_standalone_loader_accepts_matching_explicit_map_and_checksum():
    old_map = str(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v98.osm')
    params = accepted(dump(I + 'map.launch', 'map_file:=' + old_map,
                          'map_sha256:=25087b24cf9c2a65f366b33b885bcc07ff644ad7b241d8f671a24a03460dc7d0'))
    assert params['/map_projector_publisher/map_file'] == old_map
    assert params['/map_projector_publisher/map_sha256'] == (
        '25087b24cf9c2a65f366b33b885bcc07ff644ad7b241d8f671a24a03460dc7d0')


def test_new_version_can_use_explicit_existing_reviewed_assets():
    map_path = str(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v101.osm')
    requester = str(ROOT / 'base_ws/src/planning_addons/competition_route_requester/config/competition_route_requester_v101.param.yaml')
    params = accepted(dump(I + 'planning.launch', 'map_version:=102',
                          'map_file:=' + map_path, 'route_requester_config_file:=' + requester,
                          'map_sha256:=574d0db7ba7aa2fbca55253e1ed467bdb4bfb2afeea78dd0b760227d47edeb29'))
    assert params['/map_projector_publisher/map_file'] == map_path


def test_missing_required_parameter_file_still_fails_configuration():
    result = dump(I + 'planning.launch', 'traffic_light_judging_lines_config:=/missing/reviewed-profile.yaml')
    assert result.returncode != 0
    assert '_require_v101' not in result.stderr
    assert 'reviewed-profile.yaml' in result.stderr
