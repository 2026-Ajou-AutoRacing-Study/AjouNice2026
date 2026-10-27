"""Read-only roslaunch integration: no master, publishers or running nodes required.

Run with AJOUNICE_TEST_LAUNCH_DUMP=1 after sourcing source_aw149.sh.
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
    # roslaunch --dump-params can return 0 even when parsing failed. Use the
    # configuration API so a rejected launch raises and exits nonzero.
    script = """import sys, yaml, roslaunch.config
config = roslaunch.config.load_config_default(
    [(sys.argv[1], sys.argv[2:])], None, verbose=False, assign_machines=False)
print(yaml.safe_dump({key: param.value for key, param in config.params.items()}))
"""
    return subprocess.run([sys.executable, '-c', script, str(ROOT / relative), *arguments],
                          capture_output=True, text=True, timeout=30)


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS)
def test_default_and_explicit_v99_are_accepted(entrypoint):
    for args in ((), ('map_version:=99',)):
        result = dump(entrypoint, *args)
        assert result.returncode == 0, result.stderr
        params = yaml.safe_load(result.stdout) or {}
        for key, value in params.items():
            if key.endswith(('/lanelet2_map_path', '/map_file', '/lanelet2_map_file')):
                assert str(value).endswith('lanelet2_map_v99.osm'), (key, value)


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS)
def test_old_version_is_rejected_at_launch_parse_time(entrypoint):
    result = dump(entrypoint, 'map_version:=98')
    assert result.returncode != 0
    assert '_require_v99_map_version' in result.stderr


@pytest.mark.parametrize('version', ['87', '94', '95', '96', '97', '100'])
def test_other_versions_cannot_override_the_competition_entrypoint(version):
    assert dump(I + 'competition_planning.launch', 'map_version:=' + version).returncode != 0


@pytest.mark.parametrize('entrypoint,argument', [
    (I + 'planning.launch', 'map_file'), (I + 'morai_planning_stack.launch', 'map_file'),
    (P + 'object_pipeline.launch', 'map_file'), (P + 'tracking_prediction.launch', 'map_file'),
    (P + 'object_prediction.launch', 'map_file'),
    (T + 'traffic_light_node.launch', 'lanelet2_map_file'),
    (T + 'traffic_light_v5_candidate.launch', 'lanelet2_map_file'),
])
def test_map_file_override_cannot_bypass_version_guard(entrypoint, argument):
    old = str(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v98.osm')
    result = dump(entrypoint, argument + ':=' + old)
    assert result.returncode != 0
    assert '_require_v99_map_file' in result.stderr


def test_standalone_loader_and_requester_override_cannot_bypass_contract():
    old_map = str(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v98.osm')
    old_config = str(ROOT / 'base_ws/src/planning_addons/competition_route_requester/config/competition_route_requester_v98.param.yaml')
    result = dump(I + 'map.launch', 'map_file:=' + old_map,
                  'map_sha256:=25087b24cf9c2a65f366b33b885bcc07ff644ad7b241d8f671a24a03460dc7d0')
    assert result.returncode != 0
    assert '_require_v99_map_contract' in result.stderr
    result = dump(I + 'planning.launch', 'route_requester_config_file:=' + old_config)
    assert result.returncode != 0
    assert '_require_v99_requester_config' in result.stderr


@pytest.mark.parametrize('version', ['87', '94', '95', '96', '97', '98'])
def test_old_named_planning_entrypoints_are_blocked(version):
    result = dump(I + 'competition_planning_v' + version + '.launch')
    assert result.returncode != 0
    assert '_require_v99_map_version' in result.stderr
