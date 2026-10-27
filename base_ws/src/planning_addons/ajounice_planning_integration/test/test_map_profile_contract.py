"""No live nodes: explicit map selection must keep every consumer consistent."""
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
LAUNCH = PACKAGE / 'launch'


def parsed(path):
    return ET.parse(path).getroot()


def arguments(path):
    return {arg.get('name'): arg.get('default') for arg in parsed(path).findall('arg')}


def included(path, suffix):
    include = next(node for node in parsed(path).iter('include') if node.get('file').endswith(suffix))
    return {arg.get('name'): arg.get('value') for arg in include.findall('arg')}


def test_planning_defaults_use_v98_and_version_selects_route_and_map_together():
    path = LAUNCH / 'planning.launch'
    args = arguments(path)
    assert args['map_version'] == '98'
    assert args['map_file'].endswith('/lanelet2_map_v$(arg map_version).osm')
    assert args['route_requester_config_file'].endswith('/competition_route_requester_v$(arg map_version).param.yaml')
    assert args['map_sha256'].startswith('$(eval {')
    assert "[str(arg('map_version'))]" in args['map_sha256']
    for version, checksum in ((94, '3d11cd820b2f4d755960ae565dbc267e4cf5826307ef2df1149e10922c138cbf'), (95, 'a9d931eca722ab81596bce0685147b337dd7fdc3320ebeae13d0ff03ef2f2b6e'), (96, '98d0bcbd382212176a9e9efa13c778b1009e27794ef57681d977d957c9708ffd'), (97, 'da22c593a980a8fe02e136a004e1918ccf46a82381024854a4fa344eb992e0d9'), (98, '25087b24cf9c2a65f366b33b885bcc07ff644ad7b241d8f671a24a03460dc7d0')):
        assert "'%s': '%s'" % (version, checksum) in args['map_sha256']
    requester = included(path, '/competition_route_requester.launch')
    assert requester['config_file'] == '$(arg route_requester_config_file)'
    publisher = (PACKAGE / 'scripts/map_projector_publisher').read_text()
    assert "hashlib.sha256(map_path.read_bytes()).hexdigest() != map_sha" in publisher


def test_competition_entrypoints_preserve_the_explicit_version():
    ordinary = LAUNCH / 'competition_planning.launch'
    assert arguments(ordinary)['map_version'] == '98'
    assert included(ordinary, '/planning.launch')['map_version'] == '$(arg map_version)'
    for version in (87, 94, 95, 96, 97, 98):
        assert included(LAUNCH / ('competition_planning_v%d.launch' % version),
                        '/planning.launch')['map_version'] == str(version)


def test_fullstack_uses_one_selected_map_for_planning_prediction_and_signal():
    path = LAUNCH / 'morai_planning_stack.launch'
    assert arguments(path)['map_version'] == '98'
    for suffix, key in (('/planning.launch', 'map_file'),
                        ('/object_pipeline.launch', 'map_file'),
                        ('/traffic_light_node.launch', 'lanelet2_map_file')):
        forwarded = included(path, suffix)
        assert forwarded[key] == '$(arg map_file)'
        assert forwarded['map_version'] == '$(arg map_version)'


def test_separate_terminal_consumers_accept_the_same_version_argument():
    signal = ROOT / 'base_ws/src/perception/traffic_light_detection/launch'
    pipeline = ROOT / 'base_ws/src/perception_object_pipeline/launch'
    for path, key in ((signal/'traffic_light_node.launch', 'lanelet2_map_file'),
                      (signal/'traffic_light_v5_candidate.launch', 'lanelet2_map_file'),
                      (pipeline/'object_pipeline.launch', 'map_file'),
                      (pipeline/'tracking_prediction.launch', 'map_file'),
                      (pipeline/'object_prediction.launch', 'map_file')):
        args = arguments(path)
        assert args['map_version'] == '98'
        assert args[key].endswith('/lanelet2_map_v$(arg map_version).osm')
