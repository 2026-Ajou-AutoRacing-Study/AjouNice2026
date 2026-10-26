import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from shift_signal_stop_lines import GROUPS, shifted_map


def fixture():
    nodes = ''.join(f'<node id="{n}" lat="37.24" lon="126.77"><tag k="ele" v="28.4" /></node>'
                    for n in range(1, 9))
    ways = ''.join(f'<way id="{10+i}"><nd ref="{2*i+1}"/><nd ref="{2*i+2}"/>'
                   '<tag k="subtype" v="stop_line"/></way>' for i in range(4))
    relations = ''.join(f'<relation id="{gid}"><member type="way" ref="{10+i}" role="ref_line"/></relation>'
                        for i, gid in enumerate(GROUPS.values()))
    return ('<osm>'+nodes+ways+relations+'</osm>').encode(), {
        'heads': [dict(group_id=gid, heading=-90.) for gid in GROUPS.values()]}


def test_only_selected_exclusive_nodes_move():
    raw, geometry = fixture()
    output, report = shifted_map(raw, dict(A=.75, C=.75), geometry)
    assert report['changed_node_ids'] == [1, 2, 5, 6]
    before, after = ET.fromstring(raw), ET.fromstring(output)
    for x, y in zip(before, after):
        if x.tag != 'node' or int(x.get('id')) not in [1, 2, 5, 6]:
            assert ET.tostring(x) == ET.tostring(y)
        else:
            assert x.get('id') == y.get('id')
            assert ET.tostring(x.find('tag')) == ET.tostring(y.find('tag'))
            assert float(y.get('lat')) < float(x.get('lat'))


@pytest.mark.parametrize('shift', [dict(A=-1), dict(A=float('nan')), dict(A=True), dict(E=1)])
def test_invalid_shift_rejected(shift):
    raw, geometry = fixture()
    with pytest.raises(ValueError):
        shifted_map(raw, shift, geometry)


def test_road_shared_nodes_never_move():
    raw, geometry = fixture()
    raw = raw.replace(b'</osm>', b'<way id="99"><nd ref="1"/></way></osm>')
    with pytest.raises(ValueError, match='shared'):
        shifted_map(raw, dict(A=.75), geometry)


def test_released_v87_original_is_unchanged_and_delta_is_four_nodes():
    root = Path(__file__).resolve().parents[2]
    source = root/'kcity_map/lanelet_map/lanelet2_map_v87.osm'
    geometry = json.loads((root/'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json').read_text())
    raw = source.read_bytes()
    output, report = shifted_map(raw, dict(A=.75, C=.75), geometry)
    assert source.read_bytes() == raw
    assert report['source_sha256'] == '70f2cdf7ee9dee3333d6553787525890c0a73c4fbcdfdcc9fc95a5c92c687575'
    assert report['changed_node_ids'] == [92992, 92993, 93034, 93035]
    assert report['candidate_sha256'] == '3cfb4dde4dbbbb44fc86f4e45c747576565793db48b04a631a61ab34af265d44'
    assert output != raw


def test_b_d_visibility_candidate_has_only_eight_stop_nodes_changed():
    root = Path(__file__).resolve().parents[2]
    raw = (root/'kcity_map/lanelet_map/lanelet2_map_v87.osm').read_bytes()
    geometry = json.loads((root/'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json').read_text())
    output, report = shifted_map(raw, dict(A=.75, B=2., C=.75, D=2.), geometry)
    assert report['changed_node_ids'] == [92992,92993,92998,92999,93020,93021,93034,93035]
    assert report['candidate_sha256'] == '49405b0c172e390fc54028d8d03ef848b3fca612ce3a99f94e24dbdcb3cb40d2'
    assert len(ET.fromstring(raw)) == len(ET.fromstring(output))


@pytest.mark.parametrize('b_shift,expected_sha', [
    (2.5, 'd10aa2b03245ae687ef62d88e977a2c425e9c03f1c52d4e5a05c54f1a453fb8a'),
    (3.0, '5c395331fc2c771ccbcdb2c753f10a76dad416fbae265f81f22045975339bb3e')])
def test_refined_visibility_candidate_does_not_move_roads_or_old_map(b_shift, expected_sha):
    root = Path(__file__).resolve().parents[2]
    source = root/'kcity_map/lanelet_map/lanelet2_map_v87.osm'
    raw = source.read_bytes()
    geometry = json.loads((root/'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json').read_text())
    output, report = shifted_map(raw, dict(A=.75, B=b_shift, C=.75, D=2.75), geometry)
    assert source.read_bytes() == raw
    assert report['changed_node_ids'] == [92992,92993,92998,92999,93020,93021,93034,93035]
    assert report['candidate_sha256'] == expected_sha
    assert [x['upstream_shift_m'] for x in report['changes']] == [.75,b_shift,.75,2.75]
    assert len(ET.fromstring(raw)) == len(ET.fromstring(output))


def test_abcd_alignment_v93_changes_only_qualified_stop_nodes():
    root = Path(__file__).resolve().parents[2]
    raw = (root/'kcity_map/lanelet_map/lanelet2_map_v87.osm').read_bytes()
    geometry = json.loads((root/'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json').read_text())
    output, report = shifted_map(raw, dict(A=.75,B=2.6,C=.35,D=2.6), geometry)
    assert report['candidate_sha256'] == '363f107d6a1460af6a71c1dc0c129f73f7a1da049686f5df159bf557cd3133e3'
    assert report['changed_node_ids'] == [92992,92993,92998,92999,93020,93021,93034,93035]
    assert [c['upstream_shift_m'] for c in report['changes']] == [.75,2.6,.35,2.6]
    # The builder's byte-exact reversal check also guards lane boundaries,
    # successors, IDs, elevation and every other non-target source byte.
    assert len(ET.fromstring(raw)) == len(ET.fromstring(output))
