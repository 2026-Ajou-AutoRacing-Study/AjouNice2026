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
