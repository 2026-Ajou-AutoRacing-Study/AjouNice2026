"""Small synthetic three-way tests; no runtime map, ROS or remote mutation."""
import re
import xml.etree.ElementTree as ET

import pytest

from merge_signal_stop_lines import merge_stop_lines
from shift_signal_stop_lines import shifted_map
from test_shift_signal_stop_lines import fixture as stop_fixture


ROAD_NODES = (
    b'<node id="101" lat="37.25" lon="126.78"><tag k="ele" v="28.4" /></node>'
    b'<node id="102" lat="37.251" lon="126.781"><tag k="ele" v="28.5" /></node>'
)
ROAD_WAY = (b'<way id="99"><nd ref="101"/><nd ref="102"/>'
            b'<tag k="type" v="road_border"/></way>')


def maps():
    raw, geometry = stop_fixture()
    raw = raw.replace(b'<osm>', b'<osm><MetaInfo format_version="1" map_version="87" validation_version=""/>')
    for gid in (201351, 201365, 201290, 201326):
        source = ('<relation id="%s">' % gid).encode()
        raw = raw.replace(source, source+b'<tag k="subtype" v="traffic_light"/>')
    raw = raw.replace(b'</osm>', ROAD_NODES+ROAD_WAY+b'</osm>')
    ours, _ = shifted_map(raw, dict(A=.75, B=2.6, C=.35, D=2.6), geometry)
    return raw, ours


def indexed(raw):
    root = ET.fromstring(raw)
    return root, {(e.tag, e.get('id')): e for e in root if e.get('id') is not None}


def node_block(raw, ident):
    return re.search(rb'<node\b(?=[^>]*\bid="'+str(ident).encode()+rb'")[^>]*>.*?</node>',
                     raw, re.S).group()


def test_incoming_road_change_and_added_objects_are_preserved_exactly():
    base, ours = maps()
    new_way = ROAD_WAY.replace(b'road_border', b'line_thin')
    added = (b'<node id="103" lat="37.252" lon="126.782"><tag k="ele" v="29"/></node>'
             b'<way id="100"><nd ref="102"/><nd ref="103"/><tag k="type" v="virtual"/></way>')
    incoming = base.replace(b'<osm>', b'<osm generator="teammate">').replace(
        b'map_version="87"', b'map_version="88-2"').replace(ROAD_WAY, new_way).replace(
        b'</osm>', added+b'</osm>')
    original_inputs = (base, ours, incoming)
    output, report = merge_stop_lines(base, ours, incoming, 94)
    assert (base, ours, incoming) == original_inputs
    root, result = indexed(output)
    _, own = indexed(ours)
    _, theirs = indexed(incoming)
    assert root.get('generator') == 'teammate'
    assert root.find('MetaInfo').get('map_version') == '94'
    assert new_way in output and added in output
    assert report['changed_node_ids'] == list(range(1, 9))
    for ident in range(1, 9):
        assert node_block(output, ident) == node_block(ours, ident)
    for key in [('node', '101'), ('node', '102'), ('node', '103'), ('way', '99'), ('way', '100')]:
        assert ET.tostring(result[key]) == ET.tostring(theirs[key])
    assert set(result) == set(theirs)
    assert result[('node', '1')].attrib == own[('node', '1')].attrib


def test_conflicting_incoming_stop_coordinate_is_not_overwritten():
    base, ours = maps()
    incoming = base.replace(b'<node id="1" lat="37.24"', b'<node id="1" lat="37.241"')
    with pytest.raises((ValueError, AssertionError)):
        merge_stop_lines(base, ours, incoming, 94)


def test_our_non_target_change_is_rejected():
    base, ours = maps()
    ours = ours.replace(ROAD_WAY, ROAD_WAY.replace(b'road_border', b'line_thin'))
    with pytest.raises((ValueError, AssertionError)):
        merge_stop_lines(base, ours, base, 94)


def test_our_non_coordinate_stop_property_change_is_rejected():
    base, ours = maps()
    before = node_block(ours, 1)
    ours = ours.replace(before, before.replace(b'v="28.4"', b'v="31.0"'), 1)
    with pytest.raises((ValueError, AssertionError)):
        merge_stop_lines(base, ours, base, 94)


def test_incoming_stop_shared_with_road_is_rejected():
    base, ours = maps()
    incoming = base.replace(ROAD_WAY, ROAD_WAY.replace(b'<nd ref="101"/>', b'<nd ref="1"/>'))
    with pytest.raises((ValueError, AssertionError)):
        merge_stop_lines(base, ours, incoming, 94)


def test_already_applied_stop_patch_is_not_shifted_twice():
    base, ours = maps()
    incoming = ours.replace(b'map_version="87"', b'map_version="88-2"').replace(
        ROAD_WAY, ROAD_WAY.replace(b'road_border', b'line_thin'))
    output, _ = merge_stop_lines(base, ours, incoming, 94)
    for ident in range(1, 9):
        assert node_block(output, ident) == node_block(incoming, ident)
    assert ROAD_WAY.replace(b'road_border', b'line_thin') in output
    assert ET.fromstring(output).find('MetaInfo').get('map_version') == '94'


@pytest.mark.parametrize('source_index', [0, 1, 2])
@pytest.mark.parametrize('defect', ['duplicate', 'dangling'])
def test_duplicate_ids_and_dangling_references_are_rejected_in_every_source(source_index, defect):
    base, ours = maps()
    inputs = [base, ours, base]
    addition = (node_block(inputs[source_index], 101) if defect == 'duplicate' else
                b'<way id="998"><nd ref="999999"/></way>')
    inputs[source_index] = inputs[source_index].replace(b'</osm>', addition+b'</osm>')
    with pytest.raises((ValueError, AssertionError)):
        merge_stop_lines(*inputs, 94)


def test_target_ref_line_identity_change_is_rejected():
    base, ours = maps()
    incoming = base.replace(b'role="ref_line"', b'role="different_role"', 1)
    with pytest.raises((ValueError, AssertionError)):
        merge_stop_lines(base, ours, incoming, 94)


def test_local_coordinate_fields_follow_our_patch_without_changing_elevation():
    base, _ = maps()
    base = base.replace(b'<node id="1" lat="37.24" lon="126.77">',
        b'<node id="1" lat="37.24" lon="126.77"><tag k="local_x" v="1.0"/><tag k="local_y" v="2.0"/>')
    _, geometry = stop_fixture()
    ours, _ = shifted_map(base, dict(A=.75, B=2.6, C=.35, D=2.6), geometry)
    output, _ = merge_stop_lines(base, ours, base, 94)
    _, result = indexed(output)
    _, own = indexed(ours)
    assert ET.tostring(result[('node', '1')]) == ET.tostring(own[('node', '1')])
    assert b'<tag k="ele" v="28.4" />' in node_block(output, 1)


@pytest.mark.parametrize('version', [0, -1, True, 94.5])
def test_map_version_must_be_an_explicit_positive_integer(version):
    base, ours = maps()
    with pytest.raises((ValueError, AssertionError)):
        merge_stop_lines(base, ours, base, version)
