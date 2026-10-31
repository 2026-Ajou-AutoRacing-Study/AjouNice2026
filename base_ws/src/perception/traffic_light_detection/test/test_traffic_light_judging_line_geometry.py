"""Original-MGeo judging geometry; no simulator, ROS graph or map modification."""
import hashlib
import json
import math
from pathlib import Path

import pytest
import yaml


ROOT=Path(__file__).resolve().parents[5]
CONFIG=ROOT/'base_ws/src/perception/traffic_light_detection/config/traffic_signal_judging_lines.yaml'
GROUPS=dict(A=201351,B=201365,C=201290,D=201326)


def configuration():
    return yaml.safe_load(CONFIG.read_text())


def test_four_original_lines_not_virtual_stopping_margins():
    data=configuration()
    assert data['signal_judging_line_metadata']['coordinate_frame']=='map'
    lines=data['signal_judging_lines']
    assert len(lines)==4
    assert {row['regulatory_id'] for row in lines}==set(GROUPS.values())
    for row in lines:
        assert set(row)=={'regulatory_id','line_xy','approach_xy'}
        assert len(row['line_xy']) in (4,6)
        assert all(type(x) in (int,float) and math.isfinite(x) for x in row['line_xy'])
        assert math.isclose(math.hypot(*row['approach_xy']),1.,abs_tol=1e-5)


def test_exact_original_mgeo_type530_provenance_when_assets_available():
    data=configuration();metadata=data['signal_judging_line_metadata']
    source=ROOT/metadata['source_file']
    if not source.exists():
        pytest.skip('Original large MGeo asset is not installed on this machine')
    assert hashlib.sha256(source.read_bytes()).hexdigest()==metadata['source_sha256']
    global_info=source.with_name('global_info.json')
    assert hashlib.sha256(global_info.read_bytes()).hexdigest()==metadata['source_global_info_sha256']
    boundaries={row['idx']:row for row in json.loads(source.read_text())}
    by_id={row['regulatory_id']:row for row in data['signal_judging_lines']}
    tx,ty,_=metadata['map_from_MGeo_translation_xyz']
    for mission,paint_id in metadata['source_paint_ids'].items():
        source_line=boundaries[paint_id]
        assert source_line['lane_type']==[530]
        actual=by_id[GROUPS[mission]]['line_xy']
        expected=[value for p in source_line['points'] for value in (p[0]+tx,p[1]+ty)]
        assert actual==pytest.approx(expected,abs=1e-10)


def test_coordinate_transform_matches_existing_static_frame_contract():
    metadata=configuration()['signal_judging_line_metadata']
    transform=yaml.safe_load((ROOT/'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
    assert transform['parent']=='map' and transform['child']=='morai_map'
    assert transform['rpy']==[0.,0.,0.]
    assert transform['xyz']==pytest.approx(metadata['map_from_MGeo_translation_xyz'],abs=1e-8)


def test_approach_direction_matches_current_signal_static_geometry():
    physical=json.loads((CONFIG.parent/'traffic_light_map_geometry.json').read_text())
    headings={row['group_id']:row['heading'] for row in physical['heads'] if row.get('group_id') in GROUPS.values()}
    for row in configuration()['signal_judging_lines']:
        radians=math.radians(headings[row['regulatory_id']]+180.)
        assert row['approach_xy']==pytest.approx([math.cos(radians),math.sin(radians)],abs=2e-6)
