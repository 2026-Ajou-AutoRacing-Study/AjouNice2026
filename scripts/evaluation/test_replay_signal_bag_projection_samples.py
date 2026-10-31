"""Pure sample replay helpers; no ROS, GPU, bag or existing artifacts."""
import json
from pathlib import Path
from unittest.mock import patch

import replay_signal_bag_projection_samples as replay


def test_box_adapter_preserves_actual_node_five_tuple_contract():
    boxes=replay.selection_boxes([[100.,50.,180.,70.,.9]],1280,720)
    assert boxes[0][4]==(100.,50.,180.,70.,.9)
    assert boxes[0][0]==140./1280
    roi=dict(head_id='C1256W000052',group_id=201326,eligible=True,
             bbox_xyxy=[100.,50.,180.,70.],gate_xyxy=[90.,40.,190.,80.],padding_xy_px=[10.,10.])
    selected,selection=replay.associate_boxes(boxes,[roi],201326)
    assert selected==boxes[0]
    assert selection['head_id']=='C1256W000052'


def test_adjudication_explicitly_enables_production_near_rescue_not_library_off_default(tmp_path):
    root=tmp_path;directory=root/'planner_logs/analysis/source';directory.mkdir(parents=True)
    projection=directory/'projection.json'
    roi=dict(head_id='C1256W000052',group_id=201326,eligible=True,depth_m=10.,
             bbox_xyxy=[100.,50.,180.,70.],gate_xyxy=[90.,40.,190.,80.],padding_xy_px=[10.,10.])
    missing=dict(status='missing',reason='no_unambiguous_route_detection',rejected=[])
    matched=dict(status='matched',head_id='C1256W000052',reason='map_head_association',roi=roi)
    projection.write_text(json.dumps(dict(records=[dict(image_sha256='jpeg',group_id=201326,
        modes={'current3':dict(rois=[roi])})])))
    inference=directory/'inference.json'
    inference.write_text(json.dumps(dict(source_projection=str(projection),limitations=[],records=[
        dict(t=135.,section='D',image_path='images/a.jpg',image_sha256='jpeg',modes={'current3':dict(
            projection_status='ready',selection=missing,diagnostic_combined_selection=matched,
            diagnostic_cnn=dict(known=True,reason='observed_lamps'))})])))
    output=root/'planner_logs/analysis/new'
    with patch.object(replay,'ROOT',root):replay.adjudicate(inference,output)
    report=json.loads((output/'gated_rescue.json').read_text())
    assert report['counts']['current3:retry']==1
    assert report['counts']['current3:recovered']==1
