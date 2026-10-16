"""Synthetic regression of the entire scaled-box QA geometry path."""
import itertools
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_audit import audit_capture
from report_base5_compound_evidence import _validate_native_geometry


def test_native_corners_feed_points_ground_clearance_and_converter_validator(tmp_path,monkeypatch):
    folder=tmp_path/'LIDAR_5';folder.mkdir()
    center=np.asarray([8.,0.,0.]);extent=np.asarray([2.,2.,2.])
    corners=np.asarray(list(itertools.product((-1.,1.),repeat=3)))*extent/2+center
    annotation='Obstacle '+' '.join(map(str,[*center,0.,0.,0.,1.,1.,1.,0.,0.,0.,0.,8500]))+'\n'
    eight='Obstacle '+' '.join(map(str,[*corners.ravel(),0.,0.,0.,0.,0.,0.,0.,8500]))+'\n'
    (folder/'one_Instance.txt').write_text(annotation)
    (folder/'one_Instance_8Points.txt').write_text(eight)
    np.asarray([[8.9,0.,0.,1.]],np.float32).tofile(folder/'one_Intensity.bin')
    (tmp_path/'scenario.json').write_text(json.dumps(dict(objectList=[dict(
        UNIQUEID=8500,DataID=40100001,scale=dict(x=2.,y=2.,z=2.))])))
    metadata=dict(runtime_actors=[dict(unique_id=8500,name='CargoBox',data_id=40100001,
        source_list='objectList',motion='static',training_class='component_annotation')],
        scale_probe=dict(parent_uid=8500,requested_scale=[2.,2.,2.]),
        spec=dict(family='obstacle',distance_bounds=[0.,40.]),lidar_to_ego=np.eye(4).tolist(),
        ground_support_check=dict(enabled=True))
    seen=[]
    def ground(points,boxes,*args,**kwargs):
        seen.append(boxes[0]['size'].tolist())
        return dict(actors=[dict(uid=8500,verified=True,required=True)],review_reasons=[])
    monkeypatch.setattr('base5_ground_support.ground_review',ground)
    result=audit_capture(tmp_path,metadata)
    component=result['components'][0]
    assert component['size_m']==[2.,2.,2.]
    assert component['native_summary_size_m']==[1.,1.,1.]
    assert component['lidar_points']==1
    assert seen==[[2.,2.,2.]]
    assert result['actual_annotation_clearances'][0]['annotation_surface_gap_m']==3.25
    (tmp_path/'metadata.json').write_text(json.dumps(metadata))
    _validate_native_geometry(tmp_path,result['components'])
    assert (folder/'one_Instance.txt').read_text()==annotation
    assert (folder/'one_Instance_8Points.txt').read_text()==eight
