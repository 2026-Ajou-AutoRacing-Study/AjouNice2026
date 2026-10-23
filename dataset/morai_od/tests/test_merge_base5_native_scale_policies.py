import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_asset_scale as scale
from base5_capture_safety import sha256
from merge_base5_native_scale_policies import merge


def policy(tmp_path,name):
    evidence=tmp_path/(name+'_synthetic_evidence.json')
    evidence.write_text('{"test_fixture_only":true}')
    value=scale.candidate_policy([2,0,1])
    value['assets']={name:value['assets'][name]}
    row=next(iter(value['assets'][name]['profiles'].values()))
    row.update(status='qualified',native_geometry=dict(bbox_size_m=[1.,2.,1.],
        pivot_to_bbox_center_local_m=[0.,0.,.5],pivot_height_above_ground_m=0.),
        native_qualification=dict(observations=2,semantic_visual_review='accepted',
            contact_review='passed',sensor_gt_review='passed',axis_mapping_verified=True,
            single_component_obstacle_verified=True,
            evidence=[dict(path=str(evidence),sha256=sha256(evidence))]))
    value['assets'][name]['profiles']={'fixture':row}
    source=tmp_path/(name+'.json');source.write_text(json.dumps(value))
    return source,value


def test_disjoint_policies_preserve_approved_profiles(tmp_path):
    a,first=policy(tmp_path,'CargoBox');b,second=policy(tmp_path,'WoodBox')
    out=merge([a,b],tmp_path/'merged.json')
    value=json.loads(out.read_text())
    assert value['assets']==dict(first['assets'],**second['assets'])
    assert len(value['qualification']['source_policies'])==2
    assert not value['qualification']['profiles_changed']


def test_overlap_and_candidate_cannot_be_silently_adopted(tmp_path):
    a,value=policy(tmp_path,'CargoBox')
    with pytest.raises(scale.ScaleContractError):merge([a,a],tmp_path/'overlap.json')
    value=copy.deepcopy(value);value['assets']['CargoBox']['profiles']['fixture']['status']='candidate'
    a.write_text(json.dumps(value))
    with pytest.raises(scale.ScaleContractError):merge([a],tmp_path/'candidate.json')
    assert not (tmp_path/'candidate.json').exists()
