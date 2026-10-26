from pathlib import Path

import yaml

from retest_signal_latency import retry_config


def test_retry_configuration_keeps_runtime_policy_and_source_immutable(tmp_path):
    source=tmp_path/'A.yaml'
    original=dict(campaign_name='original',resume=True,
        stack=dict(traffic_light=True,traffic_light_profile='v5_candidate',pose_source='localization',detector='od_gt'),
        default_case=dict(goal=dict(x=1,y=2)),
        recording=dict(minimum_free_disk_gib=4.,selected_topics=['/Ego_topic','/sensor/camera/front/compressed']))
    source.write_text(yaml.safe_dump(original))
    before=source.read_bytes()
    result=retry_config(source,'A')
    assert source.read_bytes()==before
    assert result['stack']==original['stack']
    assert result['default_case']==original['default_case']
    assert result['recording']['minimum_free_disk_gib']==4.
    assert result['resume'] is False
    assert '/Ego_topic' in result['recording']['selected_topics']
    assert '/sensor/camera/front/compressed' not in result['recording']['selected_topics']
