from pathlib import Path

import pytest
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


def test_opt_in_original_jpegs_do_not_change_runtime_or_storage_guard(tmp_path):
    source=tmp_path/'B.yaml'
    original=dict(campaign_name='original',resume=False,stack=dict(traffic_light=True),
        recording=dict(minimum_free_disk_gib=4.,selected_topics=['/Ego_topic']))
    source.write_text(yaml.safe_dump(original))
    result=retry_config(source,'B',record_front_jpeg=True)
    assert result['stack']==original['stack']
    assert result['recording']['minimum_free_disk_gib']==4.
    assert result['recording']['selected_topics'].count('/sensor/camera/front/compressed')==1


def test_short_trial_storage_budget_remains_explicit_and_bounded(tmp_path):
    source=tmp_path/'D.yaml'
    original=dict(campaign_name='original',resume=False,recording=dict(
        minimum_free_disk_gib=4.,selected_topics=['/Ego_topic']))
    source.write_text(yaml.safe_dump(original))
    before=source.read_bytes()
    assert retry_config(source,'D',minimum_free_disk_gib=1.)['recording']['minimum_free_disk_gib']==1.
    assert source.read_bytes()==before
    for value in (0.,.5,float('nan')):
        with pytest.raises(ValueError):
            retry_config(source,'D',minimum_free_disk_gib=value)
