import copy

import pytest

from prepare_signal_map_candidate_trials import signal_corridor_campaign


def test_corridor_uses_existing_d_goal_and_preserves_original_full_course():
    points = [dict(x=x, y=0, z=1, yaw_deg=0) for x in range(6)]
    full = dict(campaign_name='full', default_case=dict(goal=points[-1], waypoints=points[:-1], timeout_sec=600))
    focused = dict(default_case=dict(goal=points[3]))
    before = copy.deepcopy(full)
    result = signal_corridor_campaign(full, focused)
    assert len(result['campaign_name']) <= 60
    assert result['default_case']['goal'] == points[3]
    assert result['default_case']['waypoints'] == points[:3]
    assert full == before
    result['default_case']['waypoints'][0]['x'] = 999
    assert full == before


def test_missing_or_duplicate_d_goal_is_rejected():
    goal = dict(x=1, y=2, z=3, yaw_deg=4)
    for points in ([], [goal], [goal, goal]):
        with pytest.raises(ValueError):
            signal_corridor_campaign(dict(campaign_name='full', default_case=dict(waypoints=points)),
                                     dict(default_case=dict(goal=goal)))
