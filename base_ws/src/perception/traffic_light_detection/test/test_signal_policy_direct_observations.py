"""Production observed lamps and identity-bound 200ms detection gaps; no ROS."""
from pathlib import Path
import sys
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from signal_phase_policy import SignalPhasePolicy
from signal_smoother import SignalSmoother

R, Y, G, L = (("red", "circle", None), ("yellow", "circle", None),
              ("green", "circle", None), ("green", "arrow", True))
GROUPS = (201351, 201365, 201290, 201326)


@pytest.fixture
def policy():
    return SignalPhasePolicy.from_yaml(str(PACKAGE/'config/traffic_signal_profiles.yaml'))


def step(policy, raw, now=10., source=10_000_000_000, group=201351,
         head='head_A', eligible=False, confidence=.95, hold=.2):
    normalized, reason = policy.apply(raw, group, observed_only=True)
    result = policy.process_normalized(normalized, reason, group, now, confidence,
        SignalSmoother(7, 4), source_stamp_ns=source, use_smoothing=False,
        observation_head_id=head, detection_miss_hold_eligible=eligible,
        detection_miss_hold_sec=hold)
    return result, policy.output_observation_metadata(group)


@pytest.mark.parametrize('group', GROUPS)
def test_observed_color_is_never_replaced_by_different_color(policy, group):
    for raw in ([R], [Y], [G]):
        out, reason = policy.apply(raw, group, observed_only=True)
        assert out == raw
        assert reason.endswith(':observed')


@pytest.mark.parametrize('group', (201290, 201326))
def test_arrow_does_not_invent_green_circle_or_red(policy, group):
    assert policy.apply([L], group, observed_only=True)[0] == [L]


@pytest.mark.parametrize('group', GROUPS)
@pytest.mark.parametrize('raw', ([R, G], [G, Y], [R, Y, G], [R, G, L]))
def test_contradictory_lamps_abstain_without_new_red(policy, group, raw):
    assert policy.apply(raw, group, observed_only=True) == (
        [], policy.profile_name(group)+':observation_rejected')


def test_d_red_yellow_and_protected_left_remain_distinct_observations(policy):
    assert set(policy.apply([R, Y], 201326, observed_only=True)[0]) == {R, Y}
    assert set(policy.apply([R, L], 201326, observed_only=True)[0]) == {R, L}
    assert policy.apply([G, L], 201326, observed_only=True)[0] == []
    assert policy.apply([R, L], 201290, observed_only=True)[0] == []


@pytest.mark.parametrize('group', GROUPS)
def test_all_16_combinations_never_add_observed_lamps(policy, group):
    for mask in range(16):
        raw = [lamp for bit, lamp in enumerate((R,Y,G,L)) if mask & (1 << bit)]
        output, _ = policy.apply(raw, group, observed_only=True)
        assert set(output).issubset(raw)


@pytest.mark.parametrize('group', GROUPS)
def test_detection_only_gap_holds_exact_source_and_never_renews_deadline(policy, group):
    step(policy, [G], group=group)
    for now in (10.08, 10.15, 10.20):
        result, metadata = step(policy, [], now, int(now*1e9), group, eligible=True)
        assert result[0] == [G]
        assert metadata['adopted_observation_source_stamp_ns'] == 10_000_000_000
        assert metadata['output_origin'] == 'detection_miss_hold'
        assert metadata['output_is_held']
    result, metadata = step(policy, [], 10.201, 10_201_000_000, group, eligible=True)
    assert result[:2] == ([], 0.)
    assert not metadata['output_is_observed']


def test_hold_age_uses_original_image_not_recent_callback_clock(policy):
    step(policy, [G], now=10., source=9_850_000_000)
    assert step(policy, [], 10.04, 10_040_000_000, eligible=True)[0][0] == [G]
    assert step(policy, [], 10.051, 10_051_000_000, eligible=True)[0][:2] == ([], 0.)


@pytest.mark.parametrize('group', GROUPS)
@pytest.mark.parametrize('raw', ([R], [Y]))
def test_real_red_yellow_immediately_replaces_held_green(policy, group, raw):
    step(policy, [G], group=group)
    step(policy, [], 10.05, 10_050_000_000, group, eligible=True)
    result, metadata = step(policy, raw, 10.1, 10_100_000_000, group, eligible=True)
    assert result[0] == raw
    assert not metadata['output_is_held']
    assert metadata['adopted_observation_source_stamp_ns'] == 10_100_000_000


@pytest.mark.parametrize('invalid', ('map_missing','tf_stale','identity_uncertain','cnn_unknown','all_OFF'))
def test_non_detection_failures_clear_old_green_instead_of_resurrecting(policy, invalid):
    step(policy, [G])
    result, metadata = step(policy, [], 10.05, 10_050_000_000, eligible=False)
    assert result[:2] == ([], 0.)
    assert not metadata['output_is_observed']
    assert step(policy, [], 10.1, 10_100_000_000, eligible=True)[0][0] == []


def test_invalid_lamp_combination_cannot_use_detection_hold_even_if_caller_errors(policy):
    step(policy, [G])
    assert step(policy, [R,G], 10.05, 10_050_000_000, eligible=True)[0][0] == []


@pytest.mark.parametrize('head', (None, '', 'other_head'))
def test_missing_or_different_head_never_inherits_green(policy, head):
    step(policy, [G])
    assert step(policy, [], 10.05, 10_050_000_000, head=head, eligible=True)[0][0] == []


def test_changed_head_fresh_observation_replaces_cache(policy):
    step(policy, [G], head='old')
    step(policy, [R], 10.05, 10_050_000_000, head='new')
    assert step(policy, [], 10.1, 10_100_000_000, head='new', eligible=True)[0][0] == [R]


def test_route_change_or_return_cannot_reuse_old_group_observation(policy):
    step(policy, [G], group=201351)
    assert step(policy, [], 10.05, 10_050_000_000, group=201365, eligible=True)[0][0] == []
    assert step(policy, [], 10.1, 10_100_000_000, group=201351, eligible=True)[0][0] == []


@pytest.mark.parametrize('now,source', ((9.9,9_900_000_000),(10.1,9_900_000_000),(10.1,10_200_000_000),(10.1,None)))
def test_time_rollback_future_or_invalid_stamp_cannot_hold(policy, now, source):
    step(policy, [G])
    assert step(policy, [], now, source, eligible=True)[0][0] == []


def test_repeated_known_stamp_does_not_claim_new_source(policy):
    step(policy, [G])
    _, metadata = step(policy, [G], 10.05, 10_000_000_000)
    assert metadata['adopted_observation_source_stamp_ns'] == 10_000_000_000
    assert step(policy, [], 10.201, 10_201_000_000, eligible=True)[0][0] == []


def test_reset_forgets_direct_identity_bound_cache(policy):
    step(policy, [G])
    policy.reset_transition()
    assert step(policy, [], 10.05, 10_050_000_000, eligible=True)[0][0] == []


@pytest.mark.parametrize('hold', (-.1,.201,float('inf'),float('nan'),True,'0.2'))
def test_hold_duration_cannot_exceed_200ms_or_be_non_numeric(policy, hold):
    with pytest.raises(ValueError):
        step(policy, [G], hold=hold)


def test_observed_only_flag_is_not_truthy_string(policy):
    with pytest.raises(ValueError):
        policy.apply([G], 201351, observed_only='false')
