"""Read-only adapter/real-source-Guard tests; not vehicle or native scoring tests."""
import csv
from pathlib import Path
import subprocess

import pytest

from replay_traffic_light_guard import (
    CPP, HEADER, classify, cpp_replay, partial_evidence, schedules, source_ns)


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    target = tmp_path_factory.mktemp('pure-guard') / 'guard'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
        '-I' + str(HEADER.parents[2]), str(CPP), '-o', str(target)], check=True)
    return target


def execute(binary, observations, grace=.2, required=1):
    lines = []
    for index, (phase, source, now, color) in enumerate(observations):
        lines.append(f'{index} fixture {required} {grace} {phase} {source} {now} 0 {int(color)} 0 1')
    result = subprocess.run([str(binary)], input='\n'.join(lines) + '\n',
                            capture_output=True, text=True, check=True)
    return list(csv.DictReader(result.stdout.splitlines()))


def element(color, shape=1, status=2):
    return dict(color=color, shape=shape, status=status)


def test_source_ns_preserves_integer_epoch():
    assert source_ns(dict(sec=1793088594, nanosec=757705633)) == 1793088594757705633
    assert source_ns(dict(sec=1, nanosec=10**9)) is None


def test_direction_and_AW_enum_contract_is_not_aliased():
    assert classify([element(3)], 201351)['phase'] == 'green'
    assert classify([element(3)], 201326)['phase'] == 'stop'
    assert classify([element(1), element(3, 2)], 201326)['phase'] == 'green'
    assert classify([element(3, 2)], 201365)['phase'] == 'stop'
    assert not classify([], 201397)['available']


def test_known_color_unknown_shape_and_yellow_green_remain_color_evidence():
    for elements in ([element(3, 0)], [element(1, status=0)], [element(2), element(3)]):
        result = classify(elements, 201351)
        assert result['phase'] == 'unknown' and result['has_color_evidence']
    assert classify([], 201351)['has_color_evidence'] is False
    assert classify([element(2)], 201326)['phase'] == 'yellow'


def evidence(bits, reason='low_confidence_bits:left_arrow'):
    decision = dict(group_id=201290, head_id='C1256W000016', planner_compliance_observations=True,
                    observed_tuples=[], normalized_tuples=[])
    runtime = dict(source_stamp_ns=123, frame_decision=decision)
    classifier = dict(source_stamp_ns=123, group_id=201290, head_id='C1256W000016',
                      bits=bits, reason=reason)
    return partial_evidence(runtime, classifier)


def test_partial_marker_only_preserves_recorded_positive_bits():
    result = evidence(dict(red=0, yellow=0, green=1, left_arrow=None))
    assert result['available'] and result['has_color_evidence']
    assert result['reconstructed_unknown_color_markers'] == [dict(color=3, shape=0, status=0)]
    assert not evidence(dict(red=0, yellow=0, green=0, left_arrow=0), 'no_lamps')['has_color_evidence']
    assert not partial_evidence(None, None)['available']
    assert not evidence(None, 'unknown_inference_failure')['available']


def test_repeated_unknown_ticks_expire_original_G_source_not_unknown_receipt(binary):
    rows = execute(binary, [('green', 10_000_000_000, 10_040_000_000, True),
        ('unknown', 10_100_000_000, 10_140_000_000, False),
        ('unknown', 10_100_000_000, 10_190_000_000, False),
        ('unknown', 10_100_000_000, 10_210_000_000, False)])
    assert [row['require_stop'] for row in rows] == ['0', '0', '0', '1']
    assert rows[1]['green_count'] == rows[2]['green_count'] == '1'


@pytest.mark.parametrize('phase', ['stop', 'yellow'])
def test_explicit_R_Y_never_delayed_by_grace(binary, phase):
    for grace in (0., .2, .3):
        rows = execute(binary, [('green', 10_000_000_000, 10_020_000_000, True),
            (phase, 10_100_000_000, 10_120_000_000, True)], grace)
        assert rows[-1]['require_stop'] == '1' and rows[-1]['grace_active'] == '0'


def test_unknown_with_partial_color_is_immediate_STOP(binary):
    rows = execute(binary, [('green', 10_000_000_000, 10_020_000_000, True),
        ('unknown', 10_100_000_000, 10_120_000_000, True)])
    assert rows[-1]['require_stop'] == '1'


def test_green2_cannot_hold_one_unconfirmed_green_or_count_duplicate_ticks(binary):
    rows = execute(binary, [('green', 10_000_000_000, 10_020_000_000, True),
        ('green', 10_000_000_000, 10_060_000_000, True),
        ('unknown', 10_100_000_000, 10_120_000_000, False)], .3, 2)
    assert [row['require_stop'] for row in rows] == ['1', '1', '1']
    assert rows[1]['green_count'] == '1'


def test_older_green_after_unknown_does_not_refresh_grant(binary):
    rows = execute(binary, [('green', 10_000_000_000, 10_020_000_000, True),
        ('unknown', 10_100_000_000, 10_120_000_000, False),
        ('green', 10_000_000_000, 10_140_000_000, True)])
    assert rows[-1]['require_stop'] == '1'


def test_synthetic_grid_uses_latest_received_wire_not_future_input():
    records = [dict(receipt_ros_ns=1_000_000_000), dict(receipt_ros_ns=1_120_000_000)]
    schedule = dict(schedules(records, [0]))['20Hz_phase_0ms']
    assert schedule == [(1_000_000_000, 0), (1_050_000_000, 0), (1_100_000_000, 0)]


def test_actual_D_fixture_20Hz_exposes_residual_02_lease_gap(binary):
    # Exact canonical source/observer receipts from closed current D. Not a car replay.
    values = [(1793088594438187360, 1793088594398011158, [element(2)]),
        (1793088594567252159, 1793088594523229070, [element(2)]),
        (1793088594669728755, 1793088594632342420, [element(1), element(3, 2)]),
        (1793088594791689872, 1793088594757705633, []),
        (1793088594911825180, 1793088594877249082, [element(1), element(3, 2)]),
        (1793088595053000000, 1793088595004793282, [element(1), element(3, 2)])]
    records = []
    for i, (receipt, source, elements) in enumerate(values):
        classified = classify(elements, 201326)
        records.append(dict(record_id=i, receipt_ros_ns=receipt, source_stamp_ns=source,
            group_id=201326, segment=1, streams=dict(wire=classified,
                counterfactual_partial_marker=classified)))
    results = cpp_replay(binary, dict(records=records, required_green=1, timeout_sec=1.),
                         [0, 10, 20, 30, 40], (1,), (0., .2, .3))
    for phase in [0, 10, 20, 30, 40]:
        unknown = [r for r in results if r['scenario'] == f'20Hz_phase_{phase}ms'
                   and r['stream'] == 'wire' and r['phase'] == 'unknown']
        assert any(r['require_stop'] for r in unknown if r['grace_sec'] == .2)
        assert not any(r['require_stop'] for r in unknown if r['grace_sec'] == .3)
