"""Pure CPU checks for the isolated vehicle ROS qualification fixture."""
import math
import copy
import socket
import types
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from build_vehicle_ros_candidates import replace_once
from replay_vehicle_ros_candidates import (
    direction_summary, paired_summary, recorded_sonata_handoff,
    validate_output_contract, validate_private_port, compare_wire_bags)


def frame(stamp=100., identity='car', center=(1., 2., 0.), velocity=(1., 0.)):
    return dict(stamp=stamp, objects=[dict(id=identity, center=list(center),
        world_velocity=list(velocity), yaw=0.)])


def test_source_patch_requires_unique_anchor():
    assert replace_once('before ANCHOR after', 'ANCHOR', 'patch') == 'before patch after'
    for source in ('missing', 'ANCHOR ANCHOR'):
        try:
            replace_once(source, 'ANCHOR', 'patch')
        except ValueError:
            pass
        else:
            raise AssertionError('Source mutation requires one unique anchor')


def test_paired_output_only_velocity_change_preserves_native_contract():
    result = paired_summary([frame()], [frame(velocity=(-1., 0.))])
    assert result['unpaired_frames'] == result['identity_set_changed_frames'] == 0
    assert result['stats']['center_delta']['maximum'] == 0.
    assert result['stats']['world_velocity_delta']['maximum'] == 2.


def test_paired_detects_native_state_and_identity_regression():
    moved = paired_summary([frame()], [frame(center=(3., 2., 0.))])
    assert moved['stats']['center_delta']['maximum'] == 2.
    changed = paired_summary([frame()], [frame(identity='new')])
    assert changed['identity_set_changed_frames'] == 1


def test_paired_rejects_distinct_scheduler_cohort():
    assert paired_summary([frame()], [frame(stamp=100.02)])['unpaired_frames'] == 1


def test_direction_diagnostic_is_fixed_actor_and_window_only():
    frames = [frame(100., velocity=(0., .6)), frame(100.1, velocity=(0., -.6)),
              frame(100.2, identity='other', velocity=(0., .6)), frame(102., velocity=(0., .6))]
    result = direction_summary(frames, 'car', 100., 101.)
    assert result['object_frames'] == 2 and result['opposite_world_y_frames'] == 1


def test_recorded_handoff_exposes_non_overlapping_state():
    old = '414a4f5500000000000000000000002c'
    new = '414a4f55000000000000000000000036'
    base = 1791908591.4141977
    first = frame(base + 134.9, old)
    first['objects'] += frame(base + 134.9, new, center=(1., 8., 0.))['objects']
    result = recorded_sonata_handoff([first, frame(base+135., old)])
    assert result['overlap_frames'] == 1 and result['new_object_frames'] == 1
    assert math.isclose(result['center_separation_m']['maximum'], 6.)
    recovered = recorded_sonata_handoff([frame(base+134.9, old)])
    assert recovered['overlap_frames'] == recovered['new_object_frames'] == 0


def test_private_master_rejects_standard_user_and_invalid_ports():
    for port in (11311, 19999, 65536, 0, True):
        try: validate_private_port(port)
        except ValueError: pass
        else: raise AssertionError('User/default/nonprivate master port must be rejected')


def test_private_master_rejects_occupied_port():
    with socket.socket() as held:
        held.bind(('127.0.0.1', 0))
        port = held.getsockname()[1]
        assert port >= 20000
        try: validate_private_port(port)
        except OSError: pass
        else: raise AssertionError('An occupied master endpoint must never be reused')


def test_complete_boundary_fixture_passes_golden_contract():
    pair = paired_summary([frame()], [frame(velocity=(-1., 0.))])
    report = dict(sent_frames=1, valid_source_frames=1,
        native_arrival_counts={'baseline': 1, 'guard': 1}, outputs={'baseline': 1, 'guard': 1},
        paired={'baseline_guard': pair})
    validate_output_contract(report)


def test_golden_contract_rejects_missing_arrival_and_any_scheduler_or_state_drift():
    pair = paired_summary([frame()], [frame()])
    golden = dict(sent_frames=1, valid_source_frames=1,
        native_arrival_counts={'baseline': 1, 'guard': 1}, outputs={'baseline': 1, 'guard': 1},
        paired={'baseline_guard': pair})
    variants = []
    missing = copy.deepcopy(golden); missing['native_arrival_counts']['guard'] = 0; variants.append(missing)
    for pair in (paired_summary([frame()], [frame(stamp=100.005)]),
                 paired_summary([frame()], [frame(stamp=100.02)]),
                 paired_summary([frame()], [frame(identity='new')]),
                 paired_summary([frame()], [frame(center=(2., 2., 0.))])):
        bad = copy.deepcopy(golden); bad['paired']['baseline_guard'] = pair; variants.append(bad)
    for bad in variants:
        try: validate_output_contract(bad)
        except RuntimeError: pass
        else: raise AssertionError('An incomplete or non-equivalent fixture cannot qualify the policy')


def test_nominal_wire_gate_catches_fields_not_kept_in_object_row(monkeypatch):
    streams = {'before': [b'pose_same_covariance_old'],
               'equal': [b'pose_same_covariance_old'],
               'changed': [b'pose_same_covariance_new'], 'missing': []}
    class Bag:
        def __init__(self, path): self.path = path
        def __enter__(self): return self
        def __exit__(self, *args): return False
        def read_messages(self, topics, raw):
            assert raw
            for blob in streams[self.path]: yield topics[0], ('type', blob, 'md5', (0, 0), object), 0.
    monkeypatch.setitem(sys.modules, 'rosbag', types.SimpleNamespace(Bag=Bag))
    assert compare_wire_bags('before', 'equal', ['/tracking'])['/tracking']['exact']
    changed = compare_wire_bags('before', 'changed', ['/tracking'])['/tracking']
    assert not changed['exact'] and changed['different_serialized_messages'] == 1
    assert not compare_wire_bags('before', 'missing', ['/tracking'])['/tracking']['exact']
