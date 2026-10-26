import sys

import pytest

from analyze_signal_map_stop_trial import main, quantiles, detector_clearance


def test_detector_clearance_keeps_clipping_and_rejects_bad_boxes():
    assert detector_clearance([100, 80, 300, 120]) == dict(left=100, top=80, right=980, bottom=600)
    assert detector_clearance([100, -2, 300, 120])['top'] == -2
    for box in (None, [1, 2], [1, 2, 0, 3], [0, float('nan'), 2, 3]):
        assert detector_clearance(box) is None


def test_empty_and_ordered_gap_summary():
    assert quantiles([]) is None
    assert quantiles([.2, -.1, 0.]) == dict(min=-.1, p50=0., max=.2)


def arguments(monkeypatch, tmp_path, output, bag):
    monkeypatch.setattr(sys, 'argv', ['analyze', '--bag', str(bag), '--observer', str(tmp_path),
        '--geometry', str(tmp_path/'geometry.json'), '--group-id', '201326', '--output', str(output)])


def test_active_recording_rejected_before_ros_import(monkeypatch, tmp_path):
    arguments(monkeypatch, tmp_path, tmp_path/'new', tmp_path/'recording.bag.active')
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
    assert not (tmp_path/'new').exists()


def test_existing_evidence_never_overwritten(monkeypatch, tmp_path):
    arguments(monkeypatch, tmp_path, tmp_path, tmp_path/'closed.bag')
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
