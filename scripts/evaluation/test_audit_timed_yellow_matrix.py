import json

import pytest

from audit_timed_yellow_matrix import audit, completed, finite_number


def test_nonfinite_log_values_do_not_become_nonstandard_JSON():
    assert finite_number('nan') is None
    assert finite_number('inf') is None
    assert finite_number('1.85') == 1.85


def test_open_wrapper_refused_and_failed_native_trial_not_marked_qualified(tmp_path):
    assert not completed(tmp_path)
    with pytest.raises(ValueError, match='closed'):
        audit(tmp_path)
    (tmp_path/'wrapper_events.jsonl').write_text(json.dumps(dict(event='validator_returned', returncode=1))+'\n')
    (tmp_path/'selection.json').write_text(json.dumps(dict(real_native_moving_yellow=True,
                          synthetic_canonical_signal_proxy=False, enable_yellow_passthrough=True)))
    phase = tmp_path/'phase_controller';phase.mkdir()
    (phase/'metadata.json').write_text(json.dumps(dict(group=dict(group_id=201290, native_ids=['C016', 'C018']))))
    (phase/'events.jsonl').write_text(json.dumps(dict(event='experiment_error', error='native mismatch'))+'\n')
    report = audit(tmp_path)
    assert report['wrapper_returncode'] == 1
    assert not report['moving_gate_qualified']
    assert not report['full_yellow_and_red_stage_complete']
    assert not report['official_penalty_certified']
