from types import SimpleNamespace as NS

from analyze_signal_latency_retest import phase_window, stamp_ns, stats


def test_statistics_do_not_convert_missing_or_negative_to_zero():
    assert stats([None,float('nan')]) == dict(count=0)
    result=stats([-2,0,2])
    assert result['p50'] == 0 and result['min'] == -2 and result['mean'] == 0


def test_both_ros_time_shapes_and_headerless_commands():
    assert stamp_ns(NS(stamp=NS(sec=2,nanosec=3))) == 2000000003
    assert stamp_ns(NS(header=NS(stamp=NS(secs=4,nsecs=5)))) == 4000000005
    assert stamp_ns(NS()) is None
    assert stamp_ns(NS(stamp=NS(sec=0,nanosec=0))) is None


def test_controller_phase_excludes_prepare_and_cleanup():
    events=[dict(event='CASE_RESULT',time='2026-10-26T00:00:00+00:00'),
        dict(event='CONTROLLER_START',time='2026-10-26T00:00:10+00:00'),
        dict(event='RUN_FINISHED',time='2026-10-26T00:00:20+00:00')]
    start,end=phase_window(events)
    assert (end-start)/1e9 == 10
