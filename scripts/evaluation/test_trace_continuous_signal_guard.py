"""Direction/lamp adapter only. Native scoring and vehicle behavior are not tested."""
from trace_continuous_signal_guard import GROUPS,phase


def test_ab_c_straight_and_d_left_contract_are_not_interchanged():
    red=['red','circle',None];green=['green','circle',None];left=['green','arrow',True]
    assert phase(201351,[green])=='green'
    assert GROUPS['B']==201365
    assert phase(201365,[green])=='green'
    assert phase(201365,[left])=='red'
    assert phase(201290,[green,left])=='green'
    assert phase(201326,[green])=='red'
    assert phase(201326,[red,left])=='green'


def test_empty_and_yellow_are_not_new_authorized_green():
    assert phase(201351,[])=='unknown'
    assert phase(201326,[['yellow','circle',None],['red','circle',None]])=='yellow'
