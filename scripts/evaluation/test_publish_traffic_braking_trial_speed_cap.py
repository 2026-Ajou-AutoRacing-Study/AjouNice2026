import pytest
from publish_traffic_braking_trial_speed_cap import validate_cap


def test_speed_cap_accepts_only_explicit_finite_bounded_speeds():
    assert validate_cap(10.) == 10.
    for value in (True, None, 0., 21., float('nan'), float('inf')):
        with pytest.raises(ValueError):
            validate_cap(value)
