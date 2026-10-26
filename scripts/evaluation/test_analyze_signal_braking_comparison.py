from analyze_signal_braking_comparison import median, nearest


def test_nearest_refuses_stale_pose_and_handles_empty_data():
    rows=[dict(t=100, speed=0),dict(t=200,speed=1)]
    assert nearest(rows,[100,200],180,tolerance=30)==rows[1]
    assert nearest(rows,[100,200],150,tolerance=30) is None
    assert nearest([],[],150) is None


def test_missing_baseline_is_not_a_zero_pixel_drop():
    assert median([]) is None
    assert median([3,1,2])==2
