from audit_timed_signal_natural_trial import distribution, signal_class


def lamp(color,shape=1,status=2):
    return dict(color=color,shape=shape,status=status)


def test_directional_authorization_and_unknown_status():
    assert signal_class(201351,[lamp(3)])=='authorized'
    assert signal_class(201326,[lamp(3)])=='stop'
    assert signal_class(201326,[lamp(1),lamp(3,2)])=='authorized'
    assert signal_class(201326,[lamp(1),lamp(2)])=='yellow'
    assert signal_class(201351,[lamp(2),lamp(3)])=='unknown'
    assert signal_class(201351,[lamp(3,status=1)])=='unknown'
    assert signal_class(201351,[])=='unknown'


def test_finite_percentiles_not_infinite_failure_counts():
    assert distribution([])=={'count':0}
    d=distribution([None,float('nan'),float('inf'),1.,3.])
    assert d['count']==2 and d['p50']==2. and d['maximum']==3.
