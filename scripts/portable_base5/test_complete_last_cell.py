from copy import deepcopy

import pytest
import complete_last_cell as last


def scene(family='shadow', split='train', distance=15., motion='static', asset='2017_Kia_Niro(HEV)'):
    return dict(spec=dict(family=family,mission=False,shadow_phase='interior',role='front'),
        split=split,planned_primary_distance_m=distance,
        runtime_actors=[dict(name=asset,motion=motion,data_id=1)])


def test_last_cell_explicitly_relaxes_pool_and_range_but_keeps_static_shadow_split():
    source=scene(distance=45.,asset='Dog1');candidate=scene()
    assert last.same_family_readable(source,candidate,{})
    for field,value in [('family','highway'),('split','val'),('distance',31.),('motion','running')]:
        kwargs={field:value}
        assert not last.same_family_readable(source,scene(**kwargs),{})
    scaled=deepcopy(candidate);scaled['runtime_actors'][0]['asset_scale']={'profile_id':'scaled'}
    assert not last.same_family_readable(source,scaled,{})


def test_only_approved_single_interior_front_templates_are_eligible():
    source=scene(distance=45.,asset='Dog1')
    preferred=scene();approach=scene();approach['spec']['shadow_phase']='approach'
    rear=scene();rear['spec']['role']='rear'
    unknown=scene(asset='thin-pole')
    donors=[({'id':str(i),'scenario_sha256':str(i)},m)
        for i,m in enumerate((preferred,approach,rear,unknown))]
    assert [e['id'] for e,m in last.easy_donors(source,donors,{},'slot',4,set())]==['0']
    assert last.easy_donors(source,donors,{},'slot',4,{'0'})==[]


def test_last_cell_does_not_reset_attempt_budget_or_accept_implicit_nested_imports():
    from types import SimpleNamespace
    for args in (SimpleNamespace(import_native_completion=[],attempts=3),
                 SimpleNamespace(import_native_completion=['unsafe'],attempts=5)):
        with pytest.raises(RuntimeError,match='historical attempts'):
            last.execute(args)
