"""새 manifest route 계약과 1nm 이내 arc-only 복원을 검사한다."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import fresh_route_compat as route


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value) if not isinstance(value,str) else value)
    return path


@pytest.fixture
def fixture(tmp_path):
    points=[dict(x=0.,y=0.,z=0.,link_idx='a'),dict(x=1.,y=0.,z=0.,link_idx='a'),
            dict(x=2.,y=0.,z=0.,link_idx='b')]
    arc=[0.,1.,2.]
    extent=SimpleNamespace(__file__=str(write(tmp_path/'extent.py','# frozen extent')),
        route_sha256=lambda ctx:route.digest([route.coordinates(ctx['route']),list(ctx['arc'])]))
    geo=SimpleNamespace(__file__=str(write(tmp_path/'geo.py','# frozen geo')),
        DEFAULT_MGEO=tmp_path/'map',DEFAULT_ROUTE=write(tmp_path/'guide.json',points),
        load_route=lambda p:(copy.deepcopy(points),{}),load_json=lambda p:{'map':True},
        build_mgeo_guided_competition_route=lambda *args:copy.deepcopy(points),
        route_arc_coordinates=lambda p:list(arc))
    write(geo.DEFAULT_MGEO/'link_set.json',{'actual':'map'})
    gen=SimpleNamespace(__file__=str(write(tmp_path/'gen.py','# frozen gen')),geo=geo,
        context=lambda runtime,mgeo:dict(route=copy.deepcopy(points),arc=list(arc)))
    manifest=write(tmp_path/'manifest.json',dict(count=5000,scenes=[{'id':str(i)} for i in range(5000)],
        config=dict(environment_tag='sunny11',static_mission_extent=dict(route_sha256=extent.route_sha256(dict(route=points,arc=arc))))))
    witness=tmp_path/'witness.json'
    return SimpleNamespace(points=points,arc=arc,extent=extent,gen=gen,manifest=manifest,witness=witness)


def export(f):
    route.export_witness(f.manifest,f.witness,generator=f.gen,extent=f.extent)
    return dict(generator=f.gen,extent=f.extent,manifest=f.manifest,expected_witness_sha256=route.sha(f.witness))


def test_export_binds_actual_new_environment_not_legacy_sunny13(fixture):
    f=fixture;opts=export(f);doc=route.verified_witness(f.witness,**opts)
    assert doc['environment']=='sunny11' and doc['manifest_sha256']==route.sha(f.manifest)
    assert doc['bulk_authorized'] is False and doc['native_capture_executed'] is False
    assert doc['point_count']==3 and doc['route_points']==f.points


def test_actual_producer_must_reproduce_manifest_route_sha(fixture):
    f=fixture;f.gen.geo.route_arc_coordinates=lambda _: [0.,1.1,2.]
    with pytest.raises(RuntimeError,match='producer route'):
        route.export_witness(f.manifest,f.witness,generator=f.gen,extent=f.extent)
    assert not f.witness.exists()


def test_restores_only_small_arc_and_keeps_other_context_unchanged(fixture):
    f=fixture;opts=export(f);ctx=dict(route=copy.deepcopy(f.points),arc=[0.,1.+1e-12,2.+1e-12],extra={'keep':True})
    original_points=ctx['route'];extra=ctx['extra']
    result=route.restore_context(ctx,f.witness,**opts)
    assert result is ctx and ctx['route'] is original_points and ctx['extra'] is extra and ctx['arc']==f.arc


@pytest.mark.parametrize('change',['xyz','link','order','arc','source','manifest','helperpin'])
def test_uncertified_changes_are_rejected_without_context_mutation(fixture,change):
    f=fixture;opts=export(f);ctx=dict(route=copy.deepcopy(f.points),arc=list(f.arc))
    if change=='xyz':ctx['route'][1]['x']+=.01
    elif change=='link':ctx['route'][1]['link_idx']='different'
    elif change=='order':ctx['route'].reverse()
    elif change=='arc':ctx['arc'][1]+=.001
    elif change=='source':Path(f.gen.__file__).write_text('# edited generator')
    elif change=='manifest':f.manifest.write_text(f.manifest.read_text()+' ')
    else:opts['expected_witness_sha256']='f'*64
    before=copy.deepcopy(ctx)
    with pytest.raises(RuntimeError):route.restore_context(ctx,f.witness,**opts)
    assert ctx==before


def test_scoped_adapter_checks_every_actual_context_and_restores_identity(fixture):
    f=fixture;opts=export(f)
    f.gen.context=lambda runtime,mgeo:dict(route=copy.deepcopy(f.points),arc=[0.,1.+1e-12,2.])
    original=f.gen.context
    with route.installed(f.witness,**opts):
        assert f.gen.context(None,f.gen.geo.DEFAULT_MGEO)['arc']==f.arc
    assert f.gen.context is original


def test_adapter_ownership_change_is_not_silent(fixture):
    f=fixture;opts=export(f);original=f.gen.context
    with pytest.raises(RuntimeError,match='ownership'):
        with route.installed(f.witness,**opts):f.gen.context=lambda *args:None
    assert f.gen.context is original


def test_existing_witness_and_symlink_are_never_overwritten(fixture,tmp_path):
    f=fixture;export(f);before=f.witness.read_bytes()
    with pytest.raises(RuntimeError,match='Fresh witness'):
        route.export_witness(f.manifest,f.witness,generator=f.gen,extent=f.extent)
    assert before==f.witness.read_bytes()
    link=tmp_path/'link.json';link.symlink_to(f.witness)
    with pytest.raises(RuntimeError,match='nonsymlink'):
        route.verified_witness(link,generator=f.gen,extent=f.extent,manifest=f.manifest,expected_witness_sha256=route.sha(f.witness))
