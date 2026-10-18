import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import overlay_read_cache as cached


def fixture(tmp_path):
    path=tmp_path/'overlay.json'
    value={'schema':'fixture','assignments':{'a':{'split':'train','group_id':'g'}},
        'policy':{'floors':[1,2,{'floor':3}]}}
    path.write_text(json.dumps(value))
    calls=[]
    def original(p):
        calls.append(str(p))
        result=json.loads(Path(p).read_text())
        if result.get('schema') != 'fixture':
            raise ValueError('Original schema rejected')
        return result
    return path,value,calls,original


def test_reuses_original_validation_but_reads_actual_bytes_on_every_hit(tmp_path):
    path,value,calls,original=fixture(tmp_path)
    with cached.installed(SimpleNamespace(read_overlay=original)) as c:
        first=c.read(path);second=c.read(path)
        assert first is second and first == value and len(calls) == 1
        assert c.stats['actual_hash_reads'] == 3  # miss before/after, then real hit SHA
        assert c.stats['hits'] == 1 and c.stats['misses'] == 1


def test_dict_list_isinstance_json_identity_and_mutable_deepcopy(tmp_path):
    path,value,_,original=fixture(tmp_path)
    with cached.installed(SimpleNamespace(read_overlay=original)) as c:
        a=c.read(path)
        assert isinstance(a,dict) and isinstance(a['policy']['floors'],list)
        assert json.loads(json.dumps(a,sort_keys=True)) == value
        # Existing explicit overlay code returns its input object unchanged.
        assert (a if isinstance(a,dict) else None) is a
        b=copy.deepcopy(a)
        assert type(b) is dict and type(b['policy']['floors']) is list
        assert type(b['assignments']['a']) is dict
        b['policy']['floors'][2]['floor']=99
        b['assignments']['a']['split']='val'
        assert a == value and copy.copy(a) is a


@pytest.mark.parametrize('operation',[
    lambda a:a.__setitem__('new',1),lambda a:a.__delitem__('schema'),
    lambda a:a.update({'new':1}),lambda a:a.pop('schema'),lambda a:a.popitem(),
    lambda a:a.clear(),lambda a:a.setdefault('new',1),lambda a:a.__ior__({'new':1}),
    lambda a:a['assignments']['a'].__setitem__('split','val'),
    lambda a:a['policy']['floors'].__setitem__(slice(None),[]),
    lambda a:a['policy']['floors'].__delitem__(0),lambda a:a['policy']['floors'].append(4),
    lambda a:a['policy']['floors'].extend([4]),lambda a:a['policy']['floors'].insert(0,4),
    lambda a:a['policy']['floors'].pop(),lambda a:a['policy']['floors'].clear(),
    lambda a:a['policy']['floors'].remove(1),lambda a:a['policy']['floors'].reverse(),
    lambda a:a['policy']['floors'].sort(),lambda a:a['policy']['floors'].__iadd__([4]),
    lambda a:a['policy']['floors'].__imul__(2),
])
def test_normal_recursive_mutation_routes_are_blocked(tmp_path,operation):
    path,value,_,original=fixture(tmp_path)
    with cached.installed(SimpleNamespace(read_overlay=original)) as c:
        a=c.read(path)
        with pytest.raises(TypeError,match='immutable'):
            operation(a)
        assert a == value


def test_reinitialization_cannot_mutate_cache_value(tmp_path):
    path,value,_,original=fixture(tmp_path)
    with cached.installed(SimpleNamespace(read_overlay=original)) as c:
        a=c.read(path)
        a.__init__({'evil':True});a['policy']['floors'].__init__([99])
        assert a == value


def test_base_builtin_mutation_bypass_is_detected_at_scope_exit(tmp_path):
    path,_,_,original=fixture(tmp_path);module=SimpleNamespace(read_overlay=original)
    with pytest.raises(cached.OverlayCacheContractError,match='memory changed'):
        with cached.installed(module) as c:
            a=c.read(path)
            dict.__setitem__(a['assignments']['a'],'split','val')
    assert module.read_overlay is original


def test_actual_byte_change_with_same_size_and_restored_mtime_is_a_miss(tmp_path):
    import os
    path,value,calls,original=fixture(tmp_path)
    with cached.installed(SimpleNamespace(read_overlay=original)) as c:
        a=c.read(path);before=path.stat()
        content=path.read_text();path.write_text(content.replace('train','other'))
        os.utime(path,ns=(before.st_atime_ns,before.st_mtime_ns))
        b=c.read(path)
        assert b['assignments']['a']['split'] == 'other' and a is not b
        assert len(calls) == 2 and c.stats['path_revisions'] == 1


def test_invalid_changed_file_is_still_rejected_by_original(tmp_path):
    path,_,calls,original=fixture(tmp_path);module=SimpleNamespace(read_overlay=original)
    with pytest.raises(ValueError,match='Original schema'):
        with cached.installed(module) as c:
            c.read(path)
            path.write_text('{"schema":"invalid"}')
            try:
                c.read(path)
            except ValueError:
                # Restore only the test fixture so scope exit checks don't mask
                # the original rejection this test is intended to observe.
                path.write_text(json.dumps({'schema':'fixture','assignments':{'a':{'split':'train','group_id':'g'}},
                    'policy':{'floors':[1,2,{'floor':3}]}}))
                raise
    assert len(calls) == 2 and module.read_overlay is original


def test_change_during_original_validation_is_not_cached(tmp_path):
    path,_,_,original=fixture(tmp_path)
    def changed(p):
        result=original(p);Path(p).write_text('{"schema":"other"}');return result
    c=cached.ReadOverlayCache(changed)
    with pytest.raises(cached.OverlayCacheContractError,match='during original'):
        c.read(path)
    assert not c.cache


def test_original_exception_restores_function_ownership(tmp_path):
    path,_,_,original=fixture(tmp_path);module=SimpleNamespace(read_overlay=original)
    with pytest.raises(RuntimeError,match='body error'):
        with cached.installed(module):
            raise RuntimeError('body error')
    assert module.read_overlay is original


def test_ownership_violation_restores_original_and_fails(tmp_path):
    _,_,_,original=fixture(tmp_path);module=SimpleNamespace(read_overlay=original)
    with pytest.raises(cached.OverlayCacheContractError,match='ownership'):
        with cached.installed(module):
            module.read_overlay=lambda path:{}
    assert module.read_overlay is original


def test_nested_scope_is_rejected_without_disabling_outer_scope(tmp_path):
    path,_,_,original=fixture(tmp_path);module=SimpleNamespace(read_overlay=original)
    with cached.installed(module) as outer:
        with pytest.raises(cached.OverlayCacheContractError,match='already installed'):
            with cached.installed(module):
                pass
        assert module.read_overlay(path) is outer.read(path)
    assert module.read_overlay is original


def test_lru_is_bounded_and_evicted_returned_values_stay_immutable(tmp_path):
    _,_,calls,original=fixture(tmp_path)
    paths=[tmp_path/f'{index}.json' for index in range(3)]
    for index,path in enumerate(paths):path.write_text(json.dumps({'schema':'fixture','n':index}))
    with cached.installed(SimpleNamespace(read_overlay=original),max_entries=2) as c:
        first=c.read(paths[0]);c.read(paths[1]);c.read(paths[2])
        assert len(c.cache) == 2 and c.stats['evictions'] == 1
        with pytest.raises(TypeError):first['n']=8
        assert c.read(paths[0]) == first and len(calls) == 4


def test_lru_checks_memory_before_eviction(tmp_path):
    path,_,_,original=fixture(tmp_path);other=tmp_path/'other.json'
    other.write_text('{"schema":"fixture"}')
    c=cached.ReadOverlayCache(original,max_entries=1);a=c.read(path)
    dict.__setitem__(a,'evil',True)
    with pytest.raises(cached.OverlayCacheContractError,match='memory changed'):
        c.read(other)


def test_excluded_new_output_always_uses_original_reader(tmp_path):
    path,_,calls,original=fixture(tmp_path)
    with cached.installed(SimpleNamespace(read_overlay=original),exclude_roots=(tmp_path,)) as c:
        a=c.read(path);b=c.read(path)
        assert a == b and a is not b and type(a) is dict
        assert len(calls) == 2 and not c.cache and c.stats['passthrough'] == 2


def test_retained_file_mutation_at_exit_is_detected_and_reader_restored(tmp_path):
    path,_,_,original=fixture(tmp_path);module=SimpleNamespace(read_overlay=original)
    with pytest.raises(cached.OverlayCacheContractError,match='before scope exit'):
        with cached.installed(module) as c:
            c.read(path);path.write_text('{"schema":"fixture","changed":true}')
    assert module.read_overlay is original


def test_symlink_input_is_rejected(tmp_path):
    path,_,_,original=fixture(tmp_path);link=tmp_path/'symlink.json';link.symlink_to(path)
    with pytest.raises(cached.OverlayCacheContractError,match='symlink'):
        cached.ReadOverlayCache(original).read(link)


@pytest.mark.parametrize('maximum',[0,-1,129,True,1.5])
def test_invalid_cache_size_rejected(maximum):
    with pytest.raises(cached.OverlayCacheContractError,match='bounded'):
        cached.ReadOverlayCache(lambda path:{},max_entries=maximum)
