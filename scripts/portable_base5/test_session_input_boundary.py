"""Offline fixtures only; no ROS, MORAI, or collection output is modified."""
import hashlib
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

import session_input_boundary as module
from session_input_boundary import BoundaryInputError, BoundaryOwnershipError, SessionInputBoundary


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fixture(tmp_path):
    root = tmp_path/'source';root.mkdir()
    path = root/'manifest.json';path.write_text('{"count": 5000}')
    code = root/'generator.py';code.write_text('FROZEN = True\n')
    pins = {str(path):digest(path), str(code):digest(code)}
    boundary = SessionInputBoundary(pins, forbidden_roots=(tmp_path/'live',))
    return path, code, boundary


def test_explicit_verify_required_and_real_hashes_at_both_boundaries(tmp_path):
    path, _, boundary = fixture(tmp_path)
    calls = []
    assert boundary.sha256(path, fallback=lambda value:calls.append(value) or digest(value)) == digest(path)
    assert calls == [path]
    boundary.verify()
    assert boundary.stats['boundary_hash_reads'] == 2
    expected = boundary.sha256(path, fallback=lambda _:pytest.fail('Unexpected hot hash'))
    path.write_text('different bytes')
    # The explicit requested contract assumes fixed bytes between boundaries.
    assert boundary.sha256(path, fallback=digest) == expected
    with pytest.raises(BoundaryInputError, match='changed at session boundary'):
        boundary.verify()
    assert boundary.sha256(path, fallback=digest) == digest(path)


def test_fixed_hot_lookup_performs_zero_stat_or_hash(tmp_path, monkeypatch):
    path, _, boundary = fixture(tmp_path);boundary.verify()
    expected = digest(path)
    monkeypatch.setattr(module, '_real_sha', lambda _:pytest.fail('Unexpected hot hash'))
    monkeypatch.setattr(Path, 'stat', lambda *a, **kw:pytest.fail('Unexpected hot stat'))
    monkeypatch.setattr(Path, 'lstat', lambda *a, **kw:pytest.fail('Unexpected hot lstat'))
    for _ in range(20):
        assert boundary.sha256(path, fallback=digest) == expected
    assert boundary.stats['fixed_hits'] == 20


@pytest.mark.parametrize('kind', ['bytes', 'deleted', 'symlink', 'ancestor_symlink', 'directory'])
def test_start_and_end_detect_unsafe_input_changes(tmp_path, kind):
    path, _, boundary = fixture(tmp_path);boundary.verify()
    if kind == 'bytes':path.write_text('new content')
    elif kind == 'deleted':path.unlink()
    elif kind == 'symlink':
        target = tmp_path/'other.json';target.write_bytes(path.read_bytes())
        path.unlink();path.symlink_to(target)
    elif kind == 'ancestor_symlink':
        moved = tmp_path/'moved';path.parent.rename(moved);path.parent.symlink_to(moved)
    else:path.unlink();path.mkdir()
    with pytest.raises((BoundaryInputError, FileNotFoundError)):
        boundary.verify()
    with pytest.raises(BoundaryInputError, match='Unverified'):
        boundary.snapshot()


@pytest.mark.parametrize('bad', ['relative/file.json', '/tmp/../x.json'])
def test_nonabsolute_or_traversing_pins_rejected(bad):
    with pytest.raises(BoundaryInputError):SessionInputBoundary({bad:'0'*64})


def test_output_can_never_be_declared_fixed(tmp_path):
    path = tmp_path/'live'/'scenario.json'
    with pytest.raises(BoundaryInputError, match='Live/derived output'):
        SessionInputBoundary({str(path):'0'*64}, forbidden_roots=(tmp_path/'live',))


@pytest.mark.parametrize('relative', ['captures/metadata.json', 'raw/scenario.json',
    'native_reference/annotation.json', 'LIDAR_5/data.json', 'CAMERA_1/instance.json',
    'reports/audit.json', 'status.json', 'shutdown_safety.json', 'session_contract.json',
    'selected_ledger.jsonl', 'reference_receipt.json', 'map_proof.json', 'image.png'])
def test_native_and_mutable_pins_always_passthrough(tmp_path, relative):
    path = tmp_path/relative;path.parent.mkdir(parents=True, exist_ok=True);path.write_text('original')
    boundary = SessionInputBoundary({str(path):digest(path)});boundary.verify()
    path.write_text('changed native evidence');calls = []
    assert boundary.sha256(path, fallback=lambda value:calls.append(value) or digest(value)) == digest(path)
    assert calls == [path] and boundary.stats['fixed_hits'] == 0


def test_unknown_and_unusual_input_keep_original_semantics(tmp_path):
    _, _, boundary = fixture(tmp_path);boundary.verify()
    value = object();calls = []
    assert boundary.sha256(value, fallback=lambda item:calls.append(item) or 'unchanged') == 'unchanged'
    assert calls == [value]
    with pytest.raises(FileNotFoundError):boundary.sha256(tmp_path/'missing.json', fallback=digest)


def test_exact_fixed_input_mapping_replay_and_return_copy(tmp_path, monkeypatch):
    path, code, boundary = fixture(tmp_path);boundary.verify()
    expected = {str(path):digest(path), str(code):digest(code)}
    boundary.remember_input_pins(path, expected)
    monkeypatch.setattr(module, '_real_sha', lambda _:pytest.fail('Unexpected fixed input traversal/hash'))
    got = boundary.input_pins(path, fallback=lambda _:pytest.fail('Unexpected full recursive traversal'))
    assert got == expected
    got[str(code)] = '0'*64
    assert boundary.input_pins(path, fallback=None) == expected
    assert boundary.stats['input_map_hits'] == 2


def test_generated_manifest_does_not_reuse_original_fixed_mapping(tmp_path):
    path, _, boundary = fixture(tmp_path);boundary.verify()
    boundary.remember_input_pins(path, {str(path):digest(path)})
    fresh = tmp_path/'live'/'manifest.json';calls = []
    assert boundary.input_pins(fresh, fallback=lambda value:calls.append(value) or {'generated':'actual'}) == {'generated':'actual'}
    assert calls == [fresh]


def test_mapping_replay_rechecks_native_proofs_every_time(tmp_path):
    path, _, boundary = fixture(tmp_path);boundary.verify()
    proof = tmp_path/'native'/'audit.json';proof.parent.mkdir();proof.write_text('native original')
    values = {str(path):digest(path), str(proof):digest(proof)}
    boundary.remember_input_pins(path, values)
    assert boundary.input_pins(path, fallback=None) == values
    proof.write_text('tampered native')
    with pytest.raises(BoundaryInputError, match='Nonfixed referenced evidence changed'):
        boundary.input_pins(path, fallback=None)


def test_register_mapping_requires_verified_manifest_and_exact_digest(tmp_path):
    path, _, boundary = fixture(tmp_path)
    with pytest.raises(BoundaryInputError):boundary.remember_input_pins(path, {str(path):digest(path)})
    boundary.verify()
    with pytest.raises(BoundaryInputError):boundary.remember_input_pins(path, {str(path):'f'*64})
    with pytest.raises(BoundaryInputError):boundary.remember_input_pins(tmp_path/'new.json', {str(path):digest(path)})


def test_runtime_discovery_cache_uses_exact_arguments(tmp_path):
    _, code, boundary = fixture(tmp_path);boundary.verify()
    expected = {str(code):digest(code)};calls = []
    def discover(*args, **kwargs):calls.append((args, kwargs));return expected
    for _ in range(3):
        assert boundary.collect_runtime(entrypoints=('a.py',), fallback=discover) == expected
    assert len(calls) == 1
    boundary.collect_runtime(entrypoints=('b.py',), fallback=discover)
    boundary.collect_runtime(entrypoints=['a.py'], fallback=discover)
    boundary.collect_runtime(None, ('a.py',), fallback=discover)
    assert len(calls) == 4
    assert boundary.stats['runtime_map_hits'] == 2


def test_runtime_mapping_with_unknown_files_is_never_memoized(tmp_path):
    _, _, boundary = fixture(tmp_path);boundary.verify()
    calls = []
    def discover():calls.append(True);return {str(tmp_path/'new.py'):'a'*64}
    boundary.collect_runtime(fallback=discover);boundary.collect_runtime(fallback=discover)
    assert len(calls) == 2


def test_snapshot_requires_external_authenticated_authority(tmp_path, monkeypatch):
    path, code, boundary = fixture(tmp_path);boundary.verify()
    values = {str(path):digest(path), str(code):digest(code)}
    boundary.remember_input_pins(path, values)
    boundary.collect_runtime(entrypoints=('generator.py',), fallback=lambda **kw:{str(code):digest(code)})
    data = json.loads(json.dumps(boundary.snapshot()))
    with pytest.raises(BoundaryInputError):SessionInputBoundary.from_verified_snapshot(data)
    monkeypatch.setattr(module, '_real_sha', lambda _:pytest.fail('Worker must not hash fixed tree again'))
    child = SessionInputBoundary.from_verified_snapshot(data, authority_verified=True)
    assert child.input_pins(path, fallback=None) == values
    assert child.collect_runtime(entrypoints=('generator.py',), fallback=lambda **kw:pytest.fail('Repeated runtime AST discovery')) == {str(code):values[str(code)]}


def test_snapshot_cannot_promote_unknown_runtime_code(tmp_path):
    _, code, boundary = fixture(tmp_path);boundary.verify()
    boundary.collect_runtime(fallback=lambda:{str(code):digest(code)})
    data = boundary.snapshot();next(iter(data['runtime_maps'].values()))[str(tmp_path/'evil.py')] = 'a'*64
    with pytest.raises(BoundaryInputError, match='nonfixed/changed'):
        SessionInputBoundary.from_verified_snapshot(data, authority_verified=True)


def frozen_modules(root, path, code):
    pins = ModuleType('base5_front_release_inputs');pins.__file__ = str(root/'base5_front_release_inputs.py')
    safety = ModuleType('base5_capture_safety');safety.__file__ = str(root/'base5_capture_safety.py')
    direction = ModuleType('run_base5_capture_campaign_direction_reference');direction.__file__ = str(root/'run_base5_capture_campaign_direction_reference.py')
    alias = ModuleType('frozen_alias');alias.__file__ = str(root/'frozen_alias.py')
    pins.file_sha256 = digest;safety.sha256 = digest;alias.sha256 = digest
    calls = []
    def runtime(*args, **kwargs):calls.append((args, kwargs));return {str(code):pins.file_sha256(code)}
    pins.collect_runtime = runtime;alias.collect_runtime = runtime
    direction.input_pins = lambda value:{str(value):digest(value)}
    return [pins, safety, direction, alias], calls


def test_install_scope_aliases_runtime_mapping_and_late_import_restored(tmp_path):
    path, code, boundary = fixture(tmp_path);boundary.verify()
    boundary.remember_input_pins(path, {str(path):digest(path)})
    modules, calls = frozen_modules(path.parent, path, code)
    pins, safety, direction, alias = modules;original_runtime = pins.collect_runtime
    original_direction = direction.input_pins
    with boundary.installed(scripts_dir=path.parent, modules=modules):
        assert pins.file_sha256(path) == digest(path)
        assert alias.sha256(path) == digest(path)
        assert direction.input_pins(path) == {str(path):digest(path)}
        assert alias.collect_runtime(entrypoints=('a.py',)) == pins.collect_runtime(entrypoints=('a.py',))
        late = ModuleType('late_import');late.__file__ = str(path.parent/'late_import.py')
        late.sha256 = safety.sha256;late.collect_runtime = pins.collect_runtime;modules.append(late)
        retained_wrapper = late.sha256
    assert pins.file_sha256 is digest and alias.sha256 is digest and late.sha256 is digest
    assert late.collect_runtime is original_runtime and direction.input_pins is original_direction
    assert len(calls) == 1
    path.write_text('new bytes after session')
    assert retained_wrapper(path) == digest(path)


def test_install_restores_bindings_when_body_raises(tmp_path):
    path, code, boundary = fixture(tmp_path);boundary.verify()
    modules, _ = frozen_modules(path.parent, path, code)
    with pytest.raises(ValueError, match='body'):
        with boundary.installed(scripts_dir=path.parent, modules=modules):raise ValueError('body')
    assert modules[0].file_sha256 is digest


def test_binding_ownership_violation_restores_everything_and_fails(tmp_path):
    path, code, boundary = fixture(tmp_path);boundary.verify()
    modules, _ = frozen_modules(path.parent, path, code)
    with pytest.raises(BoundaryOwnershipError, match='ownership changed'):
        with boundary.installed(scripts_dir=path.parent, modules=modules):modules[0].file_sha256 = lambda _:None
    assert modules[0].file_sha256 is digest and modules[1].sha256 is digest
    with boundary.installed(scripts_dir=path.parent, modules=modules):pass


def test_unverified_and_nested_install_refused(tmp_path):
    path, code, boundary = fixture(tmp_path)
    modules, _ = frozen_modules(path.parent, path, code)
    with pytest.raises(BoundaryInputError):
        with boundary.installed(scripts_dir=path.parent, modules=modules):pass
    boundary.verify()
    with boundary.installed(scripts_dir=path.parent, modules=modules):
        with pytest.raises(BoundaryOwnershipError):
            with boundary.installed(scripts_dir=path.parent, modules=modules):pass


def delta_fixture(tmp_path):
    source = tmp_path/'source';source.mkdir()
    upper = source/'upper.json';upper.write_text('{"retained": true}')
    proof = tmp_path/'native'/'audit.json';proof.parent.mkdir();proof.write_text('{"proof":true}')
    rows = []
    for index in range(2):
        scenario = source/('scene%d.json'%index);scenario.write_text(json.dumps({'scene':index}))
        meta = source/('meta%d.json'%index);meta.write_text(json.dumps({'ancestor':{'path':str(upper),'sha256':digest(upper)}}))
        rows.append(dict(id='s%d'%index, scenario=str(scenario), scenario_sha256=digest(scenario), metadata=str(meta)))
    base = source/'manifest.json'
    base.write_text(json.dumps(dict(count=2, scenes=rows, config={'native_proof':{'path':str(proof),'sha256':digest(proof)}})))
    fixed = {str(path):digest(path) for path in source.iterdir()}
    boundary = SessionInputBoundary(fixed, forbidden_roots=(tmp_path/'live',));boundary.verify()
    def references(value):
        if type(value) is dict:
            if 'path' in value and 'sha256' in value:yield value['path'],value['sha256']
            for child in value.values():yield from references(child)
        elif type(value) is list:
            for child in value:yield from references(child)
    helper = SimpleNamespace(_references=references, _default_references=lambda _: {},
        _path=lambda value, directory:(Path(value) if Path(value).is_absolute() else directory/value).resolve())
    def original(manifest):
        deck = json.loads(Path(manifest).read_text());result = {};pending = []
        def add(value, expected=None, inspect=False):
            p = helper._path(value, Path(manifest).parent);actual = digest(p)
            assert expected is None or actual == expected
            fresh = str(p) not in result;result[str(p)] = actual
            if fresh and inspect and p.suffix == '.json':pending.append(p)
        add(manifest)
        for row in deck['scenes']:add(row['scenario'],row['scenario_sha256']);add(row['metadata'],inspect=True)
        for reference, expected in references(deck.get('config',{})):add(reference,expected,True)
        while pending:
            for reference, expected in references(json.loads(pending.pop().read_text())):add(reference,expected,True)
        return dict(sorted(result.items()))
    boundary.remember_input_pins(base, original(base))
    live = tmp_path/'live';live.mkdir()
    fresh_scenario = live/'scene.json';fresh_scenario.write_text('{"new":true}')
    fresh_meta = live/'meta.json';fresh_meta.write_text(json.dumps({'root':{'path':str(base),'sha256':digest(base)}}))
    fresh = live/'manifest.json'
    fresh.write_text(json.dumps(dict(count=1, scenes=[dict(id='new', scenario=str(fresh_scenario),
        scenario_sha256=digest(fresh_scenario),metadata=str(fresh_meta))],config={})))
    return boundary, base, fresh, proof, helper, original


def test_delta_mapping_is_original_superset_with_identical_hashes(tmp_path):
    boundary, base, fresh, _, helper, original = delta_fixture(tmp_path)
    exact = original(fresh)
    expanded = boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
    assert set(exact) <= set(expanded)
    assert all(expanded[filename] == expected for filename,expected in exact.items())
    assert all(digest(filename) == expected for filename,expected in expanded.items())
    assert boundary.stats['delta_input_calls'] == 1


def test_delta_does_not_parse_closed_fixed_json_references_again(tmp_path,monkeypatch):
    boundary, base, fresh, _, helper, original = delta_fixture(tmp_path)
    real = Path.read_text;opened = []
    def read(path,*args,**kwargs):opened.append(path);return real(path,*args,**kwargs)
    monkeypatch.setattr(Path,'read_text',read)
    expanded = boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
    assert fresh in opened and base not in opened
    assert all(path.parent == fresh.parent for path in opened)
    assert str(base) in expanded and boundary.stats['delta_base_reuses'] >= 1


@pytest.mark.parametrize('change',['new_scenario','new_metadata','native_proof'])
def test_delta_never_reuses_generated_or_native_bytes(tmp_path,change):
    boundary, base, fresh, proof, helper, original = delta_fixture(tmp_path)
    first = boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
    if change == 'new_scenario':
        path = fresh.parent/'scene.json';path.write_text('{"different":"scene"}')
        with pytest.raises(BoundaryInputError,match='declared SHA'):
            boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
    elif change == 'native_proof':
        proof.write_text('{"changed":"native"}')
        with pytest.raises(BoundaryInputError,match='Nonfixed referenced evidence changed'):
            boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
    else:
        path = fresh.parent/'meta.json';path.write_text('{"different":"metadata"}')
        second = boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
        assert first[str(path)] != second[str(path)] == digest(path)


def test_delta_detects_conflicting_base_reference_hash(tmp_path):
    boundary, base, fresh, _, helper, original = delta_fixture(tmp_path)
    meta = fresh.parent/'meta.json'
    meta.write_text(json.dumps({'root':{'path':str(base),'sha256':'f'*64}}))
    with pytest.raises(BoundaryInputError,match='declared SHA'):
        boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)


def test_delta_requires_explicit_optin_and_keeps_original_exact_mapping_mode(tmp_path):
    boundary, base, fresh, _, helper, original = delta_fixture(tmp_path)
    exact = boundary.input_pins(fresh,fallback=original)
    assert exact == original(fresh)
    expanded = boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
    assert set(exact) < set(expanded)
    child = SessionInputBoundary.from_verified_snapshot(boundary.snapshot(),authority_verified=True)
    assert child.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original) == expanded


def test_delta_scope_incompatible_with_unregistered_base_fails(tmp_path):
    path, code, boundary = fixture(tmp_path);boundary.verify()
    modules, _ = frozen_modules(path.parent,path,code)
    with pytest.raises(BoundaryInputError,match='already registered'):
        with boundary.installed(scripts_dir=path.parent,modules=modules,delta_input_manifest=path):pass


def test_delta_final_recheck_detects_new_source_write_during_discovery(tmp_path,monkeypatch):
    boundary, base, fresh, _, helper, original = delta_fixture(tmp_path)
    target = fresh.parent/'scene.json';real = helper._default_references
    def changed(deck):target.write_text('changed after first hash');return real(deck)
    helper._default_references = changed
    with pytest.raises(BoundaryInputError,match='changed during discovery'):
        boundary.delta_input_pins(fresh,base_manifest=base,references=helper,fallback=original)
