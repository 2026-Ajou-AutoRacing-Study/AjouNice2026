"""CPU-only immutable-input cache fixtures; no ROS, MORAI or live outputs."""
import hashlib
import os
from pathlib import Path
import stat
from types import ModuleType

import pytest

import verified_input_cache as cache_module
from verified_input_cache import (CachePatchOwnershipError, VerifiedInputCache,
                                  VerifiedInputChangedError)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fixture_file(tmp_path, content=b'frozen source bytes\n'):
    filename = tmp_path/'source'/'input.json'
    filename.parent.mkdir()
    filename.write_bytes(content)
    return filename, VerifiedInputCache({str(filename):digest(filename)},
                                        forbidden_roots=[tmp_path/'live'])


def test_exact_sha_reused_only_after_first_verified_read(tmp_path, monkeypatch):
    filename, cache = fixture_file(tmp_path)
    now = cache_module.time.time_ns()
    monkeypatch.setattr(cache_module.time, 'time_ns', lambda:now + 10_000_000_000)
    expected = digest(filename)
    for _ in range(3):
        assert cache.sha256(filename, fallback=digest) == expected
    assert cache.snapshot()['hash_reads'] == 1
    assert cache.snapshot()['hits'] == 2
    assert cache.snapshot()['bytes_read'] == filename.stat().st_size
    assert cache.snapshot()['cached_files'] == 1


def test_recent_inputs_reread_even_if_the_signature_is_unchanged(tmp_path):
    filename, cache = fixture_file(tmp_path)
    first = cache.sha256(filename, fallback=digest)
    assert cache.sha256(filename, fallback=digest) == first
    assert cache.snapshot()['hits'] == 0
    assert cache.snapshot()['hash_reads'] == 2
    assert cache.snapshot()['recent_rechecks'] == 1


def test_other_paths_are_exact_original_passthrough(tmp_path):
    _, cache = fixture_file(tmp_path)
    unusual = object()
    calls = []
    assert cache.sha256(unusual, fallback=lambda value:calls.append(value) or 'original') == 'original'
    assert calls == [unusual]
    missing = tmp_path/'missing'
    with pytest.raises(FileNotFoundError):
        cache.sha256(missing, fallback=digest)
    assert cache.snapshot()['passthrough'] == 2


@pytest.mark.parametrize('change', ['same_length_restored_mtime', 'atomic_replace', 'delete',
                                   'symlink', 'permissions'])
def test_cached_identity_changes_fail_closed(tmp_path, change):
    filename, cache = fixture_file(tmp_path)
    expected = cache.sha256(filename, fallback=digest)
    before = filename.stat()
    if change == 'same_length_restored_mtime':
        filename.write_bytes(b'x'*before.st_size)
        os.utime(filename, ns=(before.st_atime_ns, before.st_mtime_ns))
        assert filename.stat().st_mtime_ns == before.st_mtime_ns
    elif change == 'atomic_replace':
        replacement = filename.with_name('replacement')
        replacement.write_bytes(filename.read_bytes())
        os.utime(replacement, ns=(before.st_atime_ns, before.st_mtime_ns))
        replacement.replace(filename)
    elif change == 'delete':
        filename.unlink()
    elif change == 'symlink':
        target = tmp_path/'target';target.write_bytes(filename.read_bytes())
        filename.unlink();filename.symlink_to(target)
    else:
        filename.chmod(stat.S_IMODE(before.st_mode) ^ stat.S_IXUSR)
    with pytest.raises(VerifiedInputChangedError):
        cache.sha256(filename, fallback=digest)
    with pytest.raises(VerifiedInputChangedError, match='Previously rejected'):
        cache.sha256(filename, fallback=lambda path:expected)


def test_ancestor_replacement_and_ancestor_symlink_are_rejected(tmp_path):
    filename, cache = fixture_file(tmp_path)
    cache.sha256(filename, fallback=digest)
    moved = filename.parent.with_name('moved')
    filename.parent.rename(moved)
    filename.parent.symlink_to(moved, target_is_directory=True)
    with pytest.raises(VerifiedInputChangedError):
        cache.sha256(filename, fallback=digest)


def test_same_file_under_new_ancestor_identity_is_not_a_hit(tmp_path):
    filename, cache = fixture_file(tmp_path)
    cache.sha256(filename, fallback=digest)
    moved = filename.parent.with_name('moved')
    filename.parent.rename(moved);filename.parent.mkdir()
    os.link(moved/filename.name, filename)
    with pytest.raises(VerifiedInputChangedError):
        cache.sha256(filename, fallback=digest)


def test_wrong_expected_sha_rejects_first_read_and_never_caches(tmp_path):
    filename, _ = fixture_file(tmp_path)
    cache = VerifiedInputCache({str(filename):'0'*64})
    with pytest.raises(VerifiedInputChangedError, match='SHA differs'):
        cache.sha256(filename, fallback=digest)
    assert cache.snapshot()['cached_files'] == 0


def test_mutation_during_hash_is_rejected(tmp_path, monkeypatch):
    filename, cache = fixture_file(tmp_path, b'a'*(2*1024*1024))
    original = os.read
    changed = []
    def racing_read(fd, size):
        block = original(fd, size)
        if block and not changed:
            changed.append(True)
            with filename.open('r+b') as stream:
                stream.seek(-1, os.SEEK_END);stream.write(b'z')
        return block
    monkeypatch.setattr(cache_module.os, 'read', racing_read)
    # Either the post-read identity or the pinned expected SHA must reject it;
    # coarse inode timestamp ticks can choose which guard notices the mutation.
    with pytest.raises(VerifiedInputChangedError):
        cache.sha256(filename, fallback=digest)
    assert cache.snapshot()['cached_files'] == 0


def test_replace_between_inspection_and_open_is_rejected(tmp_path, monkeypatch):
    filename, cache = fixture_file(tmp_path)
    original = cache_module._open_nofollow
    def racing_open(path):
        replacement = filename.with_name('replacement')
        replacement.write_bytes(filename.read_bytes());replacement.replace(filename)
        return original(path)
    monkeypatch.setattr(cache_module, '_open_nofollow', racing_open)
    with pytest.raises(VerifiedInputChangedError, match='before reading'):
        cache.sha256(filename, fallback=digest)


@pytest.mark.parametrize('pin', ['relative', 'dotdot', 'wrong_digest', 'non_string_digest', 'live'])
def test_malformed_or_writable_allowlist_rejected(tmp_path, pin):
    filename, _ = fixture_file(tmp_path)
    mapping = {str(filename):digest(filename)}
    if pin == 'relative':mapping = {'relative.json':'0'*64}
    elif pin == 'dotdot':mapping = {str(filename.parent/'..'/'input.json'):'0'*64}
    elif pin == 'wrong_digest':mapping[str(filename)] = 'z'*64
    elif pin == 'non_string_digest':mapping[str(filename)] = 7
    else:mapping = {str(tmp_path/'live'/'output.json'):'0'*64}
    with pytest.raises(ValueError):
        VerifiedInputCache(mapping, forbidden_roots=[tmp_path/'live'])


def modules_fixture(tmp_path):
    scripts = tmp_path/'scripts';scripts.mkdir()
    names = ['base5_front_release_inputs', 'base5_capture_safety', 'frozen_alias']
    modules = []
    for name in names:
        module = ModuleType(name);module.__file__ = str(scripts/(name+'.py'))
        Path(module.__file__).write_text('# fixture module\n')
        modules.append(module)
    def first(path):return digest(path)
    def second(path):return digest(path)
    modules[0].file_sha256 = first;modules[1].sha256 = second
    modules[2].imported_pin = first;modules[2].imported_native = second
    return scripts, modules, first, second


def test_scoped_patches_existing_aliases_and_restores_late_import(tmp_path):
    filename, cache = fixture_file(tmp_path)
    scripts, modules, first, second = modules_fixture(tmp_path)
    outside = ModuleType('outside');outside.__file__ = str(tmp_path/'outside.py');outside.sha256 = first
    modules.append(outside)
    with cache.installed(scripts_dir=scripts, modules=modules):
        assert modules[0].file_sha256 is modules[2].imported_pin
        assert modules[1].sha256 is modules[2].imported_native
        assert outside.sha256 is first
        assert modules[0].file_sha256(filename) == modules[1].sha256(filename) == digest(filename)
        late = ModuleType('late');late.__file__ = str(scripts/'late.py')
        Path(late.__file__).write_text('# late fixture\n')
        late.sha256 = modules[1].sha256;modules.append(late)
        retained = late.sha256
    assert modules[0].file_sha256 is first
    assert modules[1].sha256 is second
    assert modules[2].imported_pin is first and modules[2].imported_native is second
    assert late.sha256 is second
    filename.write_bytes(b'new bytes after the scope')
    assert retained(filename) == digest(filename)  # Old wrappers become passthrough.
    assert cache.snapshot()['hits'] + cache.snapshot()['recent_rechecks'] == 1


def test_scope_restores_on_body_exception(tmp_path):
    _, cache = fixture_file(tmp_path)
    scripts, modules, first, second = modules_fixture(tmp_path)
    with pytest.raises(RuntimeError, match='body failed'):
        with cache.installed(scripts_dir=scripts, modules=modules):
            raise RuntimeError('body failed')
    assert modules[0].file_sha256 is first and modules[1].sha256 is second


def test_ownership_change_not_clobbered_and_other_aliases_restored(tmp_path):
    _, cache = fixture_file(tmp_path)
    scripts, modules, first, second = modules_fixture(tmp_path)
    foreign = lambda path:'foreign'
    with pytest.raises(CachePatchOwnershipError):
        with cache.installed(scripts_dir=scripts, modules=modules):
            modules[0].file_sha256 = foreign
    assert modules[0].file_sha256 is foreign
    assert modules[1].sha256 is second and modules[2].imported_pin is first


def test_nested_scope_rejected_without_unpatching_outer(tmp_path):
    _, cache = fixture_file(tmp_path)
    scripts, modules, first, _ = modules_fixture(tmp_path)
    with cache.installed(scripts_dir=scripts, modules=modules):
        wrapped = modules[0].file_sha256
        with pytest.raises(CachePatchOwnershipError):
            with cache.installed(scripts_dir=scripts, modules=modules):
                pass
        assert modules[0].file_sha256 is wrapped
    assert modules[0].file_sha256 is first


def test_first_read_rejects_symlink_file_or_parent(tmp_path):
    filename, _ = fixture_file(tmp_path)
    expected = digest(filename)
    moved = filename.parent.with_name('moved');filename.parent.rename(moved)
    filename.parent.symlink_to(moved, target_is_directory=True)
    cache = VerifiedInputCache({str(filename):expected})
    with pytest.raises(VerifiedInputChangedError):
        cache.sha256(filename, fallback=digest)
