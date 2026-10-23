"""Process-local SHA reuse for explicitly pinned, immutable source inputs.

This does not cache native QA, live output, JSON, reference graphs or runtime
closure results. Callers must exclude their current output roots explicitly.
Original files and contracts are never written. Cache hits still inspect every
path component and the file's identity; a changed pin poisons that entry.
"""
from contextlib import contextmanager
import functools
import hashlib
import importlib
import os
from pathlib import Path
import re
import stat
import sys
import threading
import time
from types import ModuleType


HEX = re.compile(r'^[0-9a-f]{64}$')
_INSTALL_LOCK = threading.Lock()
# Linux inode timestamps can remain equal for multiple writes in one clock tick.
# Never reuse a freshly written inode's digest until that ambiguity has passed.
RECENT_CTIME_GUARD_NS = 2_000_000_000


class VerifiedInputChangedError(RuntimeError):
    """A pinned path is missing, unsafe, changed or has unexpected content."""


class CachePatchOwnershipError(RuntimeError):
    """Another adapter changed a binding owned by this scoped installation."""


def _absolute(value):
    raw = os.fspath(value)
    if not isinstance(raw, str) or not os.path.isabs(raw) or os.path.normpath(raw) != raw:
        raise ValueError('An exact normalized absolute path is required: ' + str(raw))
    return raw


def _fingerprint(value):
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns,
            value.st_ctime_ns, value.st_mode, value.st_uid)


def _inspect(filename):
    """No resolve(): reject, rather than follow, every symbolic link."""
    parents = []
    current = os.sep
    for part in (None, *Path(filename).parts[1:-1]):
        if part is not None:
            current = os.path.join(current, part)
        value = os.lstat(current)
        if not stat.S_ISDIR(value.st_mode):
            raise VerifiedInputChangedError('Pinned ancestor is not a real directory: ' + current)
        # Directory contents may legitimately grow; identity must not change.
        parents.append((current, value.st_dev, value.st_ino, value.st_mode, value.st_uid))
    value = os.lstat(filename)
    if not stat.S_ISREG(value.st_mode):
        raise VerifiedInputChangedError('Pinned input is not a nonsymlink regular file: ' + filename)
    return _fingerprint(value), tuple(parents)


def _open_nofollow(filename):
    """Walk from / with directory FDs so intermediate links cannot be followed."""
    directory_flags = (getattr(os, 'O_PATH', os.O_RDONLY) | os.O_DIRECTORY
                       | os.O_NOFOLLOW | os.O_CLOEXEC)
    fd = os.open(os.sep, directory_flags)
    try:
        parts = Path(filename).parts[1:]
        for part in parts[:-1]:
            following = os.open(part, directory_flags, dir_fd=fd)
            os.close(fd)
            fd = following
        return os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
                       | os.O_NONBLOCK, dir_fd=fd)
    finally:
        os.close(fd)


def _inside(filename, root):
    return filename == root or filename.startswith(root.rstrip(os.sep) + os.sep)


class VerifiedInputCache:
    """Only exact allowlisted paths can return a memoized digest.

    ``forbidden_roots`` must include the live capture output and any writable
    cache/output directories used by the launcher. All other paths are delegated
    to the original function without normalization or changed error semantics.
    No cache entry survives this Python process or is transferred to a child.
    """

    def __init__(self, immutable_pins, *, forbidden_roots=()):
        if not isinstance(immutable_pins, dict) or not immutable_pins:
            raise ValueError('An explicit nonempty immutable SHA allowlist is required')
        self.forbidden_roots = tuple(_absolute(value) for value in forbidden_roots)
        self.pins = {}
        for filename, expected in immutable_pins.items():
            if not isinstance(filename, str):
                raise ValueError('Immutable pin names must be absolute strings')
            filename = _absolute(filename)
            if not isinstance(expected, str) or not HEX.fullmatch(expected):
                raise ValueError('Malformed expected SHA for immutable input: ' + filename)
            if any(_inside(filename, root) for root in self.forbidden_roots):
                raise ValueError('Live/writable output cannot be cached: ' + filename)
            self.pins[filename] = expected
        self._entries = {}
        self._failed = set()
        self._lock = threading.RLock()
        self.stats = dict(hits=0, hash_reads=0, bytes_read=0, passthrough=0,
                          failures=0, recent_rechecks=0, elapsed_sec=0.0, hash_elapsed_sec=0.0)

    def snapshot(self):
        with self._lock:
            return dict(self.stats, pinned_files=len(self.pins), cached_files=len(self._entries))

    def sha256(self, filename, *, fallback):
        try:
            name = os.fspath(filename)
        except TypeError:
            name = None
        if name not in self.pins:
            with self._lock:
                self.stats['passthrough'] += 1
            return fallback(filename)
        started = time.perf_counter()
        with self._lock:
            try:
                if name in self._failed:
                    raise VerifiedInputChangedError('Previously rejected immutable input: ' + name)
                before = _inspect(name)
                saved = self._entries.get(name)
                if saved is not None:
                    if before != saved[1]:
                        raise VerifiedInputChangedError('Immutable input identity changed: ' + name)
                    if time.time_ns() - before[0][4] >= RECENT_CTIME_GUARD_NS:
                        self.stats['hits'] += 1
                        return saved[0]
                    self.stats['recent_rechecks'] += 1
                digest = self._read_digest(name, before)
                if digest != self.pins[name]:
                    raise VerifiedInputChangedError('Immutable input SHA differs: ' + name)
                self._entries[name] = (digest, before)
                return digest
            except (OSError, VerifiedInputChangedError) as error:
                self._failed.add(name)
                self.stats['failures'] += 1
                if isinstance(error, VerifiedInputChangedError):
                    raise
                raise VerifiedInputChangedError('Pinned input unavailable or unsafe: ' + name) from error
            finally:
                self.stats['elapsed_sec'] += time.perf_counter() - started

    def _read_digest(self, filename, before):
        started = time.perf_counter()
        fd = _open_nofollow(filename)
        try:
            opened = os.fstat(fd)
            if not stat.S_ISREG(opened.st_mode) or _fingerprint(opened) != before[0]:
                raise VerifiedInputChangedError('Immutable input changed before reading: ' + filename)
            digest = hashlib.sha256()
            self.stats['hash_reads'] += 1
            while True:
                block = os.read(fd, 1024 * 1024)
                if not block:
                    break
                self.stats['bytes_read'] += len(block)
                digest.update(block)
            if _fingerprint(os.fstat(fd)) != before[0] or _inspect(filename) != before:
                raise VerifiedInputChangedError('Immutable input changed while reading: ' + filename)
            return digest.hexdigest()
        finally:
            os.close(fd)
            self.stats['hash_elapsed_sec'] += time.perf_counter() - started

    @contextmanager
    def installed(self, *, scripts_dir, modules=None):
        """Patch only original SHA identities in loaded frozen-script modules.

        Late imports naturally receive the patched canonical function. On exit,
        those aliases are restored too. A retained wrapper outside these modules
        becomes a pure passthrough when the scope ends.
        """
        scripts = Path(scripts_dir).resolve(strict=True)
        if not scripts.is_dir():
            raise ValueError('Frozen scripts directory is required')
        if not _INSTALL_LOCK.acquire(blocking=False):
            raise CachePatchOwnershipError('Another verified input cache scope is already installed')
        patched = []
        wrappers = []
        active = [True]

        def candidates():
            values = list(sys.modules.values()) if modules is None else list(modules)
            unique = {}
            for module in values:
                if not isinstance(module, ModuleType) or not isinstance(getattr(module, '__file__', None), str):
                    continue
                try:
                    location = Path(module.__file__).resolve()
                except (OSError, RuntimeError):
                    continue
                if scripts in location.parents:
                    unique[id(module)] = module
            return list(unique.values())

        try:
            if modules is None:
                sources = [importlib.import_module('base5_front_release_inputs'),
                           importlib.import_module('base5_capture_safety')]
            else:
                sources = candidates()
            originals = []
            for name, attribute in (('base5_front_release_inputs', 'file_sha256'),
                                    ('base5_capture_safety', 'sha256')):
                found = [module for module in sources if module.__name__ == name]
                if len(found) != 1 or found[0] not in candidates():
                    raise ValueError('Exact frozen SHA source module is required: ' + name)
                function = getattr(found[0], attribute)
                if not callable(function):
                    raise ValueError('Original SHA binding is not callable: ' + name)
                if not any(function is old for old in originals):
                    originals.append(function)
            for original in originals:
                def make(original):
                    @functools.wraps(original)
                    def wrapped(filename):
                        if not active[0]:
                            return original(filename)
                        return self.sha256(filename, fallback=original)
                    return wrapped
                wrappers.append((original, make(original)))
            for module in candidates():
                for attribute, value in list(vars(module).items()):
                    for original, wrapped in wrappers:
                        if value is original:
                            setattr(module, attribute, wrapped)
                            patched.append((module, attribute, original, wrapped))
                            break
            yield self
        finally:
            active[0] = False
            changed = [(module.__name__, attribute) for module, attribute, _, wrapped in patched
                       if getattr(module, attribute, None) is not wrapped]
            # Also restore aliases imported into frozen modules during the scope.
            for module in candidates():
                for attribute, value in list(vars(module).items()):
                    for original, wrapped in wrappers:
                        if value is wrapped:
                            setattr(module, attribute, original)
                            break
            _INSTALL_LOCK.release()
            if changed:
                raise CachePatchOwnershipError('Scoped SHA binding ownership changed: ' + str(changed))
