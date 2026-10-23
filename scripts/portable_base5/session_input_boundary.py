"""Reuse explicitly verified fixed inputs only within one parent session.

The caller owns start/resume/end boundaries and authenticated child routing.
Between those boundaries fixed inputs are assumed immutable: a hot fixed lookup
does not stat, hash, or inspect ancestors. Unknown paths and all capture/native
evidence always use the original verifier. This is deliberately not a global
filesystem cache and does not replace any native quality or safety check.
"""
from contextlib import contextmanager
import copy
import functools
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import stat
import sys
import threading
import time
from types import ModuleType


SCHEMA = 'base5_session_fixed_input_boundary_v1'
HEX = re.compile(r'^[0-9a-f]{64}$')
_INSTALL_LOCK = threading.Lock()


class BoundaryInputError(RuntimeError):
    """A boundary or an authenticated fixed-input recipe is invalid."""


class BoundaryOwnershipError(RuntimeError):
    """Another adapter changed a scoped runtime binding."""


def _path(value):
    if not isinstance(value, (str, os.PathLike)):
        raise BoundaryInputError('Explicit file path required')
    value = Path(value).expanduser()
    if not value.is_absolute() or '..' in value.parts:
        raise BoundaryInputError('Absolute non-traversing input path required: '+str(value))
    return str(value)


def _mapping(values):
    if type(values) is not dict or not values:
        raise BoundaryInputError('Nonempty explicit input mapping required')
    result = {}
    for filename, digest in values.items():
        canonical = _path(filename)
        if type(filename) is not str or canonical != filename or type(digest) is not str or not HEX.fullmatch(digest):
            raise BoundaryInputError('Malformed exact absolute input pin: '+str(filename))
        result[filename] = digest
    return dict(sorted(result.items()))


def _inside(filename, root):
    return filename == root or filename.startswith(root.rstrip('/')+'/')


def _native_or_mutable(filename):
    path = Path(filename)
    name = path.name.lower()
    if path.suffix.lower() not in {'.py', '.json', '.yaml', '.yml', '.csv', '.scn'}:
        return True
    if any(part.lower() in {'capture', 'captures', 'raw', 'scene_parks'}
           or part.upper().startswith(('LIDAR_', 'CAMERA_')) for part in path.parts):
        return True
    if path.suffix.lower() != '.py' and any('native' in part.lower() or 'training_inputs' in part.lower()
                                           for part in path.parent.parts):
        return True
    return (name in {'status.json', 'report.json', 'shutdown_safety.json', 'session_contract.json',
                     'live_report.json', 'timing.json', 'quota.json', 'batch_contract.json'}
            or name.startswith(('audit', 'ledger', 'selected', 'receipt', 'event_cmd'))
            or 'receipt' in name or 'snapshot' in name or name.endswith(('_audit.json', '_proof.json')))


def _safe_file(filename):
    path = Path(filename)
    for ancestor in (*reversed(path.parents), path):
        info = ancestor.lstat()
        if stat.S_ISLNK(info.st_mode):
            raise BoundaryInputError('Fixed input path contains a symlink: '+str(ancestor))
    if not stat.S_ISREG(path.lstat().st_mode):
        raise BoundaryInputError('Fixed input must be a regular file: '+filename)
    return path


def _real_sha(filename):
    path = _safe_file(filename)
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(descriptor)
        h = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1024*1024)
            if not block:
                break
            h.update(block)
        after = os.fstat(descriptor)
        current = path.lstat()
        signature = lambda value:(value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns,
                                 value.st_ctime_ns, value.st_mode, value.st_uid, value.st_gid)
        if signature(before) != signature(after) or signature(after) != signature(current):
            raise BoundaryInputError('Fixed input changed during boundary hashing: '+filename)
        _safe_file(filename)
        return h.hexdigest(), after.st_size
    finally:
        os.close(descriptor)


def _argument(value):
    if value is None or type(value) in (bool, int, float, str):
        return [type(value).__name__, value]
    if isinstance(value, os.PathLike):
        return ['path', os.fspath(value)]
    if type(value) in (tuple, list):
        return [type(value).__name__, [_argument(item) for item in value]]
    raise BoundaryInputError('Unsupported runtime discovery argument: '+type(value).__name__)


def _runtime_key(args, kwargs):
    return json.dumps([[_argument(value) for value in args],
                       {name:_argument(value) for name, value in sorted(kwargs.items())}],
                      sort_keys=True, separators=(',', ':'), allow_nan=False)


class SessionInputBoundary:
    def __init__(self, pins, *, forbidden_roots=()):
        self.pins = _mapping(pins)
        self.forbidden_roots = tuple(sorted({_path(value) for value in forbidden_roots}))
        if any(_inside(filename, root) for filename in self.pins for root in self.forbidden_roots):
            raise BoundaryInputError('Live/derived output cannot be declared a fixed input')
        self.fixed_pins = {path:digest for path, digest in self.pins.items() if not _native_or_mutable(path)}
        self._ready = False
        self._input_maps = {}
        self._runtime_maps = {}
        self.stats = dict(boundary_checks=0, boundary_hash_reads=0, boundary_bytes_read=0,
                          boundary_elapsed_sec=0., fixed_hits=0, passthrough=0,
                          input_map_hits=0, runtime_map_hits=0, runtime_map_misses=0,
                          delta_input_calls=0, delta_new_files=0, delta_base_reuses=0)

    def verify(self):
        """Actually read all explicit bytes; call at start/resume and in finally."""
        self._ready = False
        started = time.perf_counter()
        try:
            for filename, expected in self.pins.items():
                actual, size = _real_sha(filename)
                self.stats['boundary_hash_reads'] += 1
                self.stats['boundary_bytes_read'] += size
                if actual != expected:
                    raise BoundaryInputError('Fixed input changed at session boundary: '+filename)
            self._ready = True
            self.stats['boundary_checks'] += 1
        finally:
            self.stats['boundary_elapsed_sec'] += time.perf_counter()-started
        return dict(self.stats)

    def sha256(self, filename, *, fallback):
        try:
            canonical = _path(filename)
        except (BoundaryInputError, TypeError, ValueError):
            canonical = None
        if self._ready and canonical in self.fixed_pins:
            self.stats['fixed_hits'] += 1
            return self.fixed_pins[canonical]
        self.stats['passthrough'] += 1
        return fallback(filename)

    def remember_input_pins(self, manifest, mapping):
        """Register the original function's exact mapping for a fixed manifest."""
        canonical, values = _path(manifest), _mapping(mapping)
        if not self._ready or canonical not in self.fixed_pins or values.get(canonical) != self.fixed_pins[canonical]:
            raise BoundaryInputError('Only a boundary-verified exact fixed manifest may be memoized')
        for filename, digest in values.items():
            if filename in self.pins and self.pins[filename] != digest:
                raise BoundaryInputError('Input discovery conflicts with verified fixed pin: '+filename)
        prior = self._input_maps.get(canonical)
        if prior is not None and prior != values:
            raise BoundaryInputError('Fixed manifest discovery mapping changed')
        self._input_maps[canonical] = values

    def input_pins(self, manifest, *, fallback):
        try:
            canonical = _path(manifest)
        except (BoundaryInputError, TypeError, ValueError):
            canonical = None
        if not self._ready or canonical not in self._input_maps:
            return fallback(manifest)
        values = self._input_maps[canonical]
        # Native proofs or otherwise unknown files referenced by fixed JSON are
        # never promoted into the fixed cache merely because JSON mentions them.
        for filename, expected in values.items():
            if filename not in self.fixed_pins:
                actual, _ = _real_sha(filename)
                self.stats['passthrough'] += 1
                if actual != expected:
                    raise BoundaryInputError('Nonfixed referenced evidence changed: '+filename)
        self.stats['input_map_hits'] += 1
        return dict(values)

    def collect_runtime(self, *args, fallback, **kwargs):
        key = _runtime_key(args, kwargs)
        if self._ready and key in self._runtime_maps:
            self.stats['runtime_map_hits'] += 1
            return dict(self._runtime_maps[key])
        self.stats['runtime_map_misses'] += 1
        result = fallback(*args, **kwargs)
        values = _mapping(result)
        if self._ready and all(self.fixed_pins.get(filename) == digest for filename, digest in values.items()):
            self._runtime_maps[key] = values
        return dict(values)

    def delta_input_pins(self, manifest, *, base_manifest, references, fallback):
        """Opt-in NEW-profile closure: verified full base union actual new files.

        Do not enable while checking historical direction contracts: their exact
        mapping remains the original mapping, not this deliberate superset.
        ``references`` is the original base5_front_release_inputs module. Its
        reference interpretation, declared hashes, and defaults are retained.
        Only already-closed base references skip recursive JSON traversal.
        """
        canonical = _path(manifest)
        if canonical in self._input_maps:
            return self.input_pins(manifest, fallback=fallback)
        base = _path(base_manifest)
        if not self._ready or base not in self._input_maps:
            raise BoundaryInputError('Delta input discovery requires the verified full base mapping')
        # This validates all nonfixed/native evidence in the base each time.
        result = self.input_pins(base, fallback=fallback)
        base_paths = frozenset(result)
        pending, parsed = [], set()

        def load(filename):
            _safe_file(filename)
            return json.loads(Path(filename).read_text())

        def add(filename, expected=None, inspect=False):
            resolved = references._path(filename, Path(canonical).parent)
            filename = _path(resolved)
            if filename in base_paths:
                actual = result[filename]
                self.stats['delta_base_reuses'] += 1
            else:
                actual, _ = _real_sha(filename)
                self.stats['passthrough'] += 1
                self.stats['delta_new_files'] += 1
            if expected is not None and (type(expected) is not str or not HEX.fullmatch(expected)
                                          or actual != expected):
                raise BoundaryInputError('Delta input declared SHA differs: '+filename)
            if filename in result and result[filename] != actual:
                raise BoundaryInputError('Delta input changed during enumeration: '+filename)
            fresh = filename not in result
            result[filename] = actual
            if inspect and fresh and Path(filename).suffix == '.json':
                pending.append(filename)

        deck = load(canonical)
        if type(deck) is not dict or not deck.get('scenes') or deck.get('count') != len(deck['scenes']):
            raise BoundaryInputError('Complete nonempty delta source deck required')
        add(canonical)
        identities = set()
        for entry in deck['scenes']:
            if entry['id'] in identities:
                raise BoundaryInputError('Duplicate delta source ID')
            identities.add(entry['id'])
            add(entry['scenario'], entry['scenario_sha256'])
            add(entry['metadata'], inspect=True)
        for value in (deck.get('config', {}), deck.get('split_overlay', {})):
            for filename, expected in references._references(value):
                add(filename, expected, True)
        for filename, expected in references._default_references(deck).items():
            add(filename, expected)
        while pending:
            filename = pending.pop()
            if filename in parsed:
                continue
            parsed.add(filename)
            for reference, expected in references._references(load(filename)):
                add(reference, expected, True)
        # Do not cache new source files or skip the original final verification
        # of those files. The native/proposal layer can mutate new outputs.
        for filename, expected in result.items():
            if filename not in base_paths:
                actual, _ = _real_sha(filename)
                self.stats['passthrough'] += 1
                if actual != expected:
                    raise BoundaryInputError('Delta source changed during discovery: '+filename)
        self.stats['delta_input_calls'] += 1
        return dict(sorted(result.items()))

    def snapshot(self):
        """Data only. Caller must authenticate its digest/parent before import."""
        if not self._ready:
            raise BoundaryInputError('Unverified boundary cannot be exported')
        return copy.deepcopy(dict(schema=SCHEMA, pins=self.pins, forbidden_roots=list(self.forbidden_roots),
                                  input_maps=self._input_maps, runtime_maps=self._runtime_maps))

    @classmethod
    def from_verified_snapshot(cls, document, *, authority_verified=False):
        """Not a verifier: caller attests authenticated live-parent authority.

        Never use this API merely because an arbitrary JSON says "verified".
        The launcher must bind parent UID/PID/start ticks/argv and snapshot SHA.
        """
        if authority_verified is not True or type(document) is not dict or document.get('schema') != SCHEMA:
            raise BoundaryInputError('Authenticated live-parent snapshot authority required')
        if set(document) != {'schema', 'pins', 'forbidden_roots', 'input_maps', 'runtime_maps'}:
            raise BoundaryInputError('Unknown boundary snapshot fields')
        result = cls(document['pins'], forbidden_roots=document['forbidden_roots'])
        result._ready = True
        if type(document['input_maps']) is not dict or type(document['runtime_maps']) is not dict:
            raise BoundaryInputError('Malformed boundary snapshot caches')
        for manifest, values in document['input_maps'].items():
            result.remember_input_pins(manifest, values)
        for key, values in document['runtime_maps'].items():
            if type(key) is not str:
                raise BoundaryInputError('Malformed runtime discovery key')
            try:
                if json.dumps(json.loads(key), sort_keys=True, separators=(',', ':'), allow_nan=False) != key:
                    raise ValueError('Noncanonical runtime discovery key')
            except (ValueError, TypeError) as error:
                raise BoundaryInputError('Malformed runtime discovery key') from error
            values = _mapping(values)
            if not all(result.fixed_pins.get(filename) == digest for filename, digest in values.items()):
                raise BoundaryInputError('Runtime snapshot contains nonfixed/changed pins')
            result._runtime_maps[key] = values
        return result

    @contextmanager
    def installed(self, *, scripts_dir, modules=None, direction_module=None, delta_input_manifest=None):
        """Patch original bindings in frozen modules only; no source edits.

        The caller must install separately in authenticated subprocesses. Forking
        a patched Python interpreter is not a supported child authority scheme.
        Runtime discovery is memoized by exact arguments, not by entrypoint name.
        """
        if not self._ready:
            raise BoundaryInputError('Verify the session boundary before installing')
        scripts = Path(scripts_dir).resolve(strict=True)
        if not scripts.is_dir() or not _INSTALL_LOCK.acquire(blocking=False):
            raise BoundaryOwnershipError('Another session boundary adapter is active')
        patched, replacements = [], []
        active = [True]

        def candidates():
            collection = list(sys.modules.values()) if modules is None else list(modules)
            result = {}
            for module in collection:
                if isinstance(module, ModuleType) and isinstance(getattr(module, '__file__', None), str):
                    location = Path(module.__file__).resolve()
                    if scripts in location.parents:
                        result[id(module)] = module
            return list(result.values())

        try:
            if modules is None:
                pins = importlib.import_module('base5_front_release_inputs')
                safety = importlib.import_module('base5_capture_safety')
            else:
                values = candidates()
                pins = next((m for m in values if m.__name__ == 'base5_front_release_inputs'), None)
                safety = next((m for m in values if m.__name__ == 'base5_capture_safety'), None)
            if pins not in candidates() or safety not in candidates():
                raise BoundaryInputError('Exact frozen SHA modules required')
            if direction_module is None:
                direction_module = next((m for m in candidates()
                    if m.__name__ == 'run_base5_capture_campaign_direction_reference'), None)
            specifications = [(pins, 'file_sha256', self.sha256), (safety, 'sha256', self.sha256),
                              (pins, 'collect_runtime', self.collect_runtime)]
            if direction_module is not None:
                if direction_module not in candidates():
                    raise BoundaryInputError('Direction module must belong to frozen scripts')
                if delta_input_manifest is None:
                    input_handler = self.input_pins
                else:
                    base = _path(delta_input_manifest)
                    if base not in self._input_maps:
                        raise BoundaryInputError('Delta scope requires an already registered full base mapping')
                    def input_handler(manifest, *, fallback):
                        return self.delta_input_pins(manifest, base_manifest=base,
                                                     references=pins, fallback=fallback)
                specifications.append((direction_module, 'input_pins', input_handler))
            for module, attribute, handler in specifications:
                original = getattr(module, attribute)
                if not callable(original):
                    raise BoundaryInputError('Frozen runtime binding must be callable')
                def make(original, handler):
                    @functools.wraps(original)
                    def wrapped(*args, **kwargs):
                        if not active[0]:
                            return original(*args, **kwargs)
                        return handler(*args, fallback=original, **kwargs)
                    return wrapped
                replacements.append((original, make(original, handler)))
            for module in candidates():
                for attribute, value in list(vars(module).items()):
                    for original, wrapped in replacements:
                        if value is original:
                            setattr(module, attribute, wrapped)
                            patched.append((module, attribute, original, wrapped))
                            break
            yield self
        finally:
            active[0] = False
            changed = [(m.__name__, attribute) for m, attribute, _, wrapped in patched
                       if getattr(m, attribute, None) is not wrapped]
            for module in candidates():
                for attribute, value in list(vars(module).items()):
                    for original, wrapped in replacements:
                        if value is wrapped:
                            setattr(module, attribute, original)
                            break
            for module, attribute, original, _ in patched:
                setattr(module, attribute, original)
            _INSTALL_LOCK.release()
            if changed:
                raise BoundaryOwnershipError('Session boundary adapter ownership changed: '+str(changed))
