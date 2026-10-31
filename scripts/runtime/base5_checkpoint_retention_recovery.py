#!/usr/bin/env python3
"""Explicit, external retention-only recovery for a preserved old checkpoint.

This wrapper never edits a checkpoint, frozen source, model, optimizer, or
scheduler. It suppresses exactly ONE LocalBackend.remove call for the pinned
old checkpoint, without attempting unlink. All other removals retain their
original behavior. A durable receipt is mandatory before returning success.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import runpy
import stat
import sys
import threading
import time
import uuid


SCHEMA = 'base5_checkpoint_retention_recovery_v1'
_SHA256 = re.compile(r'^[0-9a-f]{64}$')


class RetentionRecoveryError(RuntimeError):
    pass


def _require(condition, message):
    if not condition:
        raise RetentionRecoveryError(message)


def _absolute(value, field):
    _require(isinstance(value, str) and value and '\x00' not in value,
             'Invalid path field: ' + field)
    p = Path(value)
    _require(p.is_absolute() and os.path.normpath(value) == value,
             'Require an exact absolute path: ' + field)
    return p


def _regular(path):
    s = path.lstat()
    _require(stat.S_ISREG(s.st_mode) and path.resolve(strict=True) == path,
             'Require a regular, nonsymlink canonical file')
    return s


def _sha(path):
    before = _regular(path)
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b''):
            h.update(block)
        opened = os.fstat(stream.fileno())
    after = _regular(path)
    fingerprint = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns,
                             s.st_ctime_ns)
    _require(fingerprint(before) == fingerprint(opened) == fingerprint(after),
             'A pinned file changed while being hashed')
    return h.hexdigest()


def _checked_sha(value, field):
    _require(isinstance(value, str) and _SHA256.fullmatch(value) is not None,
             'Invalid SHA256 field: ' + field)
    return value


def _pid_start_ticks():
    # comm may contain spaces or ')'; fields following the final ')' start at 3.
    tail = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
    return int(tail[19])  # Linux proc(5), field 22: starttime.


def _emit(event, **fields):
    # Never print the environment, authorization document, or exception text.
    try:
        print(json.dumps(dict(event=event, **fields), sort_keys=True),
              file=sys.stderr, flush=True)
    except (OSError, ValueError):
        pass  # A durable disk receipt, not stderr availability, is authoritative.


def _write_receipt(path, value):
    parent = path.parent
    _require(parent.resolve(strict=True) == parent,
             'Receipt directory is no longer canonical')
    data = (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    # Preserve a failed/partial receipt on I/O failure; never unlink evidence.
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    directory_fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    return hashlib.sha256(data).hexdigest()


def _checkpoint(authority, name, work):
    value = authority.get(name)
    _require(isinstance(value, dict), 'Missing checkpoint binding: ' + name)
    path = _absolute(value.get('path'), name + '.path')
    epoch = value.get('epoch')
    _require(type(epoch) is int and epoch > 0 and path.parent == work
             and path.name == f'epoch_{epoch}.pth',
             'Checkpoint must be its named epoch inside the exact work directory')
    checksum = _checked_sha(value.get('sha256'), name + '.sha256')
    _require(_sha(path) == checksum, 'Pinned checkpoint SHA256 differs: ' + name)
    return dict(path=str(path), epoch=epoch, sha256=checksum)


class RetentionGuard:
    """One-shot preservation, with original behavior for every other path."""

    def __init__(self, authority, authority_sha, authority_path, backend_class,
                 receipt_dir, entry):
        self.authority = authority
        self.authority_sha256 = authority_sha
        self.authority_path = authority_path
        self.backend_class = backend_class
        self.original_remove = backend_class.remove
        self.receipt_dir = receipt_dir
        self.entrypoint = entry
        self.skip_count = 0
        self.attempt_count = 0
        self.lock = threading.Lock()
        self.wrapper = None

    def install(self):
        _require(not getattr(self.original_remove, '_base5_retention_guard', False),
                 'A retention recovery guard is already installed')
        target = self.authority['preserved_checkpoint']['path']
        guard = self

        def remove(backend, filepath):
            if os.fspath(filepath) != target:
                return guard.original_remove(backend, filepath)
            return guard._preserve()

        remove._base5_retention_guard = True
        self.wrapper = remove
        self.backend_class.remove = remove

    def _preserve(self):
        with self.lock:
            _require(self.attempt_count == 0,
                     'Repeated removal of the preserved checkpoint is not authorized')
            self.attempt_count += 1
            value = self.authority['preserved_checkpoint']
            path = Path(value['path'])
            # Do not trust only a long-lived stat cache. No unlink is attempted.
            _require(_sha(path) == value['sha256'],
                     'Preserved checkpoint SHA256 changed before retention')
            s = _regular(path)
            receipt = dict(
                schema=SCHEMA, event='PRESERVED_OLD_CHECKPOINT', complete=True,
                time_unix=time.time(), pid=os.getpid(),
                pid_start_ticks=_pid_start_ticks(),
                authority_sha256=self.authority_sha256,
                contract_sha256=self.authority['contract_sha256'],
                preserved_checkpoint=value,
                actual_bytes=s.st_size, device=s.st_dev, inode=s.st_ino,
                unlink_attempted=False, original_remove_called=False,
                retention_only=True, original_checkpoint_bytes_modified=False,
                checkpoint_metadata_modified=False,
                max_keep_ckpts_modified=False,
                note='One additional old checkpoint is retained; every new save still uses the original atomic runner.')
            checksum = _write_receipt(self.receipt_dir / 'PRESERVED_OLD_CHECKPOINT.json',
                                      receipt)
            self.skip_count = 1
            _emit('base5_retention_old_checkpoint_preserved',
                  receipt=str(self.receipt_dir / 'PRESERVED_OLD_CHECKPOINT.json'),
                  receipt_sha256=checksum, unlink_attempted=False)
            return None

    def uninstall(self):
        if self.backend_class.remove is self.wrapper:
            self.backend_class.remove = self.original_remove


def install_guard(authority_path, *, original_entrypoint=None, backend_class=None):
    """Validate pins and durably record installation BEFORE changing remove.

    The caller's outer start authority binds this helper and authority SHA.
    Checkpoint metadata is asserted by that authority and its qualified byte
    SHA, not independently unpickled here. The original runner still performs
    the full model/optimizer/resume validation.
    """
    authority_path = _absolute(str(authority_path), 'authority_path')
    _regular(authority_path)
    raw = authority_path.read_bytes()
    authority_sha = hashlib.sha256(raw).hexdigest()
    a = json.loads(raw)
    _require(isinstance(a, dict) and a.get('schema') == SCHEMA
             and a.get('authorized') is True, 'Explicit retention recovery authority required')
    _checked_sha(a.get('contract_sha256'), 'contract_sha256')
    work = _absolute(a.get('work_dir'), 'work_dir')
    _require(work.is_dir() and work.resolve(strict=True) == work,
             'Work directory must already exist at its canonical path')
    resume = _checkpoint(a, 'resume_checkpoint', work)
    iteration = a['resume_checkpoint'].get('iteration')
    _require(type(iteration) is int and iteration > 0,
             'A positive original checkpoint iteration is required')
    resume['iteration'] = iteration
    preserved = _checkpoint(a, 'preserved_checkpoint', work)
    _require(preserved['epoch'] < resume['epoch'],
             'Only an older checkpoint may be retained')
    pointer = work / 'last_checkpoint'
    _regular(pointer)
    _require(pointer.read_text().strip() == resume['path'],
             'last_checkpoint must still point at the authorized resume checkpoint')
    manifest_path = work / 'run_manifest.json'
    _regular(manifest_path)
    manifest_raw = manifest_path.read_bytes()
    manifest = json.loads(manifest_raw)
    _require(manifest.get('contract_sha256') == a['contract_sha256'],
             'Original work manifest contract differs')
    binding = a.get('original_entrypoint')
    _require(isinstance(binding, dict), 'Original entrypoint binding is required')
    entry = _absolute(binding.get('path'), 'original_entrypoint.path')
    expected_entry_sha = _checked_sha(binding.get('sha256'), 'original_entrypoint.sha256')
    _require(_sha(entry) == expected_entry_sha, 'Original entrypoint SHA256 differs')
    if original_entrypoint is not None:
        _require(_absolute(str(original_entrypoint), 'CLI entrypoint') == entry,
                 'CLI entrypoint differs from its authority')
    if a.get('guard_sha256') is not None:
        _require(_sha(Path(__file__).resolve()) == _checked_sha(a['guard_sha256'], 'guard_sha256'),
                 'External retention guard SHA256 differs')
    root = _absolute(a.get('receipt_root'), 'receipt_root')
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    _require(root.resolve(strict=True) == root, 'Receipt root must be nonsymlink canonical')
    receipt_dir = root / f'guard_{time.time_ns()}_{uuid.uuid4().hex}'
    receipt_dir.mkdir(mode=0o700)
    # Keep only a private normalized copy; never mutate the authority document.
    a = dict(a, resume_checkpoint=resume, preserved_checkpoint=preserved)
    receipt = dict(
        schema=SCHEMA, event='ENTRY_VALIDATED', complete=True,
        pid=os.getpid(), pid_start_ticks=_pid_start_ticks(),
        wandb_console=os.environ.get('WANDB_CONSOLE'),
        authority_sha256=authority_sha, contract_sha256=a['contract_sha256'],
        time_unix=time.time(), resume_checkpoint=resume,
        preserved_checkpoint=preserved,
        original_entrypoint=dict(path=str(entry), sha256=expected_entry_sha),
        run_manifest_sha256=hashlib.sha256(manifest_raw).hexdigest(),
        guard_sha256=_sha(Path(__file__).resolve()),
        checkpoint_metadata_validation='Qualified by the outer authority and exact byte SHA; original runner validation remains required.',
        frozen_source_modified=False, checkpoint_bytes_modified=False,
        checkpoint_metadata_modified=False, training_started_by_install=False,
        unlink_attempted=False)
    receipt_sha = _write_receipt(receipt_dir / 'ENTRY_VALIDATED.json', receipt)
    if backend_class is None:
        from mmengine.fileio.backends.local_backend import LocalBackend
        backend_class = LocalBackend
    guard = RetentionGuard(a, authority_sha, authority_path, backend_class,
                           receipt_dir, entry)
    guard.install()
    _emit('base5_retention_guard_installed', receipt=str(receipt_dir / 'ENTRY_VALIDATED.json'),
          receipt_sha256=receipt_sha, pid=os.getpid())
    return guard


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--authority', type=Path, required=True)
    p.add_argument('command', nargs=argparse.REMAINDER)
    args = p.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    _require(command, 'Provide -- /absolute/original.py and its unchanged arguments')
    _require(os.environ.get('WANDB_CONSOLE') == 'off',
             'This recovery entry requires WANDB_CONSOLE=off from the outer launcher')
    guard = install_guard(args.authority, original_entrypoint=command[0])
    previous_argv, previous_path = sys.argv, list(sys.path)
    sys.argv = list(command)
    if sys.path:
        sys.path[0] = str(guard.entrypoint.parent)
    else:
        sys.path.append(str(guard.entrypoint.parent))
    try:
        runpy.run_path(str(guard.entrypoint), run_name='__main__')
    finally:
        sys.argv = previous_argv
        sys.path[:] = previous_path
        guard.uninstall()


if __name__ == '__main__':
    main()
