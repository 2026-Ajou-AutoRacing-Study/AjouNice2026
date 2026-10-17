#!/usr/bin/env python3
"""Explicit, resumable CPU finish and local pull for a completed Base5 batch.

No simulator, GPU, installation, Drive upload, SSH or transfer runs by default.
The finish command preserves all source PNG/native/review files. Its single
archive contains Q90 training inputs and provenance, not a second raw-PNG copy.
Only trusted local repository code, manifests and pickle inputs are supported.
"""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pickle
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import time

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'dataset/morai_od/scripts'
SCHEMA = 'portable_base5_finish_v1'
GIB = 1024 ** 3


def dependency_paths(repo=ROOT):
    """Source-only CPU closure; no Torch/MMCV packages, ops or checkpoints.

    Dataset runtime files are already in the capture bundle's frozen runtime.
    The six extra files include bodies executed through trusted AST extraction.
    """
    repo = Path(repo)
    extras = (
        'perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py',
        'perception/detection3d/bevfusion/tools/prepare_base5_jpeg_portable.py',
        'perception/detection3d/bevfusion/tools/prepare_base5_pedestrian_merged.py',
        'perception/detection3d/bevfusion/scripts/benchmark_base5_jpeg_cache.py',
        'perception/detection3d/bevfusion/plugin/morai_jpeg_cache.py',
        'docker/bevfusion/app/online_preprocessor.py',
    )
    return sorted({Path(__file__).resolve(),
        *[path.resolve() for path in (repo / 'dataset/morai_od/scripts').glob('*.py')],
        *[(repo / value).resolve(strict=True) for value in extras]})


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def within(path, root):
    try:
        Path(path).relative_to(root)
        return True
    except ValueError:
        return False


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 ** 2), b''):
            h.update(chunk)
    return h.hexdigest()


def write_new(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def regular_files(root):
    root = Path(root)
    require(not root.is_symlink(), 'Symlink input is not supported: ' + str(root))
    if root.is_file():
        return [root]
    require(root.is_dir(), 'Missing input: ' + str(root))
    result = []
    for path in sorted(root.rglob('*')):
        require(not path.is_symlink(), 'Source symlink is not supported: ' + str(path))
        if path.is_file():
            result.append(path)
        else:
            require(path.is_dir(), 'Non-regular source: ' + str(path))
    return result


def inventory(roots, metadata=False):
    """Hash hardlinked files once, while preserving every named input binding."""
    result, hashes = {}, {}
    for root in roots:
        for path in regular_files(root):
            path = path.resolve(strict=True)
            stat = path.stat()
            key = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
            if key not in hashes:
                hashes[key] = digest(path)
            item = dict(bytes=stat.st_size, sha256=hashes[key])
            if metadata:
                item.update(inode=stat.st_ino, device=stat.st_dev, mtime_ns=stat.st_mtime_ns)
            result[str(path)] = item
    return result


def verify_inventory(expected, metadata=False):
    actual = inventory([Path(path) for path in expected], metadata)
    require(actual == expected, 'Pinned source/output payload changed')


def selected_rows(path, expected_scenes):
    payload = Path(path).read_bytes()
    require(payload and payload.endswith(b'\n'), 'Strict newline-terminated selected ledger required')
    rows = [json.loads(line) for line in payload.splitlines()]
    require(len(rows) == expected_scenes, 'Incomplete selected ledger; exact expected scene count required')
    for field in ('id', 'capture_root', 'scenario_sha256'):
        require(len({row[field] for row in rows}) == len(rows), 'Duplicate selected ' + field)
    slots = [row.get('source_cell_id', row['id']) for row in rows]
    require(len(set(slots)) == len(slots), 'Duplicate logical selection')
    for row in rows:
        require(row.get('status') == 'accepted_candidate', 'Reviewed/failed row is not a selection')
        raw = Path(row['capture_root']).resolve(strict=True)
        require(digest(raw / 'scenario.json') == row['scenario_sha256'], 'Selected raw scenario changed')
    return rows


def verify_authoritative(source, selected, rows):
    """Use only the existing read-only native-parent verification path.

    Do not call execute/resume on a completion output or create missing children.
    Separate completion roots need a separately qualified verifier, not guessing.
    """
    parent = selected.parent
    require((parent / 'fresh_runtime_contract.json').is_file(),
            'Only an authoritative completed fresh batch is supported; no arbitrary completion merge')
    status, quota = read(parent / 'status.json'), read(parent / 'quota.json')
    require(status.get('stage') == 'complete', 'Finish requires a terminal zero-deficit batch')
    require(quota.get('deficit') == 0 and not quota.get('cells'), 'Unfilled quota cannot be packaged')
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    from run_base5_fresh_completion import verify_parent
    verified, missing, elapsed = verify_parent(source, parent)
    require(verified == rows and not missing, 'Native reconstruction and selected ledger differ')
    return dict(frames=len(rows), deficit=0, elapsed_sec=elapsed,
                verifier='run_base5_fresh_completion.verify_parent/_verification',
                parent=str(parent), selected_sha256=digest(selected))


def allocated_bytes(roots):
    seen, total = set(), 0
    for root in roots:
        root = Path(root)
        if not root.exists():
            continue
        for path in regular_files(root):
            s = path.stat()
            key = s.st_dev, s.st_ino
            if key not in seen:
                seen.add(key)
                total += s.st_blocks * 512
    return total


class Budget:
    """Bound managed on-disk allocation; watch subprocesses and retain partials."""
    def __init__(self, roots, output, maximum, minimum_free):
        self.roots, self.output = list(roots), Path(output)
        self.maximum, self.minimum_free = maximum, minimum_free

    def check(self, reserve=0):
        usage = allocated_bytes(self.roots)
        ancestor = self.output
        while not ancestor.exists():
            ancestor = ancestor.parent
        free = shutil.disk_usage(ancestor).free
        require(usage + reserve <= self.maximum, 'Managed usage exceeds budget; partial products preserved')
        require(free - reserve >= self.minimum_free, 'Insufficient disk safety reserve; partial products preserved')
        return dict(managed_allocated_bytes=usage, free_bytes=free,
                    maximum_bytes=self.maximum, minimum_free_bytes=self.minimum_free)


def run_cpu(command, budget, log):
    """No shell. On resource failure stop only this new derived subprocess."""
    budget.check()
    with Path(log).open('xb') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        next_check = time.monotonic() + 10
        try:
            while process.poll() is None:
                time.sleep(.25)
                # Existing native dirs can contain many files; scan at bounded
                # intervals, in addition to every stage boundary and archive.
                if time.monotonic() >= next_check:
                    budget.check()
                    next_check = time.monotonic() + 10
            require(process.returncode == 0, 'CPU stage failed; inspect immutable attempt log: ' + str(log))
        except BaseException:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            raise
    budget.check()


@contextmanager
def exclusive(path):
    path = Path(path)
    with path.open('a+') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def step(output, name, contract_sha, function):
    marker = output / (name + '.complete.json')
    if marker.exists():
        saved = read(marker)
        require(saved['schema'] == SCHEMA and saved['contract_sha256'] == contract_sha,
                'Completed stage contract differs')
        require(inventory(saved['output_roots']) == saved['outputs'],
                'Completed stage output inventory changed')
        return saved['result']
    number = 0
    while (output / f'{name}.attempt{number:03d}').exists():
        number += 1
    attempt = output / f'{name}.attempt{number:03d}'
    attempt.mkdir()
    result, paths = function(attempt)
    outputs = inventory(paths)
    write_new(marker, dict(schema=SCHEMA, contract_sha256=contract_sha,
                          attempt=str(attempt), output_roots=[str(p) for p in paths],
                          outputs=outputs, result=result))
    return result


def safe_name(name):
    value = Path(name)
    require(not value.is_absolute() and '..' not in value.parts and name not in ('', '.'),
            'Unsafe archive path')
    return str(value)


def package_members(package, provenance):
    result = {}
    for prefix, root in [('dataset', package)] + list(provenance.items()):
        root = Path(root)
        for path in regular_files(root):
            suffix = path.relative_to(root) if root.is_dir() else Path(path.name)
            name = safe_name(str(Path(prefix) / suffix))
            require(name not in result, 'Archive path collision')
            result[name] = path
    return result


def validate_training_package(package, expected_scenes):
    sys.path.insert(0, str(ROOT / 'perception/detection3d/bevfusion/tools'))
    from prepare_base5_portable_dataset import validate
    manifest = validate(package, verify_hashes=True)
    require(manifest['scenes'] == expected_scenes and manifest['profiles'] == ['512x1056']
            and manifest.get('training_label_policy') == 'animal_to_pedestrian_v1'
            and manifest.get('image_policy') == dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'),
            'Qualified portable package policy differs')
    tokens, groups = set(), {}
    for split in ('train', 'val', 'test'):
        infos = pickle.loads((package / f'infos/512x1056/morai_base5_infos_{split}.pkl').read_bytes())
        require(tuple(infos['metainfo']['classes']) == ('vehicle', 'pedestrian', 'obstacle'),
                'Unexpected training classes')
        if split == 'test':
            require(not infos['data_list'], 'Independent test must not be silently fabricated')
        for frame in infos['data_list']:
            require(frame['token'] not in tokens, 'Duplicate training frame')
            tokens.add(frame['token'])
            group = frame['scene_group_id']
            require(group not in groups or groups[group] == split, 'Train/val group leakage')
            groups[group] = split
            require(all(i.get('original_semantic_class') in ('vehicle', 'pedestrian', 'obstacle', 'animal')
                        and i.get('bbox_label_3d') in (0, 1, 2) for i in frame['instances']),
                    'Original component semantic provenance is missing')
    require(len(tokens) == expected_scenes, 'Qualified portable frame count differs')
    return manifest


def archive_single(members, destination, budget, level=10, threads=2):
    require(shutil.which('zstd'), 'zstd is required; automatic installation is forbidden')
    # Worst case near-uncompressed output + tar frames; never stage raw tar or
    # Drive parts beside the compressed archive.
    reserve = sum(p.stat().st_size + 2048 for p in members.values()) + 1024 * 1024
    budget.check(reserve)
    partial = destination.with_suffix(destination.suffix + '.partial')
    require(not destination.exists() and not partial.exists(), 'Fresh single archive path required')
    expected = {name: dict(bytes=p.stat().st_size, sha256=digest(p)) for name, p in members.items()}
    with partial.open('xb') as stream:
        process = subprocess.Popen(['zstd', '-q', f'-{level}', f'-T{threads}', '-c'],
                                   stdin=subprocess.PIPE, stdout=stream)
        try:
            with tarfile.open(fileobj=process.stdin, mode='w|', format=tarfile.PAX_FORMAT) as archive:
                for name, source in sorted(members.items()):
                    require(source.is_file() and not source.is_symlink(), 'Only regular payloads may enter archive')
                    # No filesystem hardlinks/symlinks in the receiving archive.
                    info = tarfile.TarInfo(name)
                    info.size, info.mode = source.stat().st_size, 0o644
                    with source.open('rb') as payload:
                        archive.addfile(info, payload)
            process.stdin.close()
            require(process.wait() == 0, 'Compression failed; partial preserved')
        except BaseException:
            if process.poll() is None:
                process.terminate()
                process.wait()
            raise
        finally:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()
    verify_archive(partial, expected)
    partial.rename(destination)
    budget.check()
    return dict(archive=str(destination), bytes=destination.stat().st_size,
                sha256=digest(destination), members=expected)


def verify_archive(path, expected):
    seen = {}
    process = subprocess.Popen(['zstd', '-q', '-d', '-c', str(path)], stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=process.stdout, mode='r|') as archive:
            for item in archive:
                name = safe_name(item.name)
                require(item.isfile() and name in expected and name not in seen,
                        'Unexpected/unsafe/duplicate archive member')
                h, size = hashlib.sha256(), 0
                with archive.extractfile(item) as data:
                    for chunk in iter(lambda: data.read(4 * 1024 ** 2), b''):
                        h.update(chunk); size += len(chunk)
                seen[name] = dict(bytes=size, sha256=h.hexdigest())
        require(process.wait() == 0 and seen == expected, 'Archive payload verification failed')
    finally:
        if process.poll() is None:
            process.terminate(); process.wait()
        if process.stdout is not None:
            process.stdout.close()


def finish(args):
    global ROOT, SCRIPTS
    if getattr(args, 'repo', None) is not None:
        ROOT = Path(args.repo).resolve(strict=True)
        SCRIPTS = ROOT / 'dataset/morai_od/scripts'
        require(SCRIPTS.is_dir(), 'Repository source is missing')
    source, adopted, selected, sensor = [Path(getattr(args, key)).resolve(strict=True)
                                         for key in ('source', 'adopted', 'selected', 'sensor')]
    output = Path(args.output).resolve()
    rows = selected_rows(selected, args.expected_scenes)
    require(output != ROOT and not within(ROOT, output), 'Output must not contain repository/source')
    protected = [source, adopted, selected, sensor, selected.parent]
    protected += [Path(row['capture_root']).resolve(strict=True) for row in rows]
    require(all(not within(p, output) and not within(output, p)
                for p in protected if p.is_dir()), 'Output and native input roots must not overlap')
    code = dependency_paths(ROOT)
    roots = [Path(p).resolve() for p in args.usage_root] if args.usage_root else []
    roots += [selected.parent, output] + [Path(r['capture_root']) for r in rows]
    budget = Budget(roots, output, args.max_usage_gib * GIB, args.min_free_gib * GIB)
    current = dict(schema=SCHEMA, source=str(source), adopted=str(adopted), selected=str(selected),
        sensor=str(sensor), expected_scenes=args.expected_scenes, max_usage_gib=args.max_usage_gib,
        min_free_gib=args.min_free_gib, output=str(output), repo=str(ROOT),
        compression_level=args.compression_level, threads=args.threads,
        usage_roots=[str(p) for p in roots],
        inputs=inventory(protected, metadata=True), code=inventory(code),
        policy='single archive; source RAW/PNG/review preserved; no remote deletion; no NN/GPU')
    plan = dict(action='finish', execute=bool(args.run), output=str(output),
        frames=len(rows), disk=budget.check(), stages=['native-parent-verification',
        'global-selected-overlay', 'CPU-Q90-upright-threeclass', 'verified-single-tar-zstd'],
        source_unchanged_required=True)
    if not args.run:
        return plan
    output.mkdir(parents=True, exist_ok=True)
    with exclusive(output / '.finish.lock'):
        regular_files(output)  # Never follow an injected symlink in resumable products.
        contract_path = output / 'contract.json'
        if contract_path.exists():
            require(read(contract_path) == current, 'Finish contract changed; use a new dedicated output')
        else:
            require(not any(p.name != '.finish.lock' for p in output.iterdir()), 'Unregistered output is not resumable')
            write_new(contract_path, current)
        contract_sha = digest(contract_path)
        def authoritative(attempt):
            proof = verify_authoritative(source, selected, rows)
            path = attempt / 'proof.json'; write_new(path, proof)
            return proof, [path]
        step(output, '01_native', contract_sha, authoritative)
        def finalize(attempt):
            target = attempt / 'finalized'
            run_cpu([sys.executable, str(SCRIPTS / 'finalize_base5_layout_split.py'),
                '--source', str(source), '--adopted', str(adopted), '--selected', str(selected),
                '--output', str(target)], budget, attempt / 'CPU.log')
            return read(target / 'result.json'), [target, attempt / 'CPU.log']
        final = step(output, '02_overlay', contract_sha, finalize)
        def qualify(attempt):
            target = attempt / 'qualified'
            run_cpu([sys.executable, str(SCRIPTS / 'qualify_base5_training_inputs.py'),
                '--cohort', 'completed=' + final['derived_selected_ledger'], '--output', str(target),
                '--sensor', str(sensor), '--split-overlay', final['split_overlay']],
                budget, attempt / 'CPU.log')
            report = read(target / 'report.json')
            require(report['cohorts']['completed']['frames'] == args.expected_scenes,
                    'Qualified scene count differs')
            return dict(report=str(target / 'report.json'),
                        package=report['cohorts']['completed']['training3_package']), [target, attempt / 'CPU.log']
        qualified = step(output, '03_qualification', contract_sha, qualify)
        def compress(attempt):
            package = Path(qualified['package'])
            validate_training_package(package, args.expected_scenes)
            proof = attempt / 'provenance'; proof.mkdir()
            for name, path in [('source_manifest.json', source), ('adopted_manifest.json', adopted),
                ('source_selected_ledger.jsonl', selected), ('sensor.json', sensor),
                ('finish_contract.json', contract_path), ('qualification_report.json', Path(qualified['report']))]:
                shutil.copy2(path, proof / name)
            provenance = [('provenance', proof), ('finalized_layout', Path(final['manifest']).parent)]
            members = package_members(package, dict(provenance))
            archive = archive_single(members, attempt / 'base5_finished.tar.zst', budget,
                                     args.compression_level, args.threads)
            checksum = attempt / 'SHA256SUMS'
            checksum.write_text(archive['sha256'] + '  base5_finished.tar.zst\n')
            manifest = attempt / 'archive_manifest.json'; write_new(manifest, archive)
            return dict(archive=str(attempt / 'base5_finished.tar.zst'),
                checksum=str(checksum), manifest=str(manifest), sha256=archive['sha256'],
                bytes=archive['bytes']), [attempt / 'base5_finished.tar.zst', checksum, manifest, proof]
        packed = step(output, '04_archive', contract_sha, compress)
        verify_inventory(current['inputs'], metadata=True)
        verify_inventory(current['code'])
        result = dict(schema=SCHEMA, stage='complete', contract_sha256=contract_sha,
            frames=len(rows), archive=packed, source_unchanged=True, disk=budget.check(),
            original_png_and_rejected_raw='preserved at source; not duplicated into training archive',
            remaining='Use pull --run locally; extraction/native lineage relocation is a separate explicit operation')
        complete = output / 'COMPLETE.json'
        if complete.exists():
            saved = read(complete)
            require({k: v for k, v in saved.items() if k != 'disk'} ==
                    {k: v for k, v in result.items() if k != 'disk'}, 'Final completion differs')
            return saved
        write_new(complete, result)
        return result


def pull(args):
    require(re.fullmatch(r'[A-Za-z0-9_.@-]+', args.host or '') is not None
            and not args.host.startswith('-'), 'Use a trusted SSH config host alias, not shell syntax')
    require(1 <= args.port <= 65535, 'Invalid SSH port')
    remote = Path(args.remote_dir)
    require(remote.is_absolute() and re.fullmatch(r'/[A-Za-z0-9_./-]+', str(remote))
            and '..' not in remote.parts, 'Safe absolute remote archive directory required')
    output = Path(args.output).resolve()
    budget = Budget([output], output, args.max_usage_gib * GIB, args.min_free_gib * GIB)
    commands = [['rsync', '-rt', '--checksum', '--partial', '--partial-dir=.rsync-partial',
        '--protect-args', '--info=progress2', '-e', f'ssh -p {args.port} -o BatchMode=yes',
        f'{args.host}:{remote}/{name}', str(output / name)]
        for name in ('SHA256SUMS', 'base5_finished.tar.zst')]
    result = dict(action='local-pull', execute=bool(args.run), commands=commands,
                  no_remote_delete=True, output=str(output))
    if not args.run:
        return result
    output.mkdir(parents=True, exist_ok=True)
    with exclusive(output / '.pull.lock'):
        regular_files(output)
        intent_path = output / 'pull_intent.json'
        intent = dict(schema=SCHEMA, host=args.host, port=args.port, remote_dir=str(remote))
        if intent_path.exists():
            require(read(intent_path) == intent, 'Pull destination belongs to a different remote source')
        else:
            require(not any(p.name != '.pull.lock' for p in output.iterdir()),
                    'Pull requires a new dedicated directory, or its existing intent/contract')
            write_new(intent_path, intent)
        checksum = output / 'SHA256SUMS'
        if not (output / 'RECEIVED.json').exists():
            number = 0
            while (output / f'checksum-transfer-{number:03d}.log').exists():
                number += 1
            run_cpu(commands[0], budget, output / f'checksum-transfer-{number:03d}.log')
        payload = checksum.read_text()
        require(re.fullmatch(r'[a-f0-9]{64}  base5_finished\.tar\.zst\n', payload), 'Unsafe checksum sidecar')
        expected = payload.split()[0]
        pinned = output / 'pull_contract.json'
        contract = dict(host=args.host, port=args.port, remote_dir=str(remote), sha256=expected)
        if pinned.exists():
            require(read(pinned) == contract, 'Pull destination belongs to a different transfer')
        else:
            write_new(pinned, contract)
        target = output / 'base5_finished.tar.zst'
        if not target.exists() or digest(target) != expected:
            attempt = 0
            while (output / f'archive-transfer-{attempt:03d}.log').exists():
                attempt += 1
            run_cpu(commands[1], budget, output / f'archive-transfer-{attempt:03d}.log')
        require(target.is_file() and digest(target) == expected, 'Received archive SHA256 mismatch; remote is preserved')
        receipt = output / 'RECEIVED.json'
        result.update(sha256=expected, bytes=target.stat().st_size, verified=True)
        if receipt.exists():
            require(read(receipt) == result, 'Receiving receipt differs')
        else:
            write_new(receipt, result)
        return result


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    f = sub.add_parser('finish')
    f.add_argument('--repo', type=Path, default=Path(os.environ.get('BASE5_PORTABLE_REPO_ROOT', ROOT)))
    for key in ('source', 'adopted', 'selected', 'sensor', 'output'):
        f.add_argument('--' + key, type=Path, required=True)
    f.add_argument('--expected-scenes', type=int, default=5000)
    f.add_argument('--usage-root', type=Path, action='append', default=[])
    f.add_argument('--compression-level', type=int, choices=range(1, 16), default=10)
    f.add_argument('--threads', type=int, choices=range(1, 9), default=2)
    t = sub.add_parser('pull')
    t.add_argument('--host', required=True)
    t.add_argument('--port', type=int, default=22)
    t.add_argument('--remote-dir', required=True)
    t.add_argument('--output', type=Path, required=True)
    for command in (f, t):
        command.add_argument('--max-usage-gib', type=float, default=100.)
        command.add_argument('--min-free-gib', type=float, default=10.)
        command.add_argument('--run', action='store_true')
    return p


if __name__ == '__main__':
    arguments = parser().parse_args()
    require(arguments.max_usage_gib > 0 and arguments.min_free_gib >= 0, 'Invalid disk budget')
    print(json.dumps(finish(arguments) if arguments.action == 'finish' else pull(arguments), indent=2))
