#!/usr/bin/env python3
"""Read-only original-byte and source-Git verification for context archives."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = 'docs/maintenance/context_archive_261024.json'


def audit(root=ROOT, manifest=DEFAULT_MANIFEST):
    root = Path(root).resolve()
    errors, domains, operations, destinations = [], Counter(), Counter(), set()
    verified = git_verified = 0
    try:
        data = json.loads((root / manifest).read_text())
        rows = data['records']
        if data['schema'] != 'context_archive_v1' or not isinstance(rows, list) or not rows:
            raise ValueError('invalid schema or empty records')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return dict(errors=[f'Invalid manifest: {exc}'], verified_sha256=0, verified_git=0)
    for row in rows:
        try:
            old, new, sha, commit = (row[k] for k in
                                     ('original_path', 'archive_path', 'sha256', 'source_commit'))
            for name in (old, new):
                if Path(name).is_absolute():
                    raise ValueError(f'path outside root: {name}')
                try:
                    (root / name).resolve().relative_to(root)
                except ValueError:
                    raise ValueError(f'path outside root: {name}')
            if new in destinations:
                raise ValueError(f'duplicate archive destination: {new}')
            destinations.add(new)
            if not re.fullmatch(r'[0-9a-f]{64}', sha) or not re.fullmatch(r'[0-9a-f]{40}', commit):
                raise ValueError(f'invalid checksum/commit: {new}')
            content = (root / new).read_bytes()
            if hashlib.sha256(content).hexdigest() != sha:
                errors.append(f'Archive SHA mismatch: {new}')
            else:
                verified += 1
            result = subprocess.run(['git', 'show', f'{commit}:{old}'], cwd=root,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode:
                errors.append(f'Missing Git original: {commit}:{old}')
            elif hashlib.sha256(result.stdout).hexdigest() != sha:
                errors.append(f'Git original mismatch: {old}')
            else:
                git_verified += 1
            domains[row['domain']] += 1
            operations[row['operation']] += 1
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(f'Invalid archive record: {exc}')
    return dict(errors=errors, records=len(rows), verified_sha256=verified,
                verified_git=git_verified, by_domain=dict(domains), by_operation=dict(operations))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--manifest', default=DEFAULT_MANIFEST)
    args = parser.parse_args()
    result = audit(args.root, args.manifest)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(bool(result['errors']))
