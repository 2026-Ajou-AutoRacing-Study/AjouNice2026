#!/usr/bin/env python3
"""Remove only ZIPs whose EVERY regular entry matches an existing retained file.

Dry-run by default. Hash decompressed members and retained files; CRC alone is
not the identity check. Write evidence before unlinking. Never delete raw bags.
"""
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile


def digest(handle):
    h = hashlib.sha256()
    for block in iter(lambda: handle.read(4 * 1024 * 1024), b''):
        h.update(block)
    return h.hexdigest()


def inspect_zip(archive, retained):
    archive = Path(archive).resolve(strict=True)
    retained = Path(retained).resolve(strict=True)
    report = dict(archive=str(archive), bytes=archive.stat().st_size,
                  retained_root=str(retained), entries=[], verified=False)
    with ZipFile(archive) as z:
        for item in z.infolist():
            if item.is_dir():
                continue
            relative = Path(item.filename)
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('Unsafe ZIP path: ' + item.filename)
            target = (retained / relative).resolve(strict=True)
            target.relative_to(retained)
            if not target.is_file() or target.stat().st_size != item.file_size:
                raise ValueError('Missing or size mismatch: ' + str(target))
            with z.open(item) as handle:
                expected = digest(handle)  # ZipFile also validates CRC.
            with target.open('rb') as handle:
                actual = digest(handle)
            if expected != actual:
                raise ValueError('Content mismatch: ' + str(target))
            report['entries'].append(dict(path=str(target), bytes=item.file_size, sha256=actual))
    if not report['entries']:
        raise ValueError('Empty ZIP')
    report['verified'] = True
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive-dir', type=Path, required=True)
    p.add_argument('--retained-root', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--delete-verified', action='store_true')
    a = p.parse_args()
    if a.report.exists():
        p.error('Use a new evidence report')
    report = dict(deletion_requested=a.delete_verified, archives=[], deleted_bytes=0)
    a.report.parent.mkdir(parents=True, exist_ok=True)
    for archive in sorted(a.archive_dir.glob('*.zip')):
        print('Verifying: ' + archive.name, flush=True)
        try:
            entry = inspect_zip(archive, a.retained_root)
        except Exception as exc:
            entry = dict(archive=str(archive), verified=False, error=repr(exc))
        report['archives'].append(entry)
        a.report.write_text(json.dumps(report, indent=2) + '\n')
        if entry['verified'] and a.delete_verified:
            archive.unlink()
            entry['deleted'] = True
            report['deleted_bytes'] += entry['bytes']
            a.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in entry.items() if k != 'entries'}), flush=True)
    print('Reclaimed bytes: %d' % report['deleted_bytes'])


if __name__ == '__main__':
    main()
