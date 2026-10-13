#!/usr/bin/env python3
"""Preserve incomplete synchronous payloads for diagnosis, never quota filling."""
import argparse
import json
from pathlib import Path
import shutil

from base5_capture_safety import atomic_json, sha256


def archive(capture, sensor_root, prefix, output):
    if output.exists():
        raise FileExistsError('New failure archive required')
    if not prefix or '/' in prefix or '\\' in prefix:
        raise ValueError('Sensor timestamp prefix only, not a path')
    rows = [json.loads(line) for line in (capture / 'ledger.jsonl').read_text().splitlines()]
    failed = [r for r in rows
              if r['status'] == 'failed' and r.get('error') == 'Synchronous file completeness timeout']
    if len(failed) != 1:
        raise ValueError('Exactly one matching failed request required')
    if not json.loads((capture / 'shutdown_safety.json').read_text()).get('park_verified'):
        raise ValueError('Failed session lacks verified Park; inspect before archival')
    files = sorted(p for p in sensor_root.rglob('*') if p.is_file() and
                   (p.name.startswith(prefix + '_') or p.name == prefix + '.txt'))
    if not files:
        raise ValueError('No partial files with requested timestamp prefix')
    records = []
    for source in files:
        target = output / 'observed_payload' / source.relative_to(sensor_root)
        target.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(source, target)
        records.append(dict(source=str(source.resolve()), archived=str(target.resolve()),
                            sha256=sha256(source), bytes=source.stat().st_size))
        if sha256(target) != records[-1]['sha256']:
            raise ValueError('Partial payload archive checksum mismatch')
    names = [p.name for p in files]
    counts = dict(camera_rgb=sum(p.parent.name.startswith('CAMERA') and p.name.endswith('_Intensity.png') for p in files),
                  camera_instance=sum(p.parent.name.startswith('CAMERA') and p.name.endswith('_Instance.png') for p in files),
                  lidar_intensity=sum(n.endswith('_Intensity.bin') for n in names),
                  lidar_instance_annotation=sum(n.endswith('_Instance.txt') for n in names))
    report = dict(source_capture=str(capture.resolve()), source_ledger_sha256=sha256(capture / 'ledger.jsonl'),
                  failed_request=failed[0], prefix=prefix, observed=counts, files=records,
                  source_files_preserved=True, training_eligible=False,
                  limitation='Operator-specified timestamp group for diagnosing the failed request; '
                             'not a complete native capture or replacement annotation')
    atomic_json(output / 'report.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('capture', 'sensor-root', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--prefix', required=True); args = parser.parse_args()
    result = archive(args.capture, args.sensor_root, args.prefix, args.output)
    print(json.dumps(result['observed'], indent=2))
