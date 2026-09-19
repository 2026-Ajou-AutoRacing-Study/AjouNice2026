#!/usr/bin/env python3
"""Verify and rebuild the provider sources included in this portable release."""
import hashlib
import argparse
import json
from pathlib import Path
import shutil
import subprocess

run = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--provider', choices=['osqp', 'proxsuite'], default='osqp')
provider = parser.parse_args().provider
target = run / ('external/' + provider + '/src')
receipt_path = run / ('provenance/' + provider + '-source-r001.json')
if not target.is_dir():
    raise RuntimeError('Incomplete release: missing included provider source ' + str(target))
receipt = json.loads(receipt_path.read_text())
for name, digest in receipt['files'].items():
    # Repository metadata is deliberately excluded from the vendored source.
    if '.git' in Path(name).parts:
        continue
    if hashlib.sha256((target / name).read_bytes()).hexdigest() != digest:
        raise RuntimeError('Provider source changed: ' + name)
build = run / ('external/' + provider + '/build')
prefix = run / 'external/install'
options = [] if provider == 'osqp' else ['-DBUILD_PYTHON_INTERFACE=OFF',
    '-DBUILD_TESTING=OFF', '-DBUILD_WITH_OPENMP_SUPPORT=OFF',
    '-DBUILD_WITH_VECTORIZATION_SUPPORT=OFF']
subprocess.run(['cmake', '-S', str(target), '-B', str(build),
    '-DCMAKE_INSTALL_PREFIX=' + str(prefix), '-DCMAKE_BUILD_TYPE=Release'] + options, check=True)
subprocess.run(['cmake', '--build', str(build), '--parallel', '4'], check=True)
subprocess.run(['cmake', '--install', str(build)], check=True)
receipt['build_status'] = 'PASS'
(run / 'logs').mkdir(exist_ok=True)
(run / 'logs' / (provider + '-build.json')).write_text(json.dumps(receipt, indent=2) + '\n')
